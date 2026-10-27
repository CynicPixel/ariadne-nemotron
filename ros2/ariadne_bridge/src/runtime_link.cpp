// ---------------------------------------------------------------------------
// ariadne_bridge::RuntimeLink (design LIVE_RUNTIME_L1 D150). The only ROS code of the live runtime.
//
// Loaded into the SAME component container as ros_gz_bridge::RosGzBridge, with intra-process comms, so a bridged
// message reaches this node without a DDS hop (D40: every measured loss of the old recording path was on that hop).
//   GZ -> ROS topics of the bridge YAML (except the skipped ones, e.g. /clock): typed subscription -> CDR bytes
//     (rclcpp::Serialization, the bytes a rosbag2 MCAP would hold) -> one link frame on the Unix socket.
//   ROS -> GZ topics (the commands): link frame from the runtime -> deserialize -> typed publish -> the bridge.
// It never interprets a message. The runtime decodes them with its own ROS-free decoder (runtime/replay/ros_cdr).
//
// Parameters: bridge_config (the YAML the bridge loads), socket_path, ready_file, skip_topics (string array).
// Readiness: when every forwarded topic has a publisher and every command topic a subscriber, a #ready frame goes
// to the runtime and ready_file is written; the launcher unpauses the world only then.
// Lockstep (D165): a service entry of the YAML (ros_gz_interfaces/srv/ControlWorld, bridged to /world/<w>/control)
// gives a client; a "#run_to" frame from the runtime (int64 sim ns, little-endian) becomes a request with only
// run_to_sim_time set: gz runs to that time and pauses itself. The answer goes back as "#run_to_ack" JSON with the sim
// clock at the moment of the request (/clock, subscribed here, not forwarded): the time the world was paused at.
// ---------------------------------------------------------------------------
#include "runtime/live/link_frame.hpp"
#include "runtime/live/message_queue.hpp"
#include <cstdlib>
#include <condition_variable>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>

namespace ariadne_bridge {

namespace live = ariadne::runtime::live;

class RuntimeLink : public rclcpp::Node {
public:
    explicit RuntimeLink(const rclcpp::NodeOptions& opts) : rclcpp::Node("ariadne_runtime_link", opts) {
        const std::string config = declare_parameter<std::string>("bridge_config");
        const std::string socket_path = declare_parameter<std::string>("socket_path");
        ready_file_ = declare_parameter<std::string>("ready_file", "");
        const auto skip = declare_parameter<std::vector<std::string>>("skip_topics", std::vector<std::string>{"/clock"});
        const std::set<std::string> skipped(skip.begin(), skip.end());

        const char* latest = std::getenv("ARIADNE_LIVE_LATEST");
        asynchronous_ = latest && std::string(latest) == "1";
        control_out_.ConfigureSensor(64ULL * 1024 * 1024);
        if (asynchronous_) {
            // Excluded from the component container's executor: image serialization
            // cannot prevent clock, IMU and supervisory callbacks from running.
            control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
            control_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
            control_executor_->add_callback_group(control_group_, get_node_base_interface());
        }
        fd_ = live::Connect(socket_path, std::chrono::seconds(120));
        RCLCPP_INFO(get_logger(), "connected to the runtime at %s", socket_path.c_str());

        for (const auto& e : YAML::LoadFile(config)) {
            if (e["service_name"]) {
                const std::string type = e["ros_type_name"].as<std::string>();
                if (type != "ros_gz_interfaces/srv/ControlWorld") {
                    throw std::runtime_error("RuntimeLink: unsupported service type " + type);
                }
                control_ = create_client<ros_gz_interfaces::srv::ControlWorld>(e["service_name"].as<std::string>());
                continue;
            }
            const std::string topic = e["ros_topic_name"].as<std::string>();
            const std::string type = e["ros_type_name"].as<std::string>();
            const std::string dir = e["direction"].as<std::string>();
            if (dir == "GZ_TO_ROS") {
                if (skipped.count(topic)) continue;
                Subscribe(topic, type);
            } else if (dir == "ROS_TO_GZ") {
                if (type != "geometry_msgs/msg/PoseArray") {
                    throw std::runtime_error("RuntimeLink: unsupported command type " + type + " on " + topic);
                }
                cmd_pubs_[topic] = create_publisher<geometry_msgs::msg::PoseArray>(topic, rclcpp::QoS(100).reliable());
            } else {
                throw std::runtime_error("RuntimeLink: unsupported direction " + dir);
            }
        }
        // D167 timers: /clock (bridged but never forwarded) feeds a (sim ns, wall) ring; each forwarded message's
        // bridge latency = wall at forward - wall when /clock first reached its stamp. Aggregates go out in #stats.
        rclcpp::SubscriptionOptions clock_options;
        clock_options.callback_group = control_group_;
        clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(100), [this](const rosgraph_msgs::msg::Clock::SharedPtr m) {
                const int64_t t = int64_t(m->clock.sec) * 1000000000 + m->clock.nanosec;
                clock_ns_ = t;
                const auto now = std::chrono::steady_clock::now();
                {
                    std::lock_guard<std::mutex> lk(lat_mu_);
                    clock_ring_.emplace_back(t, now);
                    if (clock_ring_.size() > 8192) clock_ring_.pop_front();
                }
                if (asynchronous_ && now - last_clock_sent_ >= std::chrono::milliseconds(100)) {
                    const std::string js = "{\"sim_ns\":" + std::to_string(t) + "}";
                    Send("#clock", reinterpret_cast<const uint8_t*>(js.data()), js.size());
                    last_clock_sent_ = now;
                }
            }, clock_options);
        if (asynchronous_) sender_ = std::thread([this] { SendQueued(); });
        reader_ = std::thread([this] { ReadCommands(); });
        ready_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { CheckReady(); }, control_group_);
        stats_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { SendStats(); }, control_group_);
        if (control_executor_) control_thread_ = std::thread([this] { control_executor_->spin(); });
    }

    ~RuntimeLink() override {
        stop_ = true;
        if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
        send_cv_.notify_all();
        if (control_executor_) control_executor_->cancel();
        if (control_thread_.joinable()) control_thread_.join();
        if (reader_.joinable()) reader_.join();
        if (sender_.joinable()) sender_.join();
        if (fd_ >= 0) ::close(fd_);
    }

private:
    template <typename T>
    void SubscribeTyped(const std::string& topic) {
        auto ser = std::make_shared<rclcpp::Serialization<T>>();
        // One buffer per topic, ZEROED before each serialization: the CDR serializer skips alignment padding (e.g. the
        // bytes between frame_id and the next 8-byte field), so a fresh buffer carries whatever memory held there and
        // two live runs of one seed recorded different bytes for identical messages (D165 check, 2026-10-23). Message
        // sizes are constant per topic, so after the first message the zeroed buffer is never reallocated; the first
        // one is serialized, zeroed and serialized again. The callback is never re-entered (one callback group).
        auto sm = std::make_shared<rclcpp::SerializedMessage>();
        constexpr bool image = std::is_same_v<T, sensor_msgs::msg::Image>;
        rclcpp::SubscriptionOptions options;
        if (asynchronous_ && !image) options.callback_group = control_group_;
        // Demo images prioritize freshness at ROS ingress. Pair integrity is enforced
        // by the subsequent stereo queue. Preserve the original lossless profile.
        auto qos = rclcpp::QoS(asynchronous_ && image ? 2 : 500).reliable();
        if (asynchronous_ && image) qos.best_effort();
        subs_.push_back(create_subscription<T>(topic, qos,
                                               [this, topic, ser, sm](const typename T::SharedPtr m) {
                                                   auto& r = sm->get_rcl_serialized_message();
                                                   const size_t cap = r.buffer_capacity;
                                                   if (cap) std::memset(r.buffer, 0, cap);
                                                   ser->serialize_message(m.get(), sm.get());
                                                   if (r.buffer_capacity != cap) {   // grown: fresh memory, redo
                                                       std::memset(r.buffer, 0, r.buffer_capacity);
                                                       ser->serialize_message(m.get(), sm.get());
                                                   }
                                                   Send(topic, r.buffer, r.buffer_length);
                                                   const int64_t st = StampNs(*m);
                                                   if (st >= 0) NoteLatency(topic, st);
                                               }, options));
        received_[topic] = 0;
    }

    void Subscribe(const std::string& topic, const std::string& type) {
        if (asynchronous_ && type == "sensor_msgs/msg/Image") {
            const auto side = topic.find("/left/") != std::string::npos ? topic.find("/left/") : topic.find("/right/");
            if (side == std::string::npos) throw std::runtime_error("demo image topic must name a stereo side");
            const auto prefix = topic.substr(0, side);
            auto& queue = image_out_[prefix];
            if (!queue) {
                queue = std::make_shared<live::MessageQueue>();
                queue->ConfigureSensor(64ULL * 1024 * 1024, prefix + "/left/image_raw", prefix + "/right/image_raw");
            }
            image_topic_[topic] = queue;
        }
        if (type == "sensor_msgs/msg/Image") SubscribeTyped<sensor_msgs::msg::Image>(topic);
        else if (type == "sensor_msgs/msg/CameraInfo") SubscribeTyped<sensor_msgs::msg::CameraInfo>(topic);
        else if (type == "sensor_msgs/msg/Imu") SubscribeTyped<sensor_msgs::msg::Imu>(topic);
        else if (type == "geometry_msgs/msg/PoseStamped") SubscribeTyped<geometry_msgs::msg::PoseStamped>(topic);
        else if (type == "rosgraph_msgs/msg/Clock") SubscribeTyped<rosgraph_msgs::msg::Clock>(topic);
        else if (type == "std_msgs/msg/String") SubscribeTyped<std_msgs::msg::String>(topic);
        else throw std::runtime_error("RuntimeLink: unsupported sensor type " + type + " on " + topic);
        forwarded_.push_back(topic);
    }

    template <typename T, typename = void>
    struct HasHeader : std::false_type {};
    template <typename T>
    struct HasHeader<T, std::void_t<decltype(std::declval<T>().header)>> : std::true_type {};

    template <typename T>
    static int64_t StampNs(const T& m) {
        if constexpr (HasHeader<T>::value) return int64_t(m.header.stamp.sec) * 1000000000 + m.header.stamp.nanosec;
        else return -1;
    }

    void NoteLatency(const std::string& topic, int64_t stamp_ns) {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(lat_mu_);
        const auto it = std::lower_bound(clock_ring_.begin(), clock_ring_.end(), stamp_ns,
                                         [](const auto& e, int64_t v) { return e.first < v; });
        if (it == clock_ring_.end()) {   // /clock has not caught the stamp up yet: no reference, skip the sample
            ++lat_skipped_;
            return;
        }
        const double ms = std::chrono::duration<double, std::milli>(now - it->second).count();
        auto& a = lat_[topic];
        ++a.n, a.sum_ms += ms, a.max_ms = std::max(a.max_ms, ms);
    }

    void SendQueued() {
        while (!stop_) {
            uint64_t seen;
            { std::lock_guard<std::mutex> lk(send_mu_); seen = send_generation_; }
            bool sent = false;
            // Bound control bursts, then serve each agent once; neither image
            // traffic nor a continuous IMU stream can starve the other class.
            for (int i = 0; i < 32 && !stop_; ++i) {
                auto m = control_out_.TryPop(); if (!m) break;
                WireSend(*m); sent = true;
            }
            for (const auto& [agent, queue] : image_out_) {
                (void)agent;
                if (stop_) break;
                if (auto m = queue->TryPop()) { WireSend(*m); sent = true; }
            }
            if (!sent) {
                std::unique_lock<std::mutex> lk(send_mu_);
                send_cv_.wait(lk, [&] { return stop_ || send_generation_ != seen; });
            }
        }
    }
    void WireSend(const live::StreamMessage& m) {
        const bool ok = live::WriteFrame(fd_, m.topic, m.data->data(), m.data->size());
        { std::lock_guard<std::mutex> lk(write_mu_);
          if (m.topic[0] != '#') ++received_[m.topic];
          if (!ok) ++dropped_[m.topic]; }
        if (!ok) { lost_peer_ = true; stop_ = true; ::shutdown(fd_, SHUT_RDWR); send_cv_.notify_all(); }
    }
    void Send(const std::string& topic, const uint8_t* data, size_t n) {
        if (asynchronous_) {
            if (stop_) return;
            live::StreamMessage m;
            m.topic = topic;
            m.data = std::make_shared<const std::vector<uint8_t>>(data, data + n);
            const auto image = image_topic_.find(topic);
            if (image != image_topic_.end()) {
                if (n < 12) throw std::runtime_error("short stereo image header");
                int32_t sec; uint32_t nsec;
                std::memcpy(&sec, data + 4, 4); std::memcpy(&nsec, data + 8, 4);
                m.stamp_ns = int64_t(sec) * 1000000000 + nsec;
            }
            try {
                (image != image_topic_.end() ? *image->second : control_out_).Push(std::move(m));
            } catch (const std::exception& e) {
                RCLCPP_FATAL(get_logger(), "bounded sensor sender: %s", e.what());
                stop_ = true; ::shutdown(fd_, SHUT_RDWR); send_cv_.notify_all();
                rclcpp::shutdown();
                return;
            }
            { std::lock_guard<std::mutex> lk(send_mu_); ++send_generation_; }
            send_cv_.notify_one();
            return;
        }
        std::lock_guard<std::mutex> lk(write_mu_);
        if (topic[0] != '#') ++received_[topic];
        if (!live::WriteFrame(fd_, topic, data, n)) {
            if (!lost_peer_.exchange(true)) RCLCPP_ERROR(get_logger(), "runtime link closed: messages are now dropped");
            ++dropped_[topic];
        }
    }

    void ReadCommands() {
        rclcpp::Serialization<geometry_msgs::msg::PoseArray> ser;
        live::LinkFrame f;
        try {
            while (!stop_ && live::ReadFrame(fd_, f)) {
                if (f.topic == "#run_to") {
                    RunTo(f);
                    continue;
                }
                const auto it = cmd_pubs_.find(f.topic);
                if (it == cmd_pubs_.end()) {
                    RCLCPP_ERROR(get_logger(), "runtime sent a frame for unknown topic '%s'", f.topic.c_str());
                    continue;
                }
                rclcpp::SerializedMessage sm(f.payload.size());
                auto& r = sm.get_rcl_serialized_message();
                std::memcpy(r.buffer, f.payload.data(), f.payload.size());
                r.buffer_length = f.payload.size();
                geometry_msgs::msg::PoseArray msg;
                ser.deserialize_message(&sm, &msg);
                it->second->publish(msg);
                ++commands_;
            }
        } catch (const std::exception& e) {
            if (!stop_) RCLCPP_ERROR(get_logger(), "command link: %s", e.what());
        }
    }

    void RunTo(const live::LinkFrame& f) {
        int64_t t = 0;
        if (!control_ || f.payload.size() != sizeof t) {
            RCLCPP_ERROR(get_logger(), "#run_to without a control service, or a bad payload");
            return;
        }
        std::memcpy(&t, f.payload.data(), sizeof t);
        auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
        req->world_control.run_to_sim_time.sec = int32_t(t / 1000000000);
        req->world_control.run_to_sim_time.nanosec = uint32_t(t % 1000000000);
        const int64_t paused = clock_ns_.load();
        control_->async_send_request(req, [this, t, paused](rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedFuture r) {
            const std::string s = "{\"run_to_ns\":" + std::to_string(t) + ",\"paused_clock_ns\":" + std::to_string(paused) +
                                  ",\"accepted\":" + (r.get()->success ? "true" : "false") + "}";
            Send("#run_to_ack", reinterpret_cast<const uint8_t*>(s.data()), s.size());
        });
    }

    void CheckReady() {
        if (ready_) return;
        if (control_ && !control_->service_is_ready()) return;
        std::ostringstream js;
        js << "{\"forwarded\":[";
        for (size_t i = 0; i < forwarded_.size(); ++i) {
            if (count_publishers(forwarded_[i]) == 0) return;
            js << (i ? "," : "") << "\"" << forwarded_[i] << "\"";
        }
        js << "],\"commands\":[";
        bool first = true;
        for (const auto& [t, p] : cmd_pubs_) {
            if (p->get_subscription_count() == 0) return;
            js << (first ? "" : ",") << "\"" << t << "\"";
            first = false;
        }
        js << "]}";
        const std::string s = js.str();
        Send("#ready", reinterpret_cast<const uint8_t*>(s.data()), s.size());
        if (!ready_file_.empty()) std::ofstream(ready_file_) << s << "\n";
        ready_ = true;
        RCLCPP_INFO(get_logger(), "ready: %s", s.c_str());
    }

    void SendStats() {
        std::ostringstream js;
        std::unique_lock<std::mutex> lk(write_mu_);   // the counters are written under it (Send)
        js << "{\"commands_published\":" << commands_.load() << ",\"received\":{";
        bool first = true;
        for (const auto& [t, n] : received_) {
            js << (first ? "" : ",") << "\"" << t << "\":" << n;
            first = false;
        }
        js << "},\"bridge_latency_ms\":{";
        {
            std::lock_guard<std::mutex> lat_lk(lat_mu_);
            bool f2 = true;
            for (const auto& [t, a] : lat_) {
                js << (f2 ? "" : ",") << "\"" << t << "\":{\"mean\":" << (a.n ? a.sum_ms / double(a.n) : 0)
                   << ",\"max\":" << a.max_ms << ",\"n\":" << a.n << "}";
                f2 = false;
            }
            js << "},\"latency_samples_skipped\":" << lat_skipped_ << ",\"dropped_link_closed\":{";
        }
        first = true;
        for (const auto& [t, n] : dropped_) {
            js << (first ? "" : ",") << "\"" << t << "\":" << n;
            first = false;
        }
        js << "},\"sensor_sender\":{\"asynchronous\":" << (asynchronous_ ? "true" : "false") << ",\"agents\":[";
        bool first_agent = true;
        for (const auto& [agent, queue] : image_out_) {
            js << (first_agent ? "" : ",") << "{\"agent\":\"" << agent << "\",\"dropped_pairs\":"
               << queue->dropped_pairs() << ",\"dropped_incomplete\":" << queue->dropped_incomplete()
               << ",\"peak_bytes\":" << queue->max_bytes() << "}";
            first_agent = false;
        }
        js << "]}}";
        const std::string s = js.str();
        lk.unlock();
        Send("#stats", reinterpret_cast<const uint8_t*>(s.data()), s.size());
    }

    int fd_ = -1;
    std::string ready_file_;
    std::mutex write_mu_;
    std::atomic<bool> stop_{false}, lost_peer_{false};
    bool ready_ = false;
    std::atomic<long> commands_{0};
    std::vector<std::string> forwarded_;
    std::map<std::string, long> received_, dropped_;   // guarded by write_mu_
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> cmd_pubs_;
    std::thread reader_, sender_, control_thread_;
    rclcpp::CallbackGroup::SharedPtr control_group_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> control_executor_;
    bool asynchronous_ = false;
    live::MessageQueue control_out_;
    std::map<std::string, std::shared_ptr<live::MessageQueue>> image_out_, image_topic_;
    std::mutex send_mu_;
    std::condition_variable send_cv_;
    uint64_t send_generation_ = 0;
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr control_;   // lockstep only
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    struct Lat {
        long n = 0;
        double sum_ms = 0, max_ms = 0;
    };
    std::mutex lat_mu_;
    std::deque<std::pair<int64_t, std::chrono::steady_clock::time_point>> clock_ring_;   // guarded by lat_mu_
    std::map<std::string, Lat> lat_;                                                     // guarded by lat_mu_
    long lat_skipped_ = 0;                                                               // guarded by lat_mu_
    std::atomic<int64_t> clock_ns_{-1};
    std::chrono::steady_clock::time_point last_clock_sent_{};  // control-group-owned
    rclcpp::TimerBase::SharedPtr ready_timer_, stats_timer_;
};

}  // namespace ariadne_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ariadne_bridge::RuntimeLink)
