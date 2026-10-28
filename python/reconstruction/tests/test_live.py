"""Phase 6 live path, CPU parts (reconstruction.md, D190/D193): submap frames and re-anchoring (exit-gate bullet 2's algebra),
depth seeding, chunk export. No GPU, no torch."""
import json

import numpy as np
import pytest

from ariadne_recon.live.export import SPLAT_BYTES, chunk_bytes, table
from ariadne_recon.live.seeding import SeedParams, seed
from ariadne_recon.live.submaps import FrameStore, SubmapManager, inv_se3


def se3(yaw, t):
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = t
    return T


def test_submaps_close_at_max_and_hold_anchor_relative_poses():
    m = SubmapManager(max_keyframes=3)
    for k in range(7):
        sm, opened = m.add_keyframe(0, k, se3(0.1 * k, [k, 0, 0]), pose_version=1)
        assert opened == (k % 3 == 0)
    assert [len(s.keyframes) for s in m.submaps] == [3, 3, 1]
    assert [s.closed for s in m.submaps] == [True, True, False]
    s1 = m.submaps[1]
    assert s1.anchor_kf == 3
    np.testing.assert_allclose(s1.T_anchor_C[3], np.eye(4), atol=1e-12)
    np.testing.assert_allclose(s1.T_O_anchor @ s1.T_anchor_C[5], se3(0.5, [5, 0, 0]), atol=1e-12)


def test_training_frame_cache_is_scoped_to_submap_and_close_isolated():
    """Agents can both emit kf 18; closing one submap must preserve the other's frame and optimizer state."""
    from types import SimpleNamespace
    from ariadne_recon.live.trainer import LiveTrainer

    trainer = LiveTrainer.__new__(LiveTrainer)
    trainer.frames = {(3, 18): {"agent": 0, "sid": 3}, (7, 18): {"agent": 1, "sid": 7}}
    trainer.state = {
        3: SimpleNamespace(opt=SimpleNamespace(state={"old": "optimizer state"})),
        7: SimpleNamespace(opt=SimpleNamespace(state={"keep": "optimizer state"})),
    }
    assert trainer._frame(SimpleNamespace(sid=3), 18)["agent"] == 0
    assert trainer._frame(SimpleNamespace(sid=7), 18)["agent"] == 1

    trainer._close(3)
    assert (3, 18) not in trainer.frames
    assert trainer._frame(SimpleNamespace(sid=7), 18)["agent"] == 1
    assert trainer.state[3].opt.state == {}
    assert trainer.state[7].opt.state == {"keep": "optimizer state"}


def test_reanchor_moves_the_placement_and_nothing_inside():
    """Exit-gate bullet 2: a PGO update mid-training re-anchors submaps with zero retraining -- the Gaussians in the
    local frame are unchanged, only the placement moves, and it moves exactly by the anchor's correction."""
    m = SubmapManager(max_keyframes=2)
    for k in range(4):
        m.add_keyframe(0, k, se3(0.0, [k, 0, 0]), pose_version=1)
    gaussians_local = np.random.default_rng(0).normal(size=(100, 3))
    before = gaussians_local.copy()
    member_before = {k: T.copy() for k, T in m.submaps[1].T_anchor_C.items()}
    T_W_O = se3(0.3, [1, 2, 0])
    corr = se3(0.05, [0.1, -0.2, 0])
    moved = m.reanchor(0, {2: corr @ se3(0.0, [2, 0, 0]), 3: se3(1.0, [9, 9, 9])}, pose_version=2)
    assert moved == [1]                                   # kf 3 is not an anchor: ignored (no intra-submap fix)
    np.testing.assert_array_equal(gaussians_local, before)
    for k, T in m.submaps[1].T_anchor_C.items():
        np.testing.assert_array_equal(T, member_before[k])
    np.testing.assert_allclose(m.submaps[1].placement(T_W_O), T_W_O @ corr @ se3(0.0, [2, 0, 0]), atol=1e-12)
    np.testing.assert_allclose(m.submaps[0].placement(T_W_O), T_W_O, atol=1e-12)   # untouched submap
    with pytest.raises(ValueError, match="older"):
        m.reanchor(0, {2: np.eye(4)}, pose_version=1)


def test_seeding_backprojects_into_the_anchor_frame_and_respects_alpha_and_rho():
    """Gaussian-SLAM's recipe (D198): a later keyframe seeds only where alpha < 0.6; points with an existing Gaussian
    within rho are dropped; the scale is the nearest-neighbour distance."""
    h, w, f = 40, 60, 50.0
    K = np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1]])
    Z = np.full((h, w), 2.0, np.float32)
    Z[0, :] = np.nan                                     # no depth: never seeded
    S = np.full((h, w), 0.01, np.float32)
    S[1, :] = 0.2                                        # sigma above the gate
    A = np.full((h, w), 0.4, np.float32)
    A[2, :] = np.nan                                     # undefined albedo: never seeded
    alpha = np.zeros((h, w), np.float32)
    alpha[:, : w // 2] = 0.9                             # left half already opaque
    T = se3(0.2, [1, 0, 0])
    p = SeedParams(uniform_fraction=1.0, gradient_fraction=0.0, rho_m=1e-4)
    far = np.array([[100.0, 100.0, 100.0]], np.float32)  # a non-empty submap far away: a LATER keyframe
    s = seed(Z, S, A, K, T, alpha, far, p, np.random.default_rng(0))
    r, c = s.pixels[:, 0], s.pixels[:, 1]
    assert (r >= 3).all() and (c >= w // 2).all()
    assert len(s.means) == (h - 3) * (w - w // 2)
    X_C = np.stack([(c - K[0, 2]) / f * 2.0, (r - K[1, 2]) / f * 2.0, np.full(len(r), 2.0)], 1)
    np.testing.assert_allclose(s.means, X_C @ T[:3, :3].T + T[:3, 3], atol=1e-5)
    np.testing.assert_allclose(s.scales, 2.0 / f, rtol=1e-5)    # a full grid: the nearest neighbour is one footprint
    np.testing.assert_allclose(s.colors[:, 0], 0.4)
    # rho: existing Gaussians at those exact points suppress every seed.
    assert len(seed(Z, S, A, K, T, alpha, s.means, p, np.random.default_rng(0)).means) == 0
    # The first keyframe ignores alpha (nothing rendered yet) and adds colour-gradient points.
    first = seed(Z, S, A, K, T, alpha, None, SeedParams(), np.random.default_rng(0))
    assert (first.pixels[:, 1] < w // 2).any()


def test_seeding_fractions_follow_gaussian_slam_and_are_deterministic():
    h, w = 60, 80
    K = np.array([[40.0, 0, 39.5], [0, 40.0, 29.5], [0, 0, 1]])
    Z = np.full((h, w), 1.5, np.float32)
    S = np.full((h, w), 0.01, np.float32)
    img = np.zeros((h, w, 3), np.float32)
    img[:, w // 2:] = 1.0                                # one vertical edge: the high-gradient region
    p = SeedParams(rho_m=1e-4)
    a = seed(Z, S, img, K, np.eye(4), None, None, p, np.random.default_rng(3))
    b = seed(Z, S, img, K, np.eye(4), None, None, p, np.random.default_rng(3))
    np.testing.assert_array_equal(a.pixels, b.pixels)
    n = h * w
    u = round(100_000 / 307_200 * n)                     # M_u / TUM pixels
    assert u <= len(a.means) <= u + round(50_000 / 307_200 * n)
    later = seed(Z, S, img, K, np.eye(4), np.zeros((h, w), np.float32), a.means, p, np.random.default_rng(3))
    assert len(later.means) < len(a.means)               # rho against what the first keyframe seeded


def test_chunk_is_32_bytes_per_gaussian_in_the_anchor_frame_and_table_places_it():
    n = 5
    params = {"means": np.arange(15, dtype=np.float32).reshape(5, 3), "scales": np.log(np.full((n, 3), 0.02, np.float32)),
              "quats": np.tile(np.array([1, 0, 0, 0], np.float32), (n, 1)), "opacities": np.zeros(n, np.float32),
              "sh0": np.zeros((n, 1, 3), np.float32)}
    b = chunk_bytes(params)
    assert len(b) == SPLAT_BYTES * n
    pos = np.frombuffer(b, dtype=[("p", "<f4", 3), ("s", "<f4", 3), ("c", "u1", 4), ("r", "u1", 4)])["p"]
    np.testing.assert_array_equal(pos, params["means"])                       # not moved: the anchor frame
    m = SubmapManager()
    m.add_keyframe(1, 7, se3(0.0, [1, 0, 0]), pose_version=4)
    t = json.loads(table(m.submaps, {1: se3(0.0, [0, 5, 0])}, {0: n}, "albedo proxy"))
    assert t["label"] == "albedo proxy" and t["submaps"][0]["bytes"] == 32 * n
    np.testing.assert_allclose(np.reshape(t["submaps"][0]["T_W_anchor"], (4, 4)), se3(0.0, [1, 5, 0]))
    np.testing.assert_allclose(inv_se3(se3(0.4, [1, 2, 3])) @ se3(0.4, [1, 2, 3]), np.eye(4), atol=1e-12)


def _spool_v2_one_agent(tmp, n=3):
    """n keyframes of agent 0 with an albedo proxy (schema 2), the writer's exact format."""
    import cv2
    import yaml
    h, w, f = 24, 32, 20.0
    (tmp / "agent0").mkdir(parents=True)
    (tmp / "lamp_calibration.yaml").write_text("name: t\n")
    half = lambda p, a: cv2.imwrite(str(p), a.astype(np.float16).view(np.uint16))
    bundles = []
    for k in range(n):
        stem = f"agent0/kf_{k}"
        cv2.imwrite(str(tmp / f"{stem}_rgb.png"), np.zeros((h, w, 3), np.uint8))
        half(tmp / f"{stem}_depth.png", np.full((h, w), 1.5, np.float32))
        half(tmp / f"{stem}_sigma.png", np.full((h, w), 0.005, np.float32))
        half(tmp / f"{stem}_albedo.png", np.full((h, w), 0.4, np.float32))
        T = se3(0.0, [0.3 * k, 0, 0])
        bundles.append({"role": "keyframe", "agent": 0, "kf_id": k, "t_ns": k, "frame_id_of_pose": 1,
                        "T_WiC": [float(x) for x in T[:3, :].ravel()], "K": [f, f, (w - 1) / 2, (h - 1) / 2],
                        "width": w, "height": h, "baseline_m": 0.1, "files": stem, "rgb_kind": "albedo_proxy",
                        "exposure_state": {"exposure_gain": 1.0, "decision_id": 0}, "lamp_state": {"lamp_rel": 1.0},
                        "mask": "none"})
    y = {"schema_version": 2, "run_id": "r", "scene_id": "s", "lamp": {
        "calibration": "lamp_calibration.yaml", "calibration_source": "x", "calibration_name": "t", "rig": "r",
        "intensity": 6.0, "R_rect_raw": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
        "merge": {"ok": False, "T_O0_O1": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0], "edge": "none"}, "bundles": bundles}
    (tmp / "spool.yaml").write_text(yaml.safe_dump(y))
    return tmp


def test_service_cpu_init_writes_table_and_chunks_labelled(tmp_path):
    from ariadne_recon.live import service
    spool = _spool_v2_one_agent(tmp_path / "spool")
    out = tmp_path / "splat"
    assert service.main(["--spool", str(spool), "--out", str(out), "--backend", "cpu_init",
                         "--training-image", "albedo_proxy"]) == 0
    t = json.loads((out / "table.json").read_text())
    assert t["label"] == "albedo proxy — initialisation only, no GPU" and t["backend"] == "cpu_init"
    (row,) = t["submaps"]
    chunk = out / "chunks" / row["chunk"]
    assert chunk.stat().st_size == row["bytes"] == SPLAT_BYTES * row["gaussians"] > 0
    assert sorted(p.name for p in (out / "chunks").iterdir()) == [row["chunk"]]     # replaced chunks are removed
    assert len((out / "service_log.jsonl").read_text().splitlines()) == 3
    m = json.loads((out / "manifest.json").read_text())
    assert m["splat_backend"] == "cpu_init" and m["spool_schema"] == [2]


def test_service_refuses_a_keyframe_without_the_proxy(tmp_path):
    import yaml
    from ariadne_recon.live import service
    spool = _spool_v2_one_agent(tmp_path / "spool")
    y = yaml.safe_load((spool / "spool.yaml").read_text())
    y["bundles"][1]["rgb_kind"] = "raw_radiance"
    (spool / "spool.yaml").write_text(yaml.safe_dump(y))
    with pytest.raises(SystemExit, match="never a fallback"):
        service.main(["--spool", str(spool), "--out", str(tmp_path / "o"), "--backend", "cpu_init",
                      "--training-image", "albedo_proxy"])


def _scene(tmp, starts):
    """A briefing: entrances (xy, yaw deg), route start z 1.0, the sim rig's optical rotation and left offset."""
    (tmp / "routes").mkdir(parents=True)
    (tmp / "scene.json").write_text(json.dumps({"entrances": [
        {"agent": k, "start_xy_m": [x, y], "start_yaw_deg": yaw} for k, (x, y, yaw) in enumerate(starts)]}))
    (tmp / "mission_default.json").write_text(json.dumps({
        "R_B_optical_rows": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], "t_B_camera_m": {"left": [0.03, 0.05, 0.02]}}))
    for k in range(len(starts)):
        (tmp / "routes" / f"agent_{k}.json").write_text(json.dumps({"segments": [{"start": [0, 0, 1.0, 0, 0]}]}))
    return tmp


def _spool_v3_rgb(tmp, ok1=True):
    """Schema 3, two agents x 2 RGB keyframes (lit, no photo state), agent 1 merged by a 2 m shift (or not)."""
    import cv2
    import yaml
    h, w, f = 24, 32, 20.0
    half = lambda p, a: cv2.imwrite(str(p), a.astype(np.float16).view(np.uint16))
    bundles = []
    for agent in (0, 1):
        (tmp / f"agent{agent}").mkdir(parents=True)
        for k in range(2):
            stem = f"agent{agent}/kf_{k}"
            cv2.imwrite(str(tmp / f"{stem}_rgb.png"), np.full((h, w, 3), 128, np.uint8))
            half(tmp / f"{stem}_depth.png", np.full((h, w), 1.5, np.float32))
            half(tmp / f"{stem}_sigma.png", np.full((h, w), 0.005, np.float32))
            T = se3(0.0, [0.2 * k, 0, 0])
            bundles.append({"role": "keyframe", "agent": agent, "kf_id": k, "t_ns": 10 * k + agent,
                            "frame_id_of_pose": 1, "T_WiC": [float(x) for x in T[:3, :].ravel()],
                            "K": [f, f, (w - 1) / 2, (h - 1) / 2], "width": w, "height": h, "baseline_m": 0.1,
                            "files": stem, "rgb_kind": "raw_radiance", "exposure_state": "not_modeled",
                            "lamp_state": "inert", "mask": "none"})
    y = {"schema_version": 3, "run_id": "r", "scene_id": "s", "bundles": bundles, "merges": [
        {"agent": 1, "ok": ok1, "T_O0_O": [1, 0, 0, 2.0, 0, 1, 0, 0, 0, 0, 1, 0], "edge": "agent0 kf 1 <- agent1 kf 0"}]}
    (tmp / "spool.yaml").write_text(yaml.safe_dump(y))
    return tmp


def test_briefing_places_the_odometry_frame_at_the_entrance_times_the_rig_extrinsic(tmp_path):
    from ariadne_recon.live.placement import briefing_T_W_O
    scene = _scene(tmp_path / "scene", [(1.0, 2.0, 90.0)])
    T = briefing_T_W_O(scene, 0)
    # body yawed 90 deg at (1, 2, 1): the camera offset (0.03, 0.05, 0.02) turns into (-0.05, 0.03, 0.02)
    np.testing.assert_allclose(T[:3, 3], [1.0 - 0.05, 2.0 + 0.03, 1.02], atol=1e-12)
    # optical z (forward) = body x = world +y after the yaw; optical x (right) = body -y = world +x
    np.testing.assert_allclose(T[:3, 2], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(T[:3, 0], [1, 0, 0], atol=1e-12)


def test_briefing_follows_the_live_demo_entrance_choice(tmp_path):
    from ariadne_recon.live.placement import briefing_T_W_O
    scene = _scene(tmp_path / "scene", [(1.0, 2.0, 0.0), (5.0, 2.0, 180.0), (9.0, 2.0, 90.0)])
    plain = [briefing_T_W_O(scene, k) for k in range(3)]
    m = json.loads((scene / "mission_default.json").read_text())
    m["live"] = {"demo": {"entrances": [2, 0], "target_xy_m": [0, 0]}}         # mission agent 0 ran entrance 2
    (scene / "mission_live.json").write_text(json.dumps(m))
    np.testing.assert_allclose(briefing_T_W_O(scene, 0), plain[2], atol=1e-12)
    np.testing.assert_allclose(briefing_T_W_O(scene, 1), plain[0], atol=1e-12)


def _rows(out):
    t = json.loads((out / "table.json").read_text())
    return t, {r["agent"]: np.asarray(r["T_W_anchor"]).reshape(4, 4) for r in t["submaps"]}


def test_service_places_each_agent_by_its_own_briefing_or_by_the_merge(tmp_path):
    from ariadne_recon.live import service
    from ariadne_recon.live.placement import briefing_T_W_O
    spool = _spool_v3_rgb(tmp_path / "spool")
    scene = _scene(tmp_path / "scene", [(1.0, 2.0, 0.0), (5.0, 2.0, 180.0)])
    common = ["--spool", str(spool), "--backend", "cpu_init", "--training-image", "rgb", "--scene", str(scene)]
    assert service.main(common + ["--out", str(tmp_path / "b")]) == 0                       # default: briefing
    _, rows = _rows(tmp_path / "b")
    for k in (0, 1):   # each submap is anchored at its agent's kf 0 = the identity in O_k
        np.testing.assert_allclose(rows[k], briefing_T_W_O(scene, k), atol=1e-12)
    assert service.main(common + ["--out", str(tmp_path / "m"), "--placement", "merges"]) == 0
    _, rows = _rows(tmp_path / "m")
    shift = np.eye(4)
    shift[0, 3] = 2.0
    np.testing.assert_allclose(rows[1], briefing_T_W_O(scene, 0) @ shift, atol=1e-12)    # agent 1's entrance unused
    m = json.loads((tmp_path / "m" / "manifest.json").read_text())
    assert m["placement"]["mode"] == "merges" and m["placement"]["unplaced"] == {}


def test_an_agent_without_an_ok_merge_is_never_drawn(tmp_path):
    from ariadne_recon.live import service
    spool = _spool_v3_rgb(tmp_path / "spool", ok1=False)
    out = tmp_path / "o"
    assert service.main(["--spool", str(spool), "--out", str(out), "--backend", "cpu_init",
                         "--training-image", "rgb"]) == 0                                     # no scene: merges
    t, rows = _rows(out)
    assert set(rows) == {0}
    m = json.loads((out / "manifest.json").read_text())
    assert m["keyframes"] == 2 and "1" in m["placement"]["unplaced"]
    assert m["placement"]["world"] == "agent 0 odometry frame"


def test_set_placement_moves_the_table_without_new_chunks(tmp_path):
    from ariadne_recon.live.service import Service, _CpuInit
    from ariadne_recon.live.seeding import SeedParams
    svc = Service(tmp_path / "live", "cpu_init", "rgb", _CpuInit(3, 30, SeedParams(), 0))
    svc.set_placement(0, np.eye(4))
    with pytest.raises(ValueError, match="no placement"):
        svc.ingest(1, 0, np.eye(4), 1, np.eye(3), np.zeros((4, 4, 3), np.float32), np.ones((4, 4)), np.ones((4, 4)))
    svc.ingest(0, 0, np.eye(4), 1, np.array([[4.0, 0, 1.5], [0, 4.0, 1.5], [0, 0, 1]]),
               np.full((4, 4, 3), 0.5, np.float32), np.full((4, 4), 1.5, np.float32), np.full((4, 4), 0.005, np.float32))
    svc.finish()
    chunks = sorted(p.name for p in (tmp_path / "live" / "chunks").iterdir())
    v = json.loads((tmp_path / "live" / "table.json").read_text())["version"]
    T = se3(0.3, [1, 2, 3])
    svc.set_placement(0, T)
    t = json.loads((tmp_path / "live" / "table.json").read_text())
    assert t["version"] == v + 1 and sorted(p.name for p in (tmp_path / "live" / "chunks").iterdir()) == chunks
    np.testing.assert_allclose(np.asarray(t["submaps"][0]["T_W_anchor"]).reshape(4, 4), T, atol=1e-12)


def test_one_spool_per_agent_is_briefing_only_and_never_repeats_an_agent(tmp_path):
    import yaml
    from ariadne_recon.live import service
    parts = []
    for agent in (0, 1):   # split a 2-agent spool into per-agent spools, as ariadne_sim_vo --spool writes them
        sp = _spool_v3_rgb(tmp_path / f"s{agent}")
        y = yaml.safe_load((sp / "spool.yaml").read_text())
        y["bundles"] = [b for b in y["bundles"] if b["agent"] == agent]
        (sp / "spool.yaml").write_text(yaml.safe_dump(y))
        parts.append(str(sp))
    scene = _scene(tmp_path / "scene", [(1.0, 2.0, 0.0), (5.0, 2.0, 180.0)])
    base = ["--backend", "cpu_init", "--training-image", "rgb", "--scene", str(scene)]
    assert service.main(["--spool", *parts, "--out", str(tmp_path / "b")] + base) == 0
    _, rows = _rows(tmp_path / "b")
    assert set(rows) == {0, 1}
    with pytest.raises(ValueError, match="one composed spool"):
        service.main(["--spool", *parts, "--out", str(tmp_path / "m"), "--placement", "merges"] + base)
    with pytest.raises(SystemExit, match="one spool per agent"):
        service.main(["--spool", parts[0], parts[0], "--out", str(tmp_path / "d")] + base)


def test_paced_replay_waits_for_each_keyframe_mission_time(tmp_path, monkeypatch):
    from ariadne_recon.live import service
    waits = []
    monkeypatch.setattr(service.time, "sleep", lambda s: waits.append(s))
    spool = _spool_v3_rgb(tmp_path / "spool")              # keyframes at t = 0, 1, 10, 11 ns
    assert service.main(["--spool", str(spool), "--out", str(tmp_path / "o"), "--backend", "cpu_init",
                         "--training-image", "rgb", "--pace", "1e-9"]) == 0   # 1 ns of mission = 1 s of wall
    assert len(waits) == 3 and max(waits) <= 11.0 and min(waits) > 0     # one wait per later keyframe, none longer
    assert json.loads((tmp_path / "o" / "manifest.json").read_text())["pace"] == 1e-9


def test_frame_store_keeps_agents_with_colliding_keyframe_ids_apart():
    """The 626abe g4dn crash (KeyError 999): every agent numbers its own keyframes from 0, so a store keyed by kf alone
    lost one agent's frame when another agent's submap closed, and overwrote images before that."""
    m, fs = SubmapManager(max_keyframes=3), FrameStore()
    for k in range(5):                  # interleaved like the service: agent 0's kf k, then agent 1's kf k
        for agent in (0, 1):
            sm, opened = m.add_keyframe(agent, k, se3(0.0, [k, agent, 0]), pose_version=1)
            if opened:                  # the trainer closes the agent's previous submap when a new one opens
                for old in [s for s in m.submaps if s.agent == agent and s.closed]:
                    fs.drop_submap(old.sid)
            fs.put(sm.sid, k, (agent, k))
    for agent in (0, 1):
        sm = m.active(agent)
        for k in sm.keyframes:          # every keyframe the trainer may sample is present, and is THIS agent's
            assert fs.get(sm.sid, k) == (agent, k)
    assert len(fs) == sum(len(m.active(a).keyframes) for a in (0, 1))   # closed submaps released, nothing leaked


def test_spool_train_kernel_fails_when_a_budget_fails(tmp_path):
    """A failed budget fails the run and stops the later budgets (the 626abe run reported rc 0 with its live budget
    crashed). CPU only: stub torch/gsplat, and a spool the service cannot load."""
    import os
    import subprocess
    import sys
    from pathlib import Path

    stub = tmp_path / "stub"
    (stub / "torch").mkdir(parents=True)
    (stub / "torch" / "__init__.py").write_text(
        "__version__ = '0+stub'\nclass _V: cuda = '0'\nversion = _V()\n"
        "class _C:\n    @staticmethod\n    def is_available(): return False\ncuda = _C()\n")
    (stub / "gsplat").mkdir()
    (stub / "gsplat" / "__init__.py").write_text("__version__ = 'stub'\n")
    (stub / "gsplat" / "csrc.py").write_text("")
    inp = tmp_path / "in"
    (inp / "spools" / "agent0").mkdir(parents=True)
    (inp / "spools" / "agent0" / "spool.yaml").write_text("not: a spool\n")
    (inp / "scene").mkdir()
    (inp / "scene" / "scene.json").write_text("{}")
    work = tmp_path / "work"
    work.mkdir()
    root = Path(__file__).resolve().parents[1]
    env = dict(os.environ, PYTHONPATH=str(stub), ARIADNE_WORK=str(work), ARIADNE_INPUT=str(inp),
               ARIADNE_PKG=str(root), ARIADNE_BUDGETS='{"first": [15, 2], "second": 15}')
    r = subprocess.run([sys.executable, str(root / "kaggle" / "spool_train_kernel.py")], env=env,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 1, r.stdout + r.stderr
    out = json.loads((work / "remote.json").read_text())
    assert list(out["budgets"]) == ["first"] and out["budgets"]["first"]["rc"] != 0


def test_live_iteration_prediction_is_capped_and_handles_bad_timings():
    from types import SimpleNamespace
    from ariadne_recon.live.trainer import LiveTrainer, TrainParams
    trainer = LiveTrainer.__new__(LiveTrainer)
    trainer.p = TrainParams(budget_ms=50)
    assert trainer._iterations(SimpleNamespace(ms_iter=[])) == 3
    assert trainer._iterations(SimpleNamespace(ms_iter=[10.0])) == 5
    assert trainer._iterations(SimpleNamespace(ms_iter=[0.00001])) == 64
    for value in (0.0, -1.0, float("nan")):
        assert trainer._iterations(SimpleNamespace(ms_iter=[value])) == 3
    trainer.p = TrainParams(iters=100)
    assert trainer._iterations(SimpleNamespace(ms_iter=[10.0])) == 100
