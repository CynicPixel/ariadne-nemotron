#include "runtime/live/collab_live.hpp"

#include <gtest/gtest.h>

TEST(CollabSnapshot, PublishedByteCountsStayFrozenAsTheLedgerAdvances) {
    using namespace ariadne;
    core::ByteLedger ledger;
    core::ByteRecord record;
    record.type = core::MessageClass::HEARTBEAT;
    record.wire_bytes = 100;
    ledger.Record(record);
    runtime::live::CollabSnapshot old;
    old.CaptureBytes(ledger);
    ledger.Record(record);
    record.type = core::MessageClass::TASK_BID;
    record.wire_bytes = 70;
    ledger.Record(record);
    runtime::live::CollabSnapshot current;
    current.CaptureBytes(ledger);
    EXPECT_EQ(old.Bytes(core::MessageClass::HEARTBEAT), 100);
    EXPECT_EQ(old.Bytes(core::MessageClass::TASK_BID), 0);
    EXPECT_EQ(current.Bytes(core::MessageClass::HEARTBEAT), 200);
    EXPECT_EQ(current.Bytes(core::MessageClass::TASK_BID), 70);
}

TEST(CollabSnapshot, BeforeTheFirstPublishedTickAllByteClassesAreZero) {
    const ariadne::runtime::live::CollabSnapshot initial;
    EXPECT_EQ(initial.Bytes(ariadne::core::MessageClass::HEARTBEAT), 0);
    EXPECT_EQ(initial.Bytes(ariadne::core::MessageClass::SUBMAP_PATCH), 0);
}
