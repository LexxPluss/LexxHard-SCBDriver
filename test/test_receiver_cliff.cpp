// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// SCBDriver cliff ingress, deliberately stopping at the typed frame boundary:
//
//   can_frame -> shared registry route -> production decoder -> typed sink
//
// No ROS publisher, cycle state machine, watchdog or readiness decision is present here.
// The measurement payloads below were printed by firmware 3.6.0-102-g17fa922 from a real
// VL53L4CX on dasher1; the health payload was captured from can1 on the same robot. The
// flashed full-slot DFU artefact was D1_PACK.test.bin, SHA-256
// 92da8a500baa28e596ef5e399dc3c8cb076b1659c89aea6ead077f836b48a7eb.
//
// TWO ARTEFACT IDENTITIES, AND THE DIFFERENCE IS NOT IN THESE BYTES.
//   at capture time : contract fb94706a..., artefact db9cae64...
//   this decoder    : contract fb94706a..., artefact 3db018e9...
// The contract SHA is the same one, so the payloads below still describe the same wire format
// and are unchanged from what the robot produced. Only the artefact-set identity moved, and only
// because the generator's clang-format guard moved to the first line of the emitted headers --
// no vector, encoding, status-table entry or payload byte differs. The old identity is kept here
// rather than overwritten because it is what the capture was taken against, and a provenance note
// that silently adopts today's identity stops being provenance.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "can_ids.hpp"
#include "receiver_cliff.hpp"

namespace ids = lexxhard::can_ids;
namespace ctr = tof_cliff_contract;
namespace fr = tof_cliff_frame;

namespace
{

can_frame frame(uint32_t id, const std::array<uint8_t, 8>& bytes, uint8_t dlc = 8)
{
  can_frame out{};
  out.can_id = id;
  out.can_dlc = dlc;
  std::memcpy(out.data, bytes.data(), bytes.size());
  return out;
}

class recording_sink final : public lexxhard::receiver_cliff_sink
{
public:
  void on_measurement(const fr::measurement& value) override
  {
    measurements.push_back(value);
  }

  void on_health(const fr::health& value) override
  {
    health.push_back(value);
  }

  void on_rejection(uint32_t can_id, ctr::verdict reason) override
  {
    rejected_ids.push_back(can_id);
    rejections.push_back(reason);
  }

  std::vector<fr::measurement> measurements;
  std::vector<fr::health> health;
  std::vector<uint32_t> rejected_ids;
  std::vector<ctr::verdict> rejections;
};

}  // namespace

TEST(ReceiverCliffRegistry, BothContractIdentifiersAreReceivedAndRoutedExactlyOnce)
{
  EXPECT_EQ(ctr::kMeasId, ids::TOF_CLIFF_MEASUREMENT);
  EXPECT_EQ(ctr::kHealthId, ids::TOF_CLIFF_HEALTH);

  size_t cliff_owned = 0;
  for (size_t i = 0; i < ids::kTableCount; ++i)
    if (ids::kTable[i].who == ids::owner::cliff)
      ++cliff_owned;
  ASSERT_EQ(2u, cliff_owned) << "a new cliff-owned ID needs an explicit decoder and test";

  for (uint32_t id : { ids::TOF_CLIFF_MEASUREMENT, ids::TOF_CLIFF_HEALTH })
  {
    size_t matches = 0;
    for (size_t i = 0; i < ids::kTableCount; ++i)
    {
      const auto& entry = ids::kTable[i];
      if (entry.id != id)
        continue;
      ++matches;
      EXPECT_EQ(ids::direction::rx, entry.dir);
      EXPECT_EQ(ids::owner::cliff, entry.who);
    }
    EXPECT_EQ(1u, matches) << "CAN id 0x" << std::hex << id;
    EXPECT_EQ(ids::owner::cliff, ids::route(id));
  }
}

TEST(ReceiverCliffRegistry, ContractIdentityMatchesTheHardwareCapture)
{
  EXPECT_STREQ("fb94706a4d2488aa9acdc7c7defcd7fac92379cba01964ab31f949fa50955188", ctr::kContractSha256);
  EXPECT_STREQ("3db018e9f0be3ae85a295240a8314fff97491b587ec909021b8e478745caf9aa", ctr::kArtefactSetId);
  EXPECT_TRUE(ctr::kReleaseForbidden);
}

TEST(ReceiverCliffIngress, CapturedRealMeasurementsReachTheTypedSink)
{
  struct captured
  {
    std::array<uint8_t, 8> bytes;
    uint8_t source_id;
    uint8_t epoch;
    uint8_t cycle;
  };
  const captured cases[]{
    { { 0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00 }, 0, 1, 0 },
    { { 0x13, 0x01, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00 }, 3, 1, 0 },
    { { 0x10, 0x2a, 0x07, 0xff, 0xff, 0xff, 0x00, 0x00 }, 0, 42, 7 },
  };

  recording_sink sink;
  lexxhard::receiver_cliff receiver{ sink };
  for (const captured& c : cases)
  {
    EXPECT_TRUE(lexxhard::route_cliff_frame(frame(ctr::kMeasId, c.bytes), receiver));
    ASSERT_FALSE(sink.measurements.empty());
    const fr::measurement& got = sink.measurements.back();
    EXPECT_EQ(c.source_id, got.source_id);
    EXPECT_EQ(c.epoch, got.mapping_epoch);
    EXPECT_EQ(c.cycle, got.cycle_seq);
    EXPECT_EQ(ctr::status_class::no_target, got.cls);
    EXPECT_EQ(ctr::kSentinelInvalid, got.range_mm);
    EXPECT_TRUE(got.range_is_sentinel);
    EXPECT_EQ(255u, got.raw_status);
    EXPECT_EQ(0u, got.target_count);
  }

  EXPECT_EQ(3u, sink.measurements.size());
  EXPECT_TRUE(sink.health.empty());
  EXPECT_TRUE(sink.rejections.empty());
}

TEST(ReceiverCliffIngress, CapturedRealUnknownHeartbeatReachesTheTypedSink)
{
  // dasher1 can1, after the bench pack command: mapping remained UNKNOWN, no masks and no
  // implicated chain position. 0xA1 is an ordinary observed health-sequence value.
  const std::array<uint8_t, 8> captured{ 0x21, 0x00, 0xa1, 0x00, 0x00, 0x00, 0xff, 0x00 };

  recording_sink sink;
  lexxhard::receiver_cliff receiver{ sink };
  EXPECT_TRUE(lexxhard::route_cliff_frame(frame(ctr::kHealthId, captured), receiver));

  ASSERT_EQ(1u, sink.health.size());
  const fr::health& got = sink.health.front();
  EXPECT_EQ(ctr::kProtocolVersion, got.protocol_version);
  EXPECT_EQ(0u, got.mapping_epoch);
  EXPECT_EQ(0xa1u, got.health_seq);
  EXPECT_EQ(0u, got.mapping_state);
  EXPECT_EQ(0u, got.flags);
  EXPECT_EQ(0u, got.enumerated_mask);
  EXPECT_EQ(0u, got.model_verified_mask);
  EXPECT_EQ(0u, got.sample_produced_mask);
  EXPECT_EQ(0u, got.sensor_fault_mask);
  EXPECT_EQ(ctr::kChainPositionNone, got.failing_chain_position);
  EXPECT_EQ(0u, got.cycle_seq);
  EXPECT_FALSE(got.cycle_valid);
  EXPECT_FALSE(got.chain_fault);
  EXPECT_TRUE(sink.measurements.empty());
  EXPECT_TRUE(sink.rejections.empty());
}

TEST(ReceiverCliffIngress, ARejectedFrameNeverReachesATypedSink)
{
  const std::array<uint8_t, 8> bad_reserved{ 0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x00, 0x01 };

  recording_sink sink;
  lexxhard::receiver_cliff receiver{ sink };
  EXPECT_TRUE(lexxhard::route_cliff_frame(frame(ctr::kMeasId, bad_reserved), receiver));

  EXPECT_TRUE(sink.measurements.empty());
  EXPECT_TRUE(sink.health.empty());
  ASSERT_EQ(1u, sink.rejections.size());
  EXPECT_EQ(ctr::verdict::reserved_field_nonzero, sink.rejections.front());
  EXPECT_EQ(ctr::kMeasId, sink.rejected_ids.front());
}

TEST(ReceiverCliffIngress, TheCanDlcIsPassedToTheDecoderNotInventedAtIngress)
{
  const std::array<uint8_t, 8> valid_bytes{ 0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00 };

  recording_sink sink;
  lexxhard::receiver_cliff receiver{ sink };
  EXPECT_TRUE(lexxhard::route_cliff_frame(frame(ctr::kMeasId, valid_bytes, 7), receiver));

  EXPECT_TRUE(sink.measurements.empty());
  EXPECT_TRUE(sink.health.empty());
  ASSERT_EQ(1u, sink.rejections.size());
  EXPECT_EQ(ctr::verdict::dlc_not_8, sink.rejections.front());
}

TEST(ReceiverCliffIngress, NeighbouringIdentifiersAreLeftForTheirOwners)
{
  const std::array<uint8_t, 8> payload{ 0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00 };
  recording_sink sink;
  lexxhard::receiver_cliff receiver{ sink };

  for (uint32_t id : { 0x204u, 0x213u, 0x214u, 0x215u, 0x218u })
    EXPECT_FALSE(lexxhard::route_cliff_frame(frame(id, payload), receiver)) << std::hex << id;

  EXPECT_TRUE(sink.measurements.empty());
  EXPECT_TRUE(sink.health.empty());
  EXPECT_TRUE(sink.rejections.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
