// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Frame-level decode, driven entirely by the layout vectors vendored from the firmware
// repository. This file writes no byte layout of its own: every expectation comes from
// the generated header, so a contract change shows up as a vector change rather than as
// two files that have to be edited in agreement.
//
// This is the decoder half of the cross-repository pin. The firmware's
// test_tof_cliff_packer asserts the same SHA on the producer side; a contract edit
// regenerates the artefacts with a new SHA and fails both until the literals are
// updated deliberately.
//
// What is NOT tested here, because it is not implemented and cannot be yet: cycle
// assembly, retirement, staleness, READY. Every timing value they need is unresolved and
// the contract's event-multiset vectors do not exist. The layout vectors imply none of
// it.

#include <gtest/gtest.h>

#include <cstring>
#include <set>
#include <string>

#include "tof_cliff_frame.hpp"
#include "vendor/tof_cliff_contract_vectors.h"

namespace ctr = tof_cliff_contract;
namespace fr = tof_cliff_frame;

namespace {

const ctr::vector* find_vector(const char* name)
{
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i)
    if (std::strcmp(ctr::kVectors[i].name, name) == 0) return &ctr::kVectors[i];
  return nullptr;
}

ctr::verdict decode_either(const ctr::vector& v)
{
  if (v.kind == ctr::frame_kind::measurement) {
    fr::measurement m;
    return fr::decode_measurement(v.dlc, v.bytes, m);
  }
  fr::health h;
  return fr::decode_health(v.dlc, v.bytes, h);
}

}  // namespace

// ------------------------------------------------------------------- the pin -------

TEST(TofCliffFrame, ContractShaPin)
{
  EXPECT_STREQ("4b3dae652d73e6eabf17d721a90b06371effbb3aaefb3d25a2218bb860dd47f7",
               ctr::kContractSha256);
  EXPECT_STREQ("commissioning-2026-08-18b", ctr::kContractVersion);
  EXPECT_STREQ("commissioning-cliff-only-400k", ctr::kProfileName);
  // Asserted rather than merely present: this revision is not releasable, and the day
  // someone flips it must show up in a diff on both sides.
  EXPECT_TRUE(ctr::kReleaseForbidden);
  EXPECT_EQ(0x216u, ctr::kMeasId);
  EXPECT_EQ(0x217u, ctr::kHealthId);
  EXPECT_EQ(1u, ctr::kProtocolVersion);
  EXPECT_EQ(0xFFFFu, ctr::kSentinelInvalid);
  EXPECT_EQ(8u, ctr::kDlc);
}

// -------------------------------------------------------- every vector, no exception

TEST(TofCliffFrame, EveryLayoutVectorGetsItsStatedVerdict)
{
  ASSERT_GT(ctr::kVectorCount, 0u);
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
    const ctr::vector& v = ctr::kVectors[i];
    EXPECT_EQ(static_cast<int>(v.expected), static_cast<int>(decode_either(v)))
        << "vector " << v.name << " -- " << v.why;
  }
}

// A suite that silently stopped covering rules would still pass the loop above, so the
// coverage itself is asserted. The generator already refuses to emit unless every
// verdict has a vector; this is the same guard on the consuming side.
TEST(TofCliffFrame, EveryVerdictTheDecoderCanReturnIsExercised)
{
  std::set<int> seen;
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) seen.insert(static_cast<int>(decode_either(ctr::kVectors[i])));

  // accept plus every rejection the contract defines.
  for (int r = static_cast<int>(ctr::verdict::accept);
       r <= static_cast<int>(ctr::verdict::chain_position_without_fault); ++r)
    EXPECT_EQ(1u, seen.count(r)) << "no vector produced verdict " << r;
}

TEST(TofCliffFrame, RejectionLeavesTheOutputUntouched)
{
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
    const ctr::vector& v = ctr::kVectors[i];
    if (v.expected == ctr::verdict::accept) continue;

    if (v.kind == ctr::frame_kind::measurement) {
      fr::measurement m;
      m.source_id = 0xAA;
      m.raw_status = 0xBB;
      m.range_mm = 0xCCDD;
      m.target_count = 0xEE;
      ASSERT_NE(ctr::verdict::accept, fr::decode_measurement(v.dlc, v.bytes, m)) << v.name;
      EXPECT_EQ(0xAAu, m.source_id) << v.name;
      EXPECT_EQ(0xBBu, m.raw_status) << v.name;
      EXPECT_EQ(0xCCDDu, m.range_mm) << v.name;
      EXPECT_EQ(0xEEu, m.target_count) << v.name;
    } else {
      fr::health h;
      h.health_seq = 0xAA;
      h.mapping_state = 0x0B;
      h.failing_chain_position = 0xCC;
      ASSERT_NE(ctr::verdict::accept, fr::decode_health(v.dlc, v.bytes, h)) << v.name;
      EXPECT_EQ(0xAAu, h.health_seq) << v.name;
      EXPECT_EQ(0x0Bu, h.mapping_state) << v.name;
      EXPECT_EQ(0xCCu, h.failing_chain_position) << v.name;
    }
  }
}

// ------------------------------------------------------------- field decoding ------

TEST(TofCliffFrame, AValidMeasurementDecodesEveryField)
{
  const ctr::vector* v = find_vector("meas_role_0_front_left");
  ASSERT_NE(nullptr, v);

  fr::measurement m;
  ASSERT_EQ(ctr::verdict::accept, fr::decode_measurement(v->dlc, v->bytes, m));
  EXPECT_EQ(0u, m.source_id);
  EXPECT_EQ(1u, m.mapping_epoch);
  EXPECT_EQ(0u, m.cycle_seq);
  EXPECT_EQ(1234u, m.range_mm);
  EXPECT_EQ(0u, m.raw_status);
  EXPECT_EQ(1u, m.target_count);
  EXPECT_EQ(static_cast<int>(ctr::status_class::valid_range), static_cast<int>(m.cls));
  EXPECT_FALSE(m.no_target);
}

TEST(TofCliffFrame, TheNoTargetEncodingDecodesToTheSentinelNotAnEightThousandRange)
{
  const ctr::vector* v = find_vector("meas_status_255_no_target");
  ASSERT_NE(nullptr, v);

  fr::measurement m;
  ASSERT_EQ(ctr::verdict::accept, fr::decode_measurement(v->dlc, v->bytes, m));
  EXPECT_EQ(255u, m.raw_status);
  EXPECT_EQ(ctr::kSentinelInvalid, m.range_mm);
  EXPECT_EQ(0u, m.target_count);
  EXPECT_TRUE(m.no_target);
  // The ordinary outcome over a genuine drop-off, and the consumer must stop on it --
  // which is why it decodes as data rather than as a fault.
  EXPECT_EQ(static_cast<int>(ctr::status_class::no_target), static_cast<int>(m.cls));
}

TEST(TofCliffFrame, EveryClassifiedStatusDecodesToItsTableClass)
{
  // The decoder must not carry its own opinion about what a status means.
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
    const ctr::vector& v = ctr::kVectors[i];
    if (v.kind != ctr::frame_kind::measurement) continue;
    if (v.expected != ctr::verdict::accept) continue;

    fr::measurement m;
    ASSERT_EQ(ctr::verdict::accept, fr::decode_measurement(v.dlc, v.bytes, m)) << v.name;

    const uint8_t raw = v.bytes[5];
    bool found = false;
    for (std::size_t j = 0; j < ctr::kStatusRowCount; ++j) {
      if (ctr::kStatusTable[j].raw != raw) continue;
      EXPECT_EQ(static_cast<int>(ctr::kStatusTable[j].cls), static_cast<int>(m.cls)) << v.name;
      found = true;
    }
    EXPECT_TRUE(found) << "accepted an unclassified status: " << v.name;
  }
}

TEST(TofCliffFrame, AValidHealthDecodesEveryField)
{
  const ctr::vector* v = find_vector("health_proven_cycle_valid");
  ASSERT_NE(nullptr, v);

  fr::health h;
  ASSERT_EQ(ctr::verdict::accept, fr::decode_health(v->dlc, v->bytes, h));
  EXPECT_EQ(1u, h.protocol_version);
  EXPECT_EQ(1u, h.mapping_epoch);
  EXPECT_EQ(7u, h.health_seq);
  EXPECT_EQ(0x1u, h.mapping_state);  // PROVEN
  EXPECT_EQ(0xFu, h.enumerated_mask);
  EXPECT_EQ(0xFu, h.model_verified_mask);
  EXPECT_EQ(0xFu, h.sample_produced_mask);
  EXPECT_EQ(0x0u, h.sensor_fault_mask);
  EXPECT_EQ(ctr::kChainPositionNone, h.failing_chain_position);
  EXPECT_EQ(0u, h.cycle_seq);
  EXPECT_TRUE(h.cycle_valid);
  EXPECT_FALSE(h.chain_fault) << "cycle_valid is not a chain fault";
}

TEST(TofCliffFrame, AHeartbeatDecodesWithoutDescribingACycle)
{
  const ctr::vector* v = find_vector("health_heartbeat_unknown");
  ASSERT_NE(nullptr, v);

  fr::health h;
  ASSERT_EQ(ctr::verdict::accept, fr::decode_health(v->dlc, v->bytes, h));
  EXPECT_EQ(0x0u, h.mapping_state);  // UNKNOWN
  EXPECT_FALSE(h.cycle_valid);
  EXPECT_FALSE(h.chain_fault);
  EXPECT_EQ(0u, h.sample_produced_mask);
  EXPECT_EQ(0u, h.sensor_fault_mask);
}

// -------------------------------------------------------------- identifiers -------

TEST(TofCliffFrame, IdentifiersAreClassifiedAndNothingElseIsClaimed)
{
  EXPECT_EQ(static_cast<int>(fr::arrival::measurement),
            static_cast<int>(fr::classify_identifier(ctr::kMeasId)));
  EXPECT_EQ(static_cast<int>(fr::arrival::health),
            static_cast<int>(fr::classify_identifier(ctr::kHealthId)));
  // The neighbours belong to the grid contract and to the SCB peripheral block; claiming
  // one would take a frame away from its owner, silently.
  for (uint32_t id : {0x213u, 0x214u, 0x215u, 0x218u, 0x204u}) {
    EXPECT_EQ(static_cast<int>(fr::arrival::not_ours),
              static_cast<int>(fr::classify_identifier(id)))
        << "claimed identifier 0x" << std::hex << id;
  }
}

// --------------------------------------------------------- statelessness ----------

TEST(TofCliffFrame, DecodingIsStateless)
{
  // No cycle tracking, no watchdog, no memory of anything: the same bytes must give the
  // same answer however many times they arrive, and in whatever order.
  for (int pass = 0; pass < 3; ++pass) {
    for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
      const ctr::vector& v = ctr::kVectors[ctr::kVectorCount - 1 - i];
      EXPECT_EQ(static_cast<int>(v.expected), static_cast<int>(decode_either(v)))
          << "pass " << pass << " vector " << v.name;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
