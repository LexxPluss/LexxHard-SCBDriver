// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Frame-level decode, driven entirely by the layout vectors vendored from the firmware
// repository. This file writes no byte layout of its own: every expectation comes from the
// generated header, so a contract change shows up as a vector change rather than as two
// files that have to be edited in agreement.
//
// This is the decoder half of the cross-repository pin. The firmware's
// test_tof_cliff_packer asserts the same contract SHA and the same artefact-set identifier
// on the producer side; a change to either regenerates the artefacts and fails both pins
// until the literals are updated deliberately.
//
// Everything goes through decode(can_id, ...) because that is the only entry point there
// is. The per-type decoders are internal, so a test cannot pair a payload with the wrong
// decoder even by accident -- which is the point, since a caller could not either.
//
// What is NOT tested here, because it is not implemented and cannot be yet: cycle assembly,
// retirement, staleness, READY. Every timing value they need is unresolved and the
// contract's event-multiset vectors do not exist. The layout vectors imply none of it.

#include <gtest/gtest.h>

#include <cstring>
#include <set>

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

// The identifier a frame of this kind would really arrive on. Choosing the identifier is
// legitimate -- a real frame has one; choosing the decoder is not, and is no longer
// possible.
uint32_t identifier_for(const ctr::vector& v)
{
  return v.kind == ctr::frame_kind::measurement ? ctr::kMeasId : ctr::kHealthId;
}

fr::decoded decode_vector(const ctr::vector& v)
{
  return fr::decode(identifier_for(v), v.dlc, v.bytes);
}

}  // namespace

// ------------------------------------------------------------------- the pin -------

TEST(TofCliffFrame, ContractAndArtefactPins)
{
  EXPECT_STREQ("4b3dae652d73e6eabf17d721a90b06371effbb3aaefb3d25a2218bb860dd47f7",
               ctr::kContractSha256);
  EXPECT_STREQ("commissioning-2026-08-18b", ctr::kContractVersion);
  EXPECT_STREQ("commissioning-cliff-only-400k", ctr::kProfileName);
  // The contract SHA says which contract; this says which generated artefacts. Pinned
  // separately because the generator has twice changed what it emits while the contract
  // text -- and so its SHA -- stood still.
  EXPECT_STREQ("1a0a2094e928392d29c7153f84de6951159113198714e0b734e6ebd39bf74b07",
               ctr::kArtefactSetId);
  // Asserted rather than merely present: this revision is not releasable, and the day
  // someone flips it must show up in a diff on both sides.
  EXPECT_TRUE(ctr::kReleaseForbidden);
  EXPECT_EQ(0x216u, ctr::kMeasId);
  EXPECT_EQ(0x217u, ctr::kHealthId);
  EXPECT_EQ(1u, ctr::kProtocolVersion);
  EXPECT_EQ(0xFFFFu, ctr::kSentinelInvalid);
  EXPECT_EQ(8u, ctr::kDlc);
  // Distinct concepts that happen to share a value today.
  EXPECT_EQ(4u, ctr::kMaxTargets);
  EXPECT_EQ(4u, ctr::kSourceCount);
}

// -------------------------------------------------------- every vector, no exception

TEST(TofCliffFrame, EveryLayoutVectorGetsItsStatedVerdict)
{
  ASSERT_GT(ctr::kVectorCount, 0u);
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
    const ctr::vector& v = ctr::kVectors[i];
    const fr::decoded d = decode_vector(v);
    ASSERT_NE(static_cast<int>(fr::arrival::not_ours), static_cast<int>(d.which)) << v.name;
    EXPECT_EQ(static_cast<int>(v.expected), static_cast<int>(d.result))
        << "vector " << v.name << " -- " << v.why;
  }
}

// A suite that silently stopped covering rules would still pass the loop above, so the
// coverage itself is asserted. The generator refuses to emit unless every verdict has a
// vector; this is the same guard on the consuming side.
TEST(TofCliffFrame, EveryVerdictTheDecoderCanReturnIsExercised)
{
  std::set<int> seen;
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i)
    seen.insert(static_cast<int>(decode_vector(ctr::kVectors[i]).result));

  for (int r = static_cast<int>(ctr::verdict::accept);
       r <= static_cast<int>(ctr::verdict::chain_position_without_fault); ++r)
    EXPECT_EQ(1u, seen.count(r)) << "no vector produced verdict " << r;
}

TEST(TofCliffFrame, RejectionLeavesTheOutputAtItsDefault)
{
  const fr::measurement pristine_meas{};
  const fr::health pristine_health{};
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
    const ctr::vector& v = ctr::kVectors[i];
    if (v.expected == ctr::verdict::accept) continue;

    const fr::decoded d = decode_vector(v);
    ASSERT_NE(ctr::verdict::accept, d.result) << v.name;
    // Compared field by field rather than by memcmp, because padding is not guaranteed.
    EXPECT_EQ(pristine_meas.source_id, d.meas.source_id) << v.name;
    EXPECT_EQ(pristine_meas.range_mm, d.meas.range_mm) << v.name;
    EXPECT_EQ(pristine_meas.raw_status, d.meas.raw_status) << v.name;
    EXPECT_EQ(pristine_meas.target_count, d.meas.target_count) << v.name;
    EXPECT_EQ(pristine_meas.range_is_sentinel, d.meas.range_is_sentinel) << v.name;
    EXPECT_EQ(pristine_health.health_seq, d.state.health_seq) << v.name;
    EXPECT_EQ(pristine_health.mapping_state, d.state.mapping_state) << v.name;
    EXPECT_EQ(pristine_health.cycle_valid, d.state.cycle_valid) << v.name;
  }
}

// ------------------------------------------- the identifier binds the decoder ------

TEST(TofCliffFrame, APayloadOnTheWrongIdentifierIsRejected)
{
  // The mis-routing frame_type exists to catch. Every accepted vector, offered on the
  // other cliff identifier, must be refused -- and refused as a frame_type mismatch, not
  // quietly decoded as the other kind.
  int measurements = 0, healths = 0;
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
    const ctr::vector& v = ctr::kVectors[i];
    if (v.expected != ctr::verdict::accept) continue;

    const bool is_meas = (v.kind == ctr::frame_kind::measurement);
    const uint32_t wrong = is_meas ? ctr::kHealthId : ctr::kMeasId;
    const fr::decoded d = fr::decode(wrong, v.dlc, v.bytes);

    EXPECT_EQ(static_cast<int>(is_meas ? fr::arrival::health : fr::arrival::measurement),
              static_cast<int>(d.which))
        << v.name << ": the identifier must decide which decoder runs";
    EXPECT_EQ(static_cast<int>(ctr::verdict::frame_type_mismatch), static_cast<int>(d.result))
        << v.name << ": accepted on the wrong identifier";
    (is_meas ? measurements : healths)++;
  }
  // Both directions were actually exercised, not just whichever happened to be first.
  EXPECT_GT(measurements, 0);
  EXPECT_GT(healths, 0);
}

TEST(TofCliffFrame, NeighbouringIdentifiersAreNotClaimed)
{
  const ctr::vector* v = find_vector("meas_role_0_front_left");
  ASSERT_NE(nullptr, v);
  // These belong to the grid contract and to the SCB peripheral block. Claiming one would
  // take a frame away from its owner, silently.
  for (uint32_t id : {0x204u, 0x213u, 0x214u, 0x215u, 0x218u}) {
    const fr::decoded d = fr::decode(id, v->dlc, v->bytes);
    EXPECT_EQ(static_cast<int>(fr::arrival::not_ours), static_cast<int>(d.which))
        << "claimed identifier 0x" << std::hex << id;
    EXPECT_EQ(0u, d.meas.range_mm) << "decoded a frame that was never ours";
  }
}

// ------------------------------------------------------------- field decoding ------

TEST(TofCliffFrame, AValidMeasurementDecodesEveryField)
{
  const ctr::vector* v = find_vector("meas_role_0_front_left");
  ASSERT_NE(nullptr, v);

  const fr::decoded d = fr::decode(ctr::kMeasId, v->dlc, v->bytes);
  ASSERT_EQ(ctr::verdict::accept, d.result);
  EXPECT_EQ(static_cast<int>(fr::arrival::measurement), static_cast<int>(d.which));
  EXPECT_EQ(0u, d.meas.source_id);
  EXPECT_EQ(1u, d.meas.mapping_epoch);
  EXPECT_EQ(0u, d.meas.cycle_seq);
  EXPECT_EQ(1234u, d.meas.range_mm);
  EXPECT_EQ(0u, d.meas.raw_status);
  EXPECT_EQ(1u, d.meas.target_count);
  EXPECT_EQ(static_cast<int>(ctr::status_class::valid_range), static_cast<int>(d.meas.cls));
  EXPECT_FALSE(d.meas.range_is_sentinel);
}

TEST(TofCliffFrame, TheNoTargetEncodingDecodesToTheSentinelNotAnEightThousandRange)
{
  const ctr::vector* v = find_vector("meas_status_255_no_target");
  ASSERT_NE(nullptr, v);

  const fr::decoded d = fr::decode(ctr::kMeasId, v->dlc, v->bytes);
  ASSERT_EQ(ctr::verdict::accept, d.result);
  EXPECT_EQ(255u, d.meas.raw_status);
  EXPECT_EQ(ctr::kSentinelInvalid, d.meas.range_mm);
  EXPECT_EQ(0u, d.meas.target_count);
  EXPECT_TRUE(d.meas.range_is_sentinel);
  // The ordinary outcome over a genuine drop-off, and the consumer must stop on it --
  // which is why it decodes as data rather than as a fault.
  EXPECT_EQ(static_cast<int>(ctr::status_class::no_target), static_cast<int>(d.meas.cls));
}

TEST(TofCliffFrame, ASensorFaultAlsoCarriesTheSentinelSoTheClassIsWhatDistinguishesIt)
{
  // The reason the flag is called range_is_sentinel and not no_target. A consumer that
  // branched on the sentinel alone would report a hardware fault as an empty floor, which
  // for a cliff sensor means it would look like an ordinary drop rather than a defect.
  const ctr::vector* fault = find_vector("meas_status_5_sensor_fault");
  const ctr::vector* empty = find_vector("meas_status_2_no_target");
  ASSERT_NE(nullptr, fault);
  ASSERT_NE(nullptr, empty);

  const fr::decoded f = fr::decode(ctr::kMeasId, fault->dlc, fault->bytes);
  const fr::decoded e = fr::decode(ctr::kMeasId, empty->dlc, empty->bytes);
  ASSERT_EQ(ctr::verdict::accept, f.result);
  ASSERT_EQ(ctr::verdict::accept, e.result);

  EXPECT_TRUE(f.meas.range_is_sentinel);
  EXPECT_TRUE(e.meas.range_is_sentinel);
  EXPECT_EQ(f.meas.range_mm, e.meas.range_mm) << "the encoding cannot tell them apart";
  EXPECT_NE(static_cast<int>(f.meas.cls), static_cast<int>(e.meas.cls))
      << "the class must, and is the only thing that does";
  EXPECT_EQ(static_cast<int>(ctr::status_class::sensor_fault), static_cast<int>(f.meas.cls));
  EXPECT_EQ(static_cast<int>(ctr::status_class::no_target), static_cast<int>(e.meas.cls));
}

TEST(TofCliffFrame, EveryClassifiedStatusDecodesToItsTableClass)
{
  // The decoder must not carry its own opinion about what a status means.
  for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
    const ctr::vector& v = ctr::kVectors[i];
    if (v.kind != ctr::frame_kind::measurement) continue;
    if (v.expected != ctr::verdict::accept) continue;

    const fr::decoded d = fr::decode(ctr::kMeasId, v.dlc, v.bytes);
    ASSERT_EQ(ctr::verdict::accept, d.result) << v.name;

    const uint8_t raw = v.bytes[5];
    bool found = false;
    for (std::size_t j = 0; j < ctr::kStatusRowCount; ++j) {
      if (ctr::kStatusTable[j].raw != raw) continue;
      EXPECT_EQ(static_cast<int>(ctr::kStatusTable[j].cls), static_cast<int>(d.meas.cls))
          << v.name;
      found = true;
    }
    EXPECT_TRUE(found) << "accepted an unclassified status: " << v.name;
  }
}

TEST(TofCliffFrame, AValidHealthDecodesEveryField)
{
  const ctr::vector* v = find_vector("health_proven_cycle_valid");
  ASSERT_NE(nullptr, v);

  const fr::decoded d = fr::decode(ctr::kHealthId, v->dlc, v->bytes);
  ASSERT_EQ(ctr::verdict::accept, d.result);
  EXPECT_EQ(static_cast<int>(fr::arrival::health), static_cast<int>(d.which));
  EXPECT_EQ(1u, d.state.protocol_version);
  EXPECT_EQ(1u, d.state.mapping_epoch);
  EXPECT_EQ(7u, d.state.health_seq);
  EXPECT_EQ(0x1u, d.state.mapping_state);  // PROVEN
  EXPECT_EQ(0xFu, d.state.enumerated_mask);
  EXPECT_EQ(0xFu, d.state.model_verified_mask);
  EXPECT_EQ(0xFu, d.state.sample_produced_mask);
  EXPECT_EQ(0x0u, d.state.sensor_fault_mask);
  EXPECT_EQ(ctr::kChainPositionNone, d.state.failing_chain_position);
  EXPECT_EQ(0u, d.state.cycle_seq);
  EXPECT_TRUE(d.state.cycle_valid);
  EXPECT_FALSE(d.state.chain_fault) << "cycle_valid is not a chain fault";
}

TEST(TofCliffFrame, AHeartbeatDecodesWithoutDescribingACycle)
{
  const ctr::vector* v = find_vector("health_heartbeat_unknown");
  ASSERT_NE(nullptr, v);

  const fr::decoded d = fr::decode(ctr::kHealthId, v->dlc, v->bytes);
  ASSERT_EQ(ctr::verdict::accept, d.result);
  EXPECT_EQ(0x0u, d.state.mapping_state);  // UNKNOWN
  EXPECT_FALSE(d.state.cycle_valid);
  EXPECT_FALSE(d.state.chain_fault);
  EXPECT_EQ(0u, d.state.sample_produced_mask);
  EXPECT_EQ(0u, d.state.sensor_fault_mask);
}

// --------------------------------------------------------- statelessness ----------

TEST(TofCliffFrame, DecodingIsStateless)
{
  // No cycle tracking, no watchdog, no memory of anything: the same bytes must give the
  // same answer however many times they arrive, and in whatever order.
  for (int pass = 0; pass < 3; ++pass) {
    for (std::size_t i = 0; i < ctr::kVectorCount; ++i) {
      const ctr::vector& v = ctr::kVectors[ctr::kVectorCount - 1 - i];
      EXPECT_EQ(static_cast<int>(v.expected), static_cast<int>(decode_vector(v).result))
          << "pass " << pass << " vector " << v.name;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
