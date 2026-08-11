/*********************************************************************
 *
 *  Copyright (c) 2024-2026, LexxPluss Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Specification test for scbdriver/CliffTofSafetyHealth.
 *
 *  Two things are checked. First the definition, against the *generated* header
 *  rather than the .msg text, so that message generation, the constant block and
 *  the field types are all verified at once. Then the readiness mapping: a
 *  reference implementation of the rule that turns observed cliff-ToF state into
 *  (readiness, reason_mask), exercised over the cases the CAN wire contract calls
 *  out.
 *
 *  The reference mapping is deliberately here rather than in a node. It is the
 *  executable form of the interface agreement, so the producer in SCBDriver and
 *  the consumer in the cliff decision node can each be checked against the same
 *  table instead of against each other.
 *
 *********************************************************************/

#include <gtest/gtest.h>

#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <scbdriver/CliffTofSafetyHealth.h>

namespace
{

using Health = scbdriver::CliffTofSafetyHealth;

// ---------------------------------------------------------------------------
// Reference readiness mapping.
//
// Inputs are what the driver knows, not what the wire carries: the wire fields
// have already been validated and correlated by the time this runs.

struct State
{
  bool config_error{ false };
  bool protocol_fault{ false };
  bool within_startup{ false };
  bool startup_grace_expired{ false };
  bool can_health_seen{ true };
  bool protocol_version_supported{ true };
  bool health_stale{ false };
  bool cycle_valid{ true };
  uint8_t mapping_state{ Health::MAPPING_STATE_PROVEN };
  bool epoch_unauthorised{ false };
  uint8_t chain_flags{ Health::CHAIN_FLAG_CYCLE_VALID };
  uint8_t failing_chain_position{ Health::CHAIN_POSITION_NONE };
  uint8_t enumerated_mask{ 0xF };
  uint8_t model_verified_mask{ 0xF };
  uint8_t sensor_fault_mask{ 0 };
  uint8_t stale_source_mask{ 0 };
  uint8_t sample_miss_fault_mask{ 0 };
  uint8_t degraded_source_mask{ 0 };
  bool ever_ready{ false };
};

constexpr uint8_t kChainFaultBits =
    Health::CHAIN_FLAG_LENGTH_MISMATCH | Health::CHAIN_FLAG_ENUMERATION_FROZEN | Health::CHAIN_FLAG_BUS_FAULT;

// Conditions that are a definite fault the moment they appear, whether or not
// the subsystem was ever READY. Everything else is "not finished initialising",
// and only becomes a fault once a required condition has been lost after being
// READY.
constexpr uint32_t kDefiniteFault =
    Health::REASON_CONFIG_ERROR | Health::REASON_PROTOCOL_FAULT | Health::REASON_PROTOCOL_VERSION_UNSUPPORTED |
    Health::REASON_HEALTH_STALE | Health::REASON_CHAIN_FAULT | Health::REASON_CHAIN_POSITION_IMPLICATED |
    Health::REASON_SENSOR_FAULT | Health::REASON_SOURCE_STALE | Health::REASON_SAMPLE_MISS_FAULT;

struct Verdict
{
  uint8_t readiness;
  uint32_t reason_mask;
};

Verdict evaluate(const State& st)
{
  uint32_t reason = 0;
  if (st.config_error)
    reason |= Health::REASON_CONFIG_ERROR;
  if (st.protocol_fault)
    reason |= Health::REASON_PROTOCOL_FAULT;

  bool never_seen_health_past_grace = false;
  if (!st.can_health_seen)
  {
    reason |= Health::REASON_NO_CAN_HEALTH;
    if (st.within_startup && !st.startup_grace_expired)
    {
      reason |= Health::REASON_STARTUP;
    }
    else
    {
      // The grace has expired and nothing has ever arrived. This is a
      // timeout, not an initialisation still in progress: leaving it
      // NOT_READY would report a permanently broken link as "still
      // starting up" for as long as the robot is powered.
      never_seen_health_past_grace = true;
    }
  }
  else
  {
    if (!st.protocol_version_supported)
      reason |= Health::REASON_PROTOCOL_VERSION_UNSUPPORTED;
    if (st.health_stale)
      reason |= Health::REASON_HEALTH_STALE;
    if (!st.cycle_valid)
      reason |= Health::REASON_CYCLE_INVALID;
    if (st.mapping_state != Health::MAPPING_STATE_PROVEN)
      reason |= Health::REASON_MAPPING_NOT_PROVEN;
    if (st.epoch_unauthorised)
      reason |= Health::REASON_EPOCH_UNAUTHORISED;
    if (st.chain_flags & kChainFaultBits)
      reason |= Health::REASON_CHAIN_FAULT;
    if (st.failing_chain_position != Health::CHAIN_POSITION_NONE)
      reason |= Health::REASON_CHAIN_POSITION_IMPLICATED;
    if (st.enumerated_mask != 0xF)
      reason |= Health::REASON_NOT_ENUMERATED;
    if (st.model_verified_mask != 0xF)
      reason |= Health::REASON_MODEL_UNVERIFIED;
    if (st.sensor_fault_mask != 0)
      reason |= Health::REASON_SENSOR_FAULT;
    if (st.stale_source_mask != 0)
      reason |= Health::REASON_SOURCE_STALE;
    if (st.sample_miss_fault_mask != 0)
      reason |= Health::REASON_SAMPLE_MISS_FAULT;
  }

  if (reason == 0)
    return { Health::READINESS_READY, 0 };

  // A mapping that was proven and then broke is a definite fault; one that has
  // never been proven is simply not finished.
  const bool lost_mapping = st.can_health_seen && (st.mapping_state == Health::MAPPING_STATE_LOST ||
                                                   st.mapping_state == Health::MAPPING_STATE_FAULT);
  if ((reason & kDefiniteFault) != 0 || never_seen_health_past_grace || lost_mapping || st.ever_ready)
    return { Health::READINESS_FAULT, reason };
  return { Health::READINESS_NOT_READY, reason };
}

// A consumer decides novelty by inequality, never by numeric comparison, so the
// 32-bit wrap is an ordinary new snapshot rather than a regression.
bool is_new_heartbeat(uint32_t previous, uint32_t current)
{
  return current != previous;
}

std::string read_file(const std::string& path)
{
  std::ifstream in(path);
  EXPECT_TRUE(in.good()) << "cannot open " << path;
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

}  // namespace

// ---------------------------------------------------------------------------
// The definition, as generated

TEST(CliffTofSafetyHealthDefinition, NotReadyIsTheDefaultConstructedValue)
{
  // An all-zero message must mean "stop", so that a consumer receiving a
  // default-constructed or partially populated message fails safe.
  const Health msg;
  EXPECT_EQ(msg.readiness, Health::READINESS_NOT_READY);
  EXPECT_EQ(Health::READINESS_NOT_READY, 0);
  EXPECT_EQ(msg.reason_mask, 0u);
  EXPECT_FALSE(msg.can_health_seen);
}

TEST(CliffTofSafetyHealthDefinition, ReadinessValuesAreDistinct)
{
  const std::set<uint8_t> values{ Health::READINESS_NOT_READY, Health::READINESS_READY, Health::READINESS_FAULT };
  EXPECT_EQ(values.size(), 3u);
}

TEST(CliffTofSafetyHealthDefinition, ReasonBitsAreDistinctSingleBits)
{
  const std::vector<uint32_t> reasons{
    Health::REASON_STARTUP,
    Health::REASON_CONFIG_ERROR,
    Health::REASON_NO_CAN_HEALTH,
    Health::REASON_HEALTH_STALE,
    Health::REASON_PROTOCOL_VERSION_UNSUPPORTED,
    Health::REASON_PROTOCOL_FAULT,
    Health::REASON_CYCLE_INVALID,
    Health::REASON_MAPPING_NOT_PROVEN,
    Health::REASON_EPOCH_UNAUTHORISED,
    Health::REASON_CHAIN_FAULT,
    Health::REASON_CHAIN_POSITION_IMPLICATED,
    Health::REASON_NOT_ENUMERATED,
    Health::REASON_MODEL_UNVERIFIED,
    Health::REASON_SENSOR_FAULT,
    Health::REASON_SOURCE_STALE,
    Health::REASON_SAMPLE_MISS_FAULT,
  };
  std::set<uint32_t> seen;
  for (const uint32_t r : reasons)
  {
    EXPECT_NE(r, 0u);
    EXPECT_EQ(r & (r - 1), 0u) << "reason " << r << " is not a single bit";
    EXPECT_TRUE(seen.insert(r).second) << "reason " << r << " is duplicated";
  }
}

TEST(CliffTofSafetyHealthDefinition, ChainFlagsFitInANibbleAndAreDistinct)
{
  const std::vector<uint8_t> flags{ Health::CHAIN_FLAG_LENGTH_MISMATCH, Health::CHAIN_FLAG_ENUMERATION_FROZEN,
                                    Health::CHAIN_FLAG_BUS_FAULT, Health::CHAIN_FLAG_CYCLE_VALID };
  std::set<uint8_t> seen;
  for (const uint8_t f : flags)
  {
    EXPECT_NE(f, 0);
    EXPECT_EQ(f & (f - 1), 0) << "flag " << static_cast<int>(f) << " is not a single bit";
    EXPECT_LE(f, 0x8);
    EXPECT_TRUE(seen.insert(f).second);
  }
  EXPECT_EQ(kChainFaultBits & Health::CHAIN_FLAG_CYCLE_VALID, 0) << "cycle_valid must not be counted as a chain fault";
}

TEST(CliffTofSafetyHealthDefinition, SourceCountMatchesTheArrayLengths)
{
  const Health msg;
  EXPECT_EQ(Health::SOURCE_COUNT, 4);
  EXPECT_EQ(msg.measurement_age.size(), static_cast<size_t>(Health::SOURCE_COUNT));
  EXPECT_EQ(msg.consecutive_missed_cycles.size(), static_cast<size_t>(Health::SOURCE_COUNT));
}

TEST(CliffTofSafetyHealthDefinition, ChainPositionNoneIsOutsideTheValidRange)
{
  // Positions are 1-6 over the whole six-board chain; 255 means none.
  EXPECT_EQ(Health::CHAIN_POSITION_NONE, 255);
}

// ---------------------------------------------------------------------------
// The definition as text, so a silent de-registration is caught

TEST(CliffTofSafetyHealthDefinition, RegisteredWithTheBuild)
{
  const std::string cmake = read_file(std::string(PACKAGE_SOURCE_DIR) + "/CMakeLists.txt");
  EXPECT_NE(cmake.find("CliffTofSafetyHealth.msg"), std::string::npos);
  const std::string msg = read_file(std::string(PACKAGE_SOURCE_DIR) + "/msg/CliffTofSafetyHealth.msg");
  EXPECT_NE(msg.find("\nHeader header\n"), std::string::npos);
}

// ---------------------------------------------------------------------------
// The readiness mapping

namespace
{

void expect_case(const State& st, uint8_t expected_readiness, uint32_t expected_reasons)
{
  const Verdict v = evaluate(st);
  EXPECT_EQ(v.readiness, expected_readiness);
  EXPECT_EQ(v.reason_mask, expected_reasons);
}

}  // namespace

TEST(CliffTofSafetyHealthReadiness, EverythingHolds)
{
  expect_case(State{}, Health::READINESS_READY, 0);
}

TEST(CliffTofSafetyHealthReadiness, StartupWithNothingReceived)
{
  State st;
  st.can_health_seen = false;
  st.within_startup = true;
  expect_case(st, Health::READINESS_NOT_READY, Health::REASON_STARTUP | Health::REASON_NO_CAN_HEALTH);
}

TEST(CliffTofSafetyHealthReadiness, NoHealthAtTheLastMomentOfTheGrace)
{
  State st;
  st.can_health_seen = false;
  st.within_startup = true;
  st.startup_grace_expired = false;
  expect_case(st, Health::READINESS_NOT_READY, Health::REASON_STARTUP | Health::REASON_NO_CAN_HEALTH);
}

TEST(CliffTofSafetyHealthReadiness, NoHealthOnceTheGraceExpiresIsAFault)
{
  // The rule this test exists for: never having received health must resolve
  // into FAULT, or a dead link reads as "still starting up" forever.
  State st;
  st.can_health_seen = false;
  st.within_startup = true;
  st.startup_grace_expired = true;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_NO_CAN_HEALTH);
}

TEST(CliffTofSafetyHealthReadiness, ConfigErrorIsAFaultBeforeAnythingArrives)
{
  State st;
  st.config_error = true;
  st.can_health_seen = false;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_CONFIG_ERROR | Health::REASON_NO_CAN_HEALTH);
}

TEST(CliffTofSafetyHealthReadiness, UnsupportedProtocolVersion)
{
  State st;
  st.protocol_version_supported = false;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_PROTOCOL_VERSION_UNSUPPORTED);
}

TEST(CliffTofSafetyHealthReadiness, ProtocolFaultLatched)
{
  State st;
  st.protocol_fault = true;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_PROTOCOL_FAULT);
}

TEST(CliffTofSafetyHealthReadiness, HealthStaleIsATimeoutFault)
{
  State st;
  st.health_stale = true;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_HEALTH_STALE);
}

TEST(CliffTofSafetyHealthReadiness, ATimeoutFaultClearsWhenHealthArrives)
{
  // A timeout fault is recomputed from current state, so it must clear by
  // itself once conditions hold. Only a protocol fault latches.
  State broken;
  broken.can_health_seen = false;
  broken.startup_grace_expired = true;
  EXPECT_EQ(evaluate(broken).readiness, Health::READINESS_FAULT);
  expect_case(State{}, Health::READINESS_READY, 0);
}

TEST(CliffTofSafetyHealthReadiness, HeartbeatOnlyCycleInvalidIsNotReady)
{
  // A state heartbeat carries no completed cycle. Nothing is broken; the
  // subsystem simply cannot claim readiness.
  State st;
  st.cycle_valid = false;
  st.chain_flags = 0;
  expect_case(st, Health::READINESS_NOT_READY, Health::REASON_CYCLE_INVALID);
}

TEST(CliffTofSafetyHealthReadiness, MappingUnknownIsNotReady)
{
  State st;
  st.mapping_state = Health::MAPPING_STATE_UNKNOWN;
  expect_case(st, Health::READINESS_NOT_READY, Health::REASON_MAPPING_NOT_PROVEN);
}

TEST(CliffTofSafetyHealthReadiness, MappingLostIsAFault)
{
  State st;
  st.mapping_state = Health::MAPPING_STATE_LOST;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_MAPPING_NOT_PROVEN);
}

TEST(CliffTofSafetyHealthReadiness, EpochChangeAwaitingAuthorisation)
{
  State st;
  st.epoch_unauthorised = true;
  expect_case(st, Health::READINESS_NOT_READY, Health::REASON_EPOCH_UNAUTHORISED);
}

TEST(CliffTofSafetyHealthReadiness, ChainBusFault)
{
  State st;
  st.chain_flags = Health::CHAIN_FLAG_CYCLE_VALID | Health::CHAIN_FLAG_BUS_FAULT;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_CHAIN_FAULT);
}

TEST(CliffTofSafetyHealthReadiness, PartialEnumerationWhileUnknownIsNotReady)
{
  State st;
  st.mapping_state = Health::MAPPING_STATE_UNKNOWN;
  st.enumerated_mask = 0x7;
  st.model_verified_mask = 0x7;
  expect_case(st, Health::READINESS_NOT_READY,
              Health::REASON_MAPPING_NOT_PROVEN | Health::REASON_NOT_ENUMERATED | Health::REASON_MODEL_UNVERIFIED);
}

TEST(CliffTofSafetyHealthReadiness, PartialEnumerationThatFrozeIsAFault)
{
  State st;
  st.mapping_state = Health::MAPPING_STATE_UNKNOWN;
  st.chain_flags = Health::CHAIN_FLAG_CYCLE_VALID | Health::CHAIN_FLAG_ENUMERATION_FROZEN;
  st.enumerated_mask = 0x7;
  expect_case(st, Health::READINESS_FAULT,
              Health::REASON_MAPPING_NOT_PROVEN | Health::REASON_CHAIN_FAULT | Health::REASON_NOT_ENUMERATED);
}

TEST(CliffTofSafetyHealthReadiness, OneSensorFaultCostsReadinessImmediately)
{
  State st;
  st.sensor_fault_mask = 0x2;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_SENSOR_FAULT);
}

TEST(CliffTofSafetyHealthReadiness, OneStaleSource)
{
  State st;
  st.stale_source_mask = 0x1;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_SOURCE_STALE);
}

TEST(CliffTofSafetyHealthReadiness, ConsecutiveMissFault)
{
  State st;
  st.sample_miss_fault_mask = 0x8;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_SAMPLE_MISS_FAULT);
}

TEST(CliffTofSafetyHealthReadiness, DegradedSourceAloneStaysReady)
{
  // The rule this test exists for: a source temporarily short of a sample,
  // still inside its age bound, does NOT cost readiness and does not get a
  // readiness state of its own.
  State st;
  st.degraded_source_mask = 0x4;
  expect_case(st, Health::READINESS_READY, 0);
}

TEST(CliffTofSafetyHealthReadiness, LosingAConditionAfterBeingReadyIsAFault)
{
  State st;
  st.cycle_valid = false;
  st.chain_flags = 0;
  st.ever_ready = true;
  expect_case(st, Health::READINESS_FAULT, Health::REASON_CYCLE_INVALID);
}

TEST(CliffTofSafetyHealthReadiness, SeveralReasonsAccumulate)
{
  State st;
  st.health_stale = true;
  st.sensor_fault_mask = 0x1;
  st.stale_source_mask = 0x1;
  const Verdict v = evaluate(st);
  EXPECT_EQ(v.readiness, Health::READINESS_FAULT);
  EXPECT_TRUE(v.reason_mask & Health::REASON_HEALTH_STALE);
  EXPECT_TRUE(v.reason_mask & Health::REASON_SENSOR_FAULT);
  EXPECT_TRUE(v.reason_mask & Health::REASON_SOURCE_STALE);
}

// ---------------------------------------------------------------------------
// Heartbeat novelty

TEST(CliffTofSafetyHealthHeartbeat, RepeatIsNotNew)
{
  EXPECT_FALSE(is_new_heartbeat(7, 7));
}

TEST(CliffTofSafetyHealthHeartbeat, IncrementIsNew)
{
  EXPECT_TRUE(is_new_heartbeat(7, 8));
}

TEST(CliffTofSafetyHealthHeartbeat, WrapIsNew)
{
  EXPECT_TRUE(is_new_heartbeat(0xFFFFFFFFu, 0u));
}

TEST(CliffTofSafetyHealthHeartbeat, ANumericComparisonWouldHaveRejectedTheWrap)
{
  // Documents why the rule is inequality: the obvious ">" test fails here.
  const uint32_t previous = 0xFFFFFFFFu;
  const uint32_t current = 0u;
  EXPECT_FALSE(current > previous);
  EXPECT_TRUE(is_new_heartbeat(previous, current));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
