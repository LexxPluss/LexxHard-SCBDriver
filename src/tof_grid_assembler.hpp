/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Reassembles the VL53L7CX 8x8 grid from CAN frames.
//
// Deliberately free of ROS and SocketCAN so it can be tested exhaustively against the
// golden vectors without a node, a bus, or a sleep. The caller supplies monotonic time.
//
// The wire format is specified in LexxHard-SensorControlBoard-Firmware
// docs/can/tof_can_wire_contract.md, and this file must stay in step with the vendored
// copy of the generated vectors. See test/vendor/tof_contract_vectors.h.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace lexxhard
{

class tof_grid_assembler
{
public:
  static constexpr uint8_t ZONES = 64;
  static constexpr uint8_t CHUNKS = 16;
  static constexpr uint8_t ZONES_PER_CHUNK = 4;
  static constexpr uint16_t INVALID_MM = 0xFFF;
  static constexpr uint16_t MAX_VALID_MM = 0xFFE;
  static constexpr uint16_t COMPLETE_BITMAP = 0xFFFF;
  // Status flag bits 4-7 are reserved; so are health bytes 6-7. They are the only
  // reserved fields, and the low nibble of byte 1 is explicitly not among them.
  static constexpr uint8_t STATUS_FLAG_MASK = 0x0F;

  // Every timestamp in this class is a 64-bit monotonic millisecond count, and the caller
  // must supply one. A 32-bit count wraps every 49.7 days, and the comparison that decides
  // HEALTHY used to need a signed cast to tolerate a reference in the future -- which then
  // read a 24.8-day-old reference as recent and emitted a false SOURCE_RECOVERED. Sixty-four
  // bits removes the wrap instead of arranging the arithmetic around it.
  static constexpr uint64_t ASSEMBLY_TIMEOUT_MS = 300;
  // Deliberately independent of ASSEMBLY_TIMEOUT_MS: that one bounds how long a single
  // grid may take to arrive, this one bounds how long the robot may go without a usable
  // grid. They are not the same property and will not move together.
  static constexpr uint64_t SOURCE_STALE_MS = 1000;
  static constexpr uint64_t STARTUP_GRACE_MS = 3000;

  // Stable logical identity. NOT the chain position: the firmware owns the
  // chain_position -> source_id mapping and is the only place the physical topology
  // appears. Note that 0 is the RIGHT sensor, which reverses the legacy convention.
  enum source_id : uint8_t
  {
    SOURCE_FRONT_RIGHT = 0,
    SOURCE_FRONT_LEFT = 1,
    SOURCE_COUNT = 2,
  };

  enum class event : uint8_t
  {
    GRID_PUBLISHED,
    INCOMPLETE_BY_TIMEOUT,
    INCOMPLETE_BY_GENERATION_CHANGE,
    DUPLICATE_CHUNK_IDENTICAL,
    CONFLICTING_CHUNK,
    DUPLICATE_HEALTH_IDENTICAL,
    CONFLICTING_HEALTH,
    MALFORMED_HEADER,
    HEALTH_COUNT_MISMATCH,
    ORPHAN_HEALTH_TIMEOUT,
    FRAME_FOR_RETIRED_GENERATION,
    SOURCE_NEVER_SEEN,
    SOURCE_STALE,
    SOURCE_RECOVERED,
    EVENT_COUNT,
  };

  // What a health frame reported about itself. The wire contract is explicit that the status
  // flags, chain_position, boards_detected and last_error are DIAGNOSTIC AND DO NOT GATE: under
  // the firmware transmit obligation a grid only exists after a complete, successful,
  // model-verified read, so every flag that can legally appear on a grid-closing frame describes
  // something already recovered or something at the chain level. Gating on them would discard a
  // grid that is good by construction.
  //
  // So this enum exists for the OTHER half of "diagnostic": these facts were being parsed,
  // normalised and then read by nothing except the duplicate comparison. A field defined as
  // diagnostic with no consumer is not diagnostic, it is dead. The golden scenario
  // peer_enumeration_failure_reported says it outright -- "Publishes, and the flag must reach
  // diagnostics" -- and until now only the first half was true.
  enum class health_note : uint8_t
  {
    I2C_ERROR_RECOVERED,           // status flag bit 0
    DATA_READY_TIMEOUT_RECOVERED,  // status flag bit 1
    CHAIN_LENGTH_MISMATCH,         // status flag bit 2
    PEER_ENUMERATION_FAILED,       // status flag bit 3
    LAST_ERROR_NONZERO,            // health byte 5
    HEALTH_NOTE_COUNT,
  };

  enum class source_state : uint8_t
  {
    NEVER_SEEN,
    HEALTHY,
    STALE_NOT_COMPLETING,
    STALE_NO_FRAMES,
  };

  // Reserved fields are already stripped: see parse_health.
  struct health_info
  {
    uint8_t valid_zone_count{ 0 };
    uint8_t flags{ 0 };
    uint8_t chain_position{ 0 };
    uint8_t boards_detected{ 0 };
    uint8_t last_error{ 0 };
  };

  struct grid
  {
    uint8_t source{ 0 };
    uint8_t generation{ 0 };
    std::array<uint16_t, ZONES> zones_mm{};  // INVALID_MM where there is no target
    health_info health{};
  };

  // Carries who and what, not just that something happened. A bare SOURCE_STALE tells an
  // operator neither which sensor stopped nor whether frames are still arriving, which is
  // exactly the information the four-state model exists to provide.
  struct diagnostic
  {
    event kind{ event::EVENT_COUNT };
    uint8_t source{ SOURCE_COUNT };                  // SOURCE_COUNT when not attributable to one
    source_state state{ source_state::NEVER_SEEN };  // meaningful for the watchdog events
  };

  // A structured health diagnostic, drained like the event queue. It exists so the emission seam
  // lives in pure code: the receiver only formats what it is handed, which means a test can prove
  // the seam is still wired by replaying a scenario, and deleting the emission fails that test
  // rather than passing silently because the logging line is unreachable from a host suite.
  //
  // Carries the CONTEXT as well as the notes. chain_position and boards_detected were the two
  // fields with no consumer at all: parsed, normalised, compared for duplicate detection, and then
  // read by nothing. A chain-length mismatch is far more useful with the number of boards the
  // sensor actually saw and where in the chain it sits, and neither is worth a note of its own.
  //
  // Not a gate. This is built AFTER the grid has been accepted and cannot change that decision --
  // see the wire contract's "diagnostic and do not gate". chain_position is deliberately NOT
  // checked against source: the contract forbids a decoder branching on chain order, and both
  // fields come from the same producer, so agreement would prove nothing about installation.
  struct health_report
  {
    uint8_t source{ SOURCE_COUNT };
    uint8_t notes{ 0 };  // bitmask over health_note; never 0 for a queued report
    uint8_t chain_position{ 0 };
    uint8_t boards_detected{ 0 };
    uint8_t last_error{ 0 };
  };

  struct status
  {
    source_state state{ source_state::NEVER_SEEN };
    // True only once poll() has actually raised an alarm for this source, and false again
    // once it has cleared. A state that is merely not HEALTHY is NOT a fault to report:
    // during the startup grace every source is legitimately NEVER_SEEN, and anything that
    // reports on state alone announces two broken sensors on every single boot.
    bool alarm_active{ false };
    bool ever_published{ false };
    uint64_t since_last_frame_ms{ 0 };
    uint64_t since_last_publish_ms{ 0 };
  };

  // can_id is compared against the two configured identifiers. They stay constructor
  // arguments even now that the allocation is assigned (wire contract 2026-08-02f):
  // nothing in this class may hard-code a literal, and the tests prove the numbers
  // carry no meaning by using arbitrary ones.
  tof_grid_assembler(uint32_t data_can_id, uint32_t health_can_id, uint64_t startup_time_ms);

  // Returns a grid only when every publish-gate condition holds. Anything doubtful is
  // rejected and counted rather than published with a warning: a partial or inconsistent
  // grid reads downstream as "no obstacle", so publishing it fails in the unsafe direction.
  std::optional<grid> consume(uint32_t can_id, uint8_t dlc, const uint8_t* payload, uint64_t now_ms);

  // Must be driven from a timer, independently of traffic. consume() cannot detect a
  // source that has gone silent, because with no frames arriving it is never called.
  void poll(uint64_t now_ms);

  status source_status(uint8_t source, uint64_t now_ms) const;

  uint32_t count(event e) const
  {
    return counters_[static_cast<size_t>(e)];
  }

  // Same contract as drain_events(): moved out, processed once.
  std::vector<health_report> drain_health_reports()
  {
    std::vector<health_report> out;
    out.swap(health_reports_);
    return out;
  }

  // Moves the queue out. Callers process each event exactly once; leaving them in place
  // would replay the whole history on every ROS spin.
  std::vector<diagnostic> drain_events()
  {
    std::vector<diagnostic> out;
    out.swap(events_);
    return out;
  }

  static const char* source_name(uint8_t source)
  {
    switch (source)
    {
      case SOURCE_FRONT_RIGHT:
        return "front-right";
      case SOURCE_FRONT_LEFT:
        return "front-left";
      default:
        return "unknown-source";
    }
  }

  // Pure: no state, no ROS, no time. Returns a bitmask over health_note.
  static uint8_t health_notes(const health_info& h);

  static const char* health_note_text(health_note n)
  {
    switch (n)
    {
      case health_note::I2C_ERROR_RECOVERED:
        return "sensor reported an I2C error that has since recovered";
      case health_note::DATA_READY_TIMEOUT_RECOVERED:
        return "sensor reported a data-ready timeout that has since recovered";
      case health_note::CHAIN_LENGTH_MISMATCH:
        return "chain length differs from the configured expectation";
      case health_note::PEER_ENUMERATION_FAILED:
        return "ANOTHER sensor on the chain failed enumeration";
      case health_note::LAST_ERROR_NONZERO:
        return "sensor reported a nonzero last error code";
      case health_note::HEALTH_NOTE_COUNT:
        break;
    }
    return "?";
  }

  // Slot keying for the report throttle, and the reason C2 existed: ROS_*_THROTTLE keeps its
  // state at the macro expansion site, so one call inside a loop is ONE window shared by every
  // source and every reason. A benign duplicate chunk could then swallow the first
  // HEALTH_COUNT_MISMATCH or CONFLICTING_CHUNK of the same window, and the operator would read
  // "duplicate chunk (identical, ignored)" while every grid was being discarded.
  //
  // The key is (reason, source). Unattributable diagnostics get their own slot per reason rather
  // than sharing source 0's, so a malformed frame from nowhere cannot silence a real fault on the
  // right-hand sensor.
  static constexpr size_t REPORT_REASONS =
      static_cast<size_t>(event::EVENT_COUNT) + static_cast<size_t>(health_note::HEALTH_NOTE_COUNT);
  static constexpr size_t REPORT_SLOTS = (static_cast<size_t>(SOURCE_COUNT) + 1) * REPORT_REASONS;

  static constexpr size_t report_slot(event e, uint8_t source)
  {
    return reason_slot(static_cast<size_t>(e), source);
  }

  static constexpr size_t report_slot(health_note n, uint8_t source)
  {
    return reason_slot(static_cast<size_t>(event::EVENT_COUNT) + static_cast<size_t>(n), source);
  }

  static const char* state_name(source_state s)
  {
    switch (s)
    {
      case source_state::NEVER_SEEN:
        return "NEVER_SEEN";
      case source_state::HEALTHY:
        return "HEALTHY";
      case source_state::STALE_NOT_COMPLETING:
        return "STALE_NOT_COMPLETING";
      case source_state::STALE_NO_FRAMES:
        return "STALE_NO_FRAMES";
    }
    return "?";
  }

private:
  struct slot
  {
    bool active{ false };
    uint8_t generation{ 0 };
    uint16_t bitmap{ 0 };
    uint64_t first_seen_ms{ 0 };
    bool health_seen{ false };
    health_info health{};
    std::array<uint16_t, ZONES> zones_mm{};
  };

  struct tracker
  {
    bool ever_seen_frame{ false };
    bool ever_published{ false };
    uint64_t last_frame_ms{ 0 };
    uint64_t last_publish_ms{ 0 };
    // True between raising an alarm and clearing it, so alarms are edge triggered: a ROS
    // layer polling at 10 Hz must not get one event per tick for an unchanged condition.
    bool alarm_active{ false };
    // A generation is single use. Once it ends, by publication or by any rejection, it is
    // retired so that a late or retransmitted frame cannot restart it. Only one is kept,
    // and it is never cleared, only overwritten when the next generation ends. That is
    // safe because generations increment once per grid: the value is replaced long before
    // the counter could wrap back onto it, and a source quiet enough for it to wrap has
    // already tripped the watchdog.
    bool retired_valid{ false };
    uint8_t retired_generation{ 0 };
  };

  static constexpr size_t reason_slot(size_t reason, uint8_t source)
  {
    const size_t s = source < SOURCE_COUNT ? source : static_cast<size_t>(SOURCE_COUNT);
    return reason * (static_cast<size_t>(SOURCE_COUNT) + 1) + s;
  }

  static health_info parse_health(const uint8_t* payload);
  static bool normalised_equal(const health_info& a, const health_info& b);
  void emit(event e, uint8_t source = SOURCE_COUNT, source_state state = source_state::NEVER_SEEN);
  void retire(uint8_t source);
  void expire_stale_slots(uint64_t now_ms);
  std::optional<grid> try_complete(uint8_t source, uint64_t now_ms);

  uint32_t data_can_id_;
  uint32_t health_can_id_;
  uint64_t startup_time_ms_;
  std::array<slot, SOURCE_COUNT> slots_{};
  std::array<tracker, SOURCE_COUNT> trackers_{};
  std::array<uint32_t, static_cast<size_t>(event::EVENT_COUNT)> counters_{};
  std::vector<diagnostic> events_;
  std::vector<health_report> health_reports_;
};

}  // namespace lexxhard
