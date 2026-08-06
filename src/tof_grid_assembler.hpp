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

namespace lexxhard {

class tof_grid_assembler {
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

  static constexpr uint32_t ASSEMBLY_TIMEOUT_MS = 300;
  // Deliberately independent of ASSEMBLY_TIMEOUT_MS: that one bounds how long a single
  // grid may take to arrive, this one bounds how long the robot may go without a usable
  // grid. They are not the same property and will not move together.
  static constexpr uint32_t SOURCE_STALE_MS = 1000;
  static constexpr uint32_t STARTUP_GRACE_MS = 3000;

  // Stable logical identity. NOT the chain position: the firmware owns the
  // chain_position -> source_id mapping and is the only place the physical topology
  // appears. Note that 0 is the RIGHT sensor, which reverses the legacy convention.
  enum source_id : uint8_t {
    SOURCE_FRONT_RIGHT = 0,
    SOURCE_FRONT_LEFT = 1,
    SOURCE_COUNT = 2,
  };

  enum class event : uint8_t {
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

  enum class source_state : uint8_t {
    NEVER_SEEN,
    HEALTHY,
    STALE_NOT_COMPLETING,
    STALE_NO_FRAMES,
  };

  // Reserved fields are already stripped: see parse_health.
  struct health_info {
    uint8_t valid_zone_count{0};
    uint8_t flags{0};
    uint8_t chain_position{0};
    uint8_t boards_detected{0};
    uint8_t last_error{0};
  };

  struct grid {
    uint8_t source{0};
    uint8_t generation{0};
    std::array<uint16_t, ZONES> zones_mm{};  // INVALID_MM where there is no target
    health_info health{};
  };

  // Carries who and what, not just that something happened. A bare SOURCE_STALE tells an
  // operator neither which sensor stopped nor whether frames are still arriving, which is
  // exactly the information the four-state model exists to provide.
  struct diagnostic {
    event kind{event::EVENT_COUNT};
    uint8_t source{SOURCE_COUNT};        // SOURCE_COUNT when not attributable to one
    source_state state{source_state::NEVER_SEEN};  // meaningful for the watchdog events
  };

  struct status {
    source_state state{source_state::NEVER_SEEN};
    // True only once poll() has actually raised an alarm for this source, and false again
    // once it has cleared. A state that is merely not HEALTHY is NOT a fault to report:
    // during the startup grace every source is legitimately NEVER_SEEN, and anything that
    // reports on state alone announces two broken sensors on every single boot.
    bool alarm_active{false};
    bool ever_published{false};
    uint32_t since_last_frame_ms{0};
    uint32_t since_last_publish_ms{0};
  };

  // can_id is compared against the two configured identifiers. They stay constructor
  // arguments even now that the allocation is assigned (wire contract 2026-08-02f):
  // nothing in this class may hard-code a literal, and the tests prove the numbers
  // carry no meaning by using arbitrary ones.
  tof_grid_assembler(uint32_t data_can_id, uint32_t health_can_id, uint32_t startup_time_ms);

  // Returns a grid only when every publish-gate condition holds. Anything doubtful is
  // rejected and counted rather than published with a warning: a partial or inconsistent
  // grid reads downstream as "no obstacle", so publishing it fails in the unsafe direction.
  std::optional<grid> consume(uint32_t can_id, uint8_t dlc, const uint8_t* payload,
                              uint32_t now_ms);

  // Must be driven from a timer, independently of traffic. consume() cannot detect a
  // source that has gone silent, because with no frames arriving it is never called.
  void poll(uint32_t now_ms);

  status source_status(uint8_t source, uint32_t now_ms) const;

  uint32_t count(event e) const { return counters_[static_cast<size_t>(e)]; }

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
    switch (source) {
    case SOURCE_FRONT_RIGHT: return "front-right";
    case SOURCE_FRONT_LEFT: return "front-left";
    default: return "unknown-source";
    }
  }

  static const char* state_name(source_state s)
  {
    switch (s) {
    case source_state::NEVER_SEEN: return "NEVER_SEEN";
    case source_state::HEALTHY: return "HEALTHY";
    case source_state::STALE_NOT_COMPLETING: return "STALE_NOT_COMPLETING";
    case source_state::STALE_NO_FRAMES: return "STALE_NO_FRAMES";
    }
    return "?";
  }

private:
  struct slot {
    bool active{false};
    uint8_t generation{0};
    uint16_t bitmap{0};
    uint32_t first_seen_ms{0};
    bool health_seen{false};
    health_info health{};
    std::array<uint16_t, ZONES> zones_mm{};
  };

  struct tracker {
    bool ever_seen_frame{false};
    bool ever_published{false};
    uint32_t last_frame_ms{0};
    uint32_t last_publish_ms{0};
    // True between raising an alarm and clearing it, so alarms are edge triggered: a ROS
    // layer polling at 10 Hz must not get one event per tick for an unchanged condition.
    bool alarm_active{false};
    // A generation is single use. Once it ends, by publication or by any rejection, it is
    // retired so that a late or retransmitted frame cannot restart it. Only one is kept,
    // and it is never cleared, only overwritten when the next generation ends. That is
    // safe because generations increment once per grid: the value is replaced long before
    // the counter could wrap back onto it, and a source quiet enough for it to wrap has
    // already tripped the watchdog.
    bool retired_valid{false};
    uint8_t retired_generation{0};
  };

  static health_info parse_health(const uint8_t* payload);
  static bool normalised_equal(const health_info& a, const health_info& b);
  void emit(event e, uint8_t source = SOURCE_COUNT,
            source_state state = source_state::NEVER_SEEN);
  void retire(uint8_t source);
  void expire_stale_slots(uint32_t now_ms);
  std::optional<grid> try_complete(uint8_t source, uint32_t now_ms);

  uint32_t data_can_id_;
  uint32_t health_can_id_;
  uint32_t startup_time_ms_;
  std::array<slot, SOURCE_COUNT> slots_{};
  std::array<tracker, SOURCE_COUNT> trackers_{};
  std::array<uint32_t, static_cast<size_t>(event::EVENT_COUNT)> counters_{};
  std::vector<diagnostic> events_;
};

}  // namespace lexxhard
