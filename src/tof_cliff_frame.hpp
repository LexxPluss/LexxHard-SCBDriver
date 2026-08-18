// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

// Frame-level decode for the cliff ToF CAN contract.
//
// SCOPE, DELIBERATELY NARROW
//
// This validates one frame and unpacks its fields. That is all. It does not assemble
// cycles, does not retire them, does not track staleness, does not know what READY means
// and holds no state whatsoever -- the same bytes give the same answer however often they
// arrive, which the tests assert rather than assume. All of that belongs to the state
// machine, which cannot be built yet: every timing value it needs is still unresolved and
// the contract's event-multiset vectors do not exist. Nothing in the layout vectors
// implies any of it, and it must not be inferred from them.
//
// ONE ENTRY POINT, AND WHY
//
// decode() takes the identifier the frame arrived on and picks the decoder from it. The
// per-type decoders are deliberately not exposed: with them public, a caller could hand a
// measurement-shaped payload that arrived on the health identifier to the measurement
// decoder and get an accept, which is precisely the mis-routing that frame_type exists to
// catch. Binding the identifier to the decoder inside this file means the frame_type check
// is unavoidable rather than merely available.
//
// The validation order is the contract's stated order, and the layout vectors pin which
// verdict each frame must receive. The status classification table, the target limit and
// the verdict vocabulary come from the vendored production header rather than being
// restated here -- the firmware's packer uses the same table from the same generated file,
// which is what makes the two sides agree by construction instead of by review.
//
// On any verdict other than accept the output is left at its default. A half-filled result
// is worse than none: it looks decoded to anything that forgets to check.

#include <cstdint>

#include "tof_cliff_contract.h"

namespace tof_cliff_frame {

using tof_cliff_contract::status_class;
using tof_cliff_contract::verdict;

// Which of the contract's identifiers a frame arrived on. `not_ours` is separate from
// every verdict on purpose: "not a cliff frame" and "a malformed cliff frame" are
// different facts, and a decoder that collapsed them would let a routing bug read as a
// protocol error.
enum class arrival : uint8_t {
    measurement,
    health,
    not_ours,
};

struct measurement {
    uint8_t source_id{0};
    uint8_t mapping_epoch{0};
    uint8_t cycle_seq{0};
    uint16_t range_mm{0};
    uint8_t raw_status{0};
    uint8_t target_count{0};
    // The class is the safety-relevant fact; a consumer must branch on it and never
    // re-derive it from the raw status.
    status_class cls{status_class::no_target};
    // Names the encoding, not the meaning. Both NO_TARGET and SENSOR_FAULT carry the
    // sentinel, so this being true does NOT mean "no target was found" -- read `cls` for
    // that. It is spelled this way because the obvious name, no_target, would have let a
    // hardware fault read downstream as an ordinary empty floor.
    bool range_is_sentinel{false};
};

struct health {
    uint8_t protocol_version{0};
    uint8_t mapping_epoch{0};
    uint8_t health_seq{0};
    uint8_t mapping_state{0};
    uint8_t flags{0};
    uint8_t enumerated_mask{0};
    uint8_t model_verified_mask{0};
    uint8_t sample_produced_mask{0};
    uint8_t sensor_fault_mask{0};
    uint8_t failing_chain_position{0};
    uint8_t cycle_seq{0};
    // Derived from `flags` so a consumer does not repeat the bit arithmetic. A chain fault
    // is bits 0-2 only: cycle_valid is not a fault and must not be counted as one.
    bool cycle_valid{false};
    bool chain_fault{false};
};

struct decoded {
    arrival which{arrival::not_ours};
    // Meaningful only when `which` is not not_ours. Left at accept otherwise, and reading
    // it in that case is a caller bug -- check `which` first.
    verdict result{verdict::accept};
    measurement meas{};
    health state{};
};

// The only entry point. `data` must point to at least `dlc` bytes; a dlc other than 8 is
// rejected before any byte is read.
decoded decode(uint32_t can_id, uint8_t dlc, const uint8_t* data);

}  // namespace tof_cliff_frame
