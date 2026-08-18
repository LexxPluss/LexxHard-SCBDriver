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
// cycles, does not retire them, does not track staleness, does not know what READY
// means and holds no state whatsoever -- two calls with the same bytes give the same
// answer. All of that belongs to the state machine, which cannot be built yet: every
// timing value it needs is still unresolved, and the contract's event-multiset vectors
// do not exist. Nothing in the layout vectors implies any of it, and it must not be
// inferred from them.
//
// The validation order is the contract's stated order, and the layout vectors pin which
// verdict each frame must receive. The status classification table and the verdict
// vocabulary come from the vendored production header rather than being restated here --
// the firmware's packer uses the same table from the same generated file, which is what
// makes the two sides agree by construction instead of by review.
//
// On any verdict other than accept the output struct is left untouched. A half-filled
// result is worse than none: it looks decoded to anything that forgets to check.

#include <cstdint>

#include "vendor/tof_cliff_contract.h"

namespace tof_cliff_frame {

using tof_cliff_contract::status_class;
using tof_cliff_contract::verdict;

// Which of the contract's identifiers a frame arrived on. `not_ours` exists so the
// caller can tell "not a cliff frame" from "a malformed cliff frame" -- routing a frame
// away from its owner and silently mis-decoding it are different bugs.
enum class arrival : uint8_t {
    measurement,
    health,
    not_ours,
};

arrival classify_identifier(uint32_t can_id);

struct measurement {
    uint8_t source_id{0};
    uint8_t mapping_epoch{0};
    uint8_t cycle_seq{0};
    uint16_t range_mm{0};
    uint8_t raw_status{0};
    uint8_t target_count{0};
    // Derived, so a consumer never re-implements the classification. `no_target` is the
    // sentinel test, kept separate from the class because a NO_TARGET class always
    // carries the sentinel while a SENSOR_FAULT class does too -- the class says why.
    status_class cls{status_class::no_target};
    bool no_target{false};
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
    // Derived from `flags`, so a consumer does not repeat the bit arithmetic. A chain
    // fault is bits 0-2 only: cycle_valid is not a fault and must not be counted as one.
    bool cycle_valid{false};
    bool chain_fault{false};
};

// `data` must point to at least 8 bytes when dlc is 8. A short dlc is rejected before
// any byte is read, which is why the length check comes first.
verdict decode_measurement(uint8_t dlc, const uint8_t* data, measurement& out);
verdict decode_health(uint8_t dlc, const uint8_t* data, health& out);

}  // namespace tof_cliff_frame
