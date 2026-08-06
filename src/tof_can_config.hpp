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

// Validation of the ToF CAN identifiers, kept free of ROS so it can be tested. The
// caller reads the parameters and reports whatever reason comes back.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace lexxhard {

// Default identifiers = the assigned allocation (firmware wire contract, version
// 2026-08-02f; team-authorized self-assignment, see can_ids.hpp for the registered
// rows). A launch file may override the PAIR for bench use; overriding only one of
// the two is refused by the caller. These must stay equal to the registered
// can_ids::TOF_GRID_* values -- a test asserts it.
inline constexpr int TOF_GRID_DATA_ID{0x214};
inline constexpr int TOF_GRID_HEALTH_ID{0x215};

struct tof_can_ids {
  uint32_t data_id{0};
  uint32_t health_id{0};
};

// Returns an empty string on success, otherwise the reason the configuration is refused.
//
// Conflicts are checked against can_ids::kTable, which covers both directions. Taking
// only the receive filter would miss the transmitted identifiers: the receiver would
// accept such a value happily while the frames collided on the bus, appearing as
// intermittent corruption with nothing obviously wrong in this node.
std::string validate_tof_can_ids(int data_raw, int health_raw, tof_can_ids& out);

// Pair-atomicity of the two override parameters, kept free of ROS so it can be tested.
// has_* say whether the parameter exists at all; *_parsed say whether getParam()
// returned it as an integer (pass true when no override was attempted). Empty string
// when the configuration is coherent — either no override, or a fully parsed pair.
// Anything else names the parameter at fault: presence of one without the other, and
// just as important, a parameter that exists but fails integer parsing, which would
// otherwise silently leave its default in place and recreate the half-override this
// check exists to forbid.
std::string check_tof_id_param_pair(bool has_data, bool has_health,
                                    bool data_parsed, bool health_parsed);

}  // namespace lexxhard
