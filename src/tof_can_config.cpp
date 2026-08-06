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

#include "tof_can_config.hpp"

#include <linux/can.h>
#include <cstdio>

#include "can_ids.hpp"

namespace lexxhard {

namespace {

std::string hex(uint32_t v)
{
  char buf[16];
  snprintf(buf, sizeof buf, "0x%03x", v);
  return buf;
}

std::string check_one(const char* name, int raw, uint32_t& out)
{
  // One test catches a negative, anything above 0x7ff, and the EFF, RTR and ERR flag
  // bits, which live above the 11-bit field and would change what the filter matches
  // without changing how the value reads in a launch file.
  if (raw < 0 || (static_cast<uint32_t>(raw) & ~static_cast<uint32_t>(CAN_SFF_MASK)) != 0)
    return std::string{name} + " must be an 11-bit identifier in 0x000..0x7ff with no "
                               "EFF, RTR or ERR flag";
  out = static_cast<uint32_t>(raw);
  return {};
}

}  // namespace

std::string validate_tof_can_ids(int data_raw, int health_raw, tof_can_ids& out)
{
  if (std::string e = check_one("tof_can_data_id", data_raw, out.data_id); !e.empty())
    return e;
  if (std::string e = check_one("tof_can_health_id", health_raw, out.health_id); !e.empty())
    return e;

  if (out.data_id == out.health_id)
    return "tof_can_data_id and tof_can_health_id must differ, both are " + hex(out.data_id);

  // The ToF branch in handle_can runs before the switch, so a colliding identifier does
  // not merely double-handle a frame: it takes it away from its real owner entirely, and
  // silently. Configuring 0x204 here would stop the ultrasonic frames ever reaching their
  // receiver, and 0x20F would corrupt the board command this driver transmits.
  //
  // The exemption is per role: each parameter may use its OWN registered allocation
  // (can_ids::TOF_GRID_DATA / TOF_GRID_HEALTH) or any identifier the table does not
  // know. Swapping the two registered values, or taking the reserved drop-sense id,
  // is refused like any other collision.
  if (out.data_id != can_ids::TOF_GRID_DATA && can_ids::is_taken(out.data_id))
    return "ToF CAN identifier " + hex(out.data_id) + " is already in use on this bus";
  if (out.health_id != can_ids::TOF_GRID_HEALTH && can_ids::is_taken(out.health_id))
    return "ToF CAN identifier " + hex(out.health_id) + " is already in use on this bus";
  return {};
}

std::string check_tof_id_param_pair(bool has_data, bool has_health,
                                    bool data_parsed, bool health_parsed)
{
  if (has_data != has_health)
    return std::string{"tof_can_data_id and tof_can_health_id are a pair: override both "
                       "or neither ("} +
           (has_data ? "tof_can_data_id" : "tof_can_health_id") +
           " is set, the other is not)";
  if (has_data && (!data_parsed || !health_parsed)) {
    std::string which;
    if (!data_parsed && !health_parsed)
      which = "tof_can_data_id and tof_can_health_id";
    else
      which = !data_parsed ? "tof_can_data_id" : "tof_can_health_id";
    return which + " must be an integer parameter; a failed parse would silently keep "
                   "the default and mix an override with a default";
  }
  return {};
}

}  // namespace lexxhard
