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

// The single list of CAN identifiers this driver uses on can1, with a direction and an
// owner for each.
//
// It exists because the identifiers used to be literals scattered across every sender,
// receiver and handler, and anything wanting the full picture had to copy them by hand.
// That copy drifted at once: a hand-written table missed 0x20F and 0x211 purely because
// they are declared in headers rather than .cpp files, and a ToF configuration colliding
// with either would have been accepted.
//
// Everything now derives from kTable: the SocketCAN receive filter, the dispatch in
// receiver.cpp, and the ToF identifier conflict check. Adding an identifier means adding
// one row here, and a row cannot be filtered without also being routed.

#pragma once

#include <cstddef>
#include <cstdint>

namespace lexxhard {
namespace can_ids {

enum class direction : uint8_t {
  rx,  // received from the SCB, so it belongs in the filter and needs a route
  tx,  // transmitted to the SCB; invisible to the filter, but still taken on the bus
};

enum class owner : uint8_t {
  none,  // transmit-only rows have no receive route
  bmu,
  pgv,
  uss,
  imu,
  actuator,
  board,
  dfu,
  tug_encoder,
  gpio,
  led,
};

struct entry {
  uint32_t id;
  direction dir;
  owner who;
};

// Received.
constexpr uint32_t BMU_0 = 0x100;
constexpr uint32_t BMU_1 = 0x101;
constexpr uint32_t BMU_2 = 0x103;
constexpr uint32_t BMU_3 = 0x110;
constexpr uint32_t BMU_4 = 0x111;
constexpr uint32_t BMU_5 = 0x112;
constexpr uint32_t BMU_6 = 0x113;
constexpr uint32_t BMU_7 = 0x120;
constexpr uint32_t BMU_8 = 0x130;
constexpr uint32_t PGV_RX_0 = 0x200;
constexpr uint32_t PGV_RX_1 = 0x201;
constexpr uint32_t PGV_RX_2 = 0x202;
constexpr uint32_t USS = 0x204;
constexpr uint32_t IMU_0 = 0x206;
constexpr uint32_t IMU_1 = 0x207;
constexpr uint32_t ACTUATOR_RX_0 = 0x209;
constexpr uint32_t ACTUATOR_RX_1 = 0x20a;
constexpr uint32_t BOARD_RX = 0x20c;
constexpr uint32_t DFU_RX = 0x20e;
constexpr uint32_t TUG_ENCODER = 0x210;
constexpr uint32_t GPIO_RX = 0x212;
constexpr uint32_t ACTUATOR_RX_2 = 0x213;

// Transmitted. Not in the filter, so a collision with one of these is invisible to
// anything that only inspects the filter, yet it still corrupts the bus.
constexpr uint32_t PGV_TX = 0x203;
constexpr uint32_t LED_TX = 0x205;
constexpr uint32_t ACTUATOR_TX_0 = 0x208;
constexpr uint32_t ACTUATOR_TX_1 = 0x20b;
constexpr uint32_t DFU_TX = 0x20d;
constexpr uint32_t BOARD_TX = 0x20F;
constexpr uint32_t GPIO_TX = 0x211;

constexpr entry kTable[]{
    {BMU_0, direction::rx, owner::bmu},
    {BMU_1, direction::rx, owner::bmu},
    {BMU_2, direction::rx, owner::bmu},
    {BMU_3, direction::rx, owner::bmu},
    {BMU_4, direction::rx, owner::bmu},
    {BMU_5, direction::rx, owner::bmu},
    {BMU_6, direction::rx, owner::bmu},
    {BMU_7, direction::rx, owner::bmu},
    {BMU_8, direction::rx, owner::bmu},
    {PGV_RX_0, direction::rx, owner::pgv},
    {PGV_RX_1, direction::rx, owner::pgv},
    {PGV_RX_2, direction::rx, owner::pgv},
    {USS, direction::rx, owner::uss},
    {IMU_0, direction::rx, owner::imu},
    {IMU_1, direction::rx, owner::imu},
    {ACTUATOR_RX_0, direction::rx, owner::actuator},
    {ACTUATOR_RX_1, direction::rx, owner::actuator},
    {ACTUATOR_RX_2, direction::rx, owner::actuator},
    {BOARD_RX, direction::rx, owner::board},
    {DFU_RX, direction::rx, owner::dfu},
    {TUG_ENCODER, direction::rx, owner::tug_encoder},
    {GPIO_RX, direction::rx, owner::gpio},
    {PGV_TX, direction::tx, owner::none},
    {LED_TX, direction::tx, owner::none},
    {ACTUATOR_TX_0, direction::tx, owner::none},
    {ACTUATOR_TX_1, direction::tx, owner::none},
    {DFU_TX, direction::tx, owner::none},
    {BOARD_TX, direction::tx, owner::none},
    {GPIO_TX, direction::tx, owner::none},
};
constexpr size_t kTableCount = sizeof(kTable) / sizeof(entry);

// Which receiver a frame belongs to, or owner::none if this driver does not receive it.
// A lookup rather than a switch so that the filter and the routing cannot disagree: both
// read the same rows.
inline owner route(uint32_t id)
{
  for (size_t i = 0; i < kTableCount; ++i) {
    if (kTable[i].id == id && kTable[i].dir == direction::rx)
      return kTable[i].who;
  }
  return owner::none;
}

inline bool is_taken(uint32_t id)
{
  for (size_t i = 0; i < kTableCount; ++i) {
    if (kTable[i].id == id)
      return true;
  }
  return false;
}

}  // namespace can_ids
}  // namespace lexxhard
