/*
 * Copyright (c) 2025, LexxPluss Inc.
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

#pragma once

#include <ros/ros.h>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "tof_grid_assembler.hpp"
#include "tof_report_throttle.hpp"

struct can_frame;

// A single enum rather than a pair of booleans. Two independent flags could both be on,
// and then the UART path and the CAN path would publish to the same topics at once, with
// no way for a subscriber to tell which reading it had.
enum class tof_transport
{
  disabled,
  legacy_uart,  // the standalone Nucleo ToF board, SLIP over USB serial
  scb_can,      // the differential-I2C chain on the SCB, reassembled from CAN
};

class receiver_tof
{
public:
  receiver_tof(ros::NodeHandle& n);

  // Only called for tof_transport::scb_can. The identifiers arrive from the caller: the
  // assigned allocation (wire contract 2026-08-02f) is the parameter-pair default in
  // receiver.cpp, and nothing below that layer hard-codes a value.
  void configure_can(uint32_t data_can_id, uint32_t health_can_id, uint64_t now_ms);

  void handle(const std::vector<uint8_t>& packet);  // legacy UART/SLIP path
  void handle_can(const can_frame& frame, uint64_t now_ms);

  // Drives the watchdog. Must be called even when no frames are arriving; that is the
  // only case it exists for.
  void poll(uint64_t now_ms);

private:
  void publish_grid(const lexxhard::tof_grid_assembler::grid& g);
  void drain_diagnostics(uint64_t now_ms);
  void report_health_reports(uint64_t now_ms);
  void report_persistent_state(uint64_t now_ms);

  static constexpr int queue_size{ 10 };
  ros::Publisher pub_tof_front;
  ros::Publisher pub_tof_rear;
  ros::Publisher pub_low_object_left;
  ros::Publisher pub_low_object_right;
  std::unique_ptr<lexxhard::tof_grid_assembler> assembler;
  // Not ROS_*_THROTTLE: see tof_report_throttle.hpp for why one call site inside a loop
  // over the sources silently starves every source but the first.
  lexxhard::report_throttle<lexxhard::tof_grid_assembler::SOURCE_COUNT> state_throttle{ 5000 };
  // Keyed by (reason, source), so a benign duplicate cannot swallow the first conflict or count
  // mismatch of the same window, and the two sensors never suppress each other. Covers both the
  // assembler's events and the health frame's self-reported notes; see report_slot().
  lexxhard::report_throttle<lexxhard::tof_grid_assembler::REPORT_SLOTS> report_throttle_{ 5000 };
};
