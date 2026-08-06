/*
 * Copyright (c) 2024, LexxPluss Inc.
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

#include <linux/can.h>
#include <atomic>
#include <thread>
#include <iostream>
#include "ros/ros.h"
#include "canif.hpp"
#include "uartif.hpp"
#include "receiver_actuator.hpp"
#include "receiver_bmu.hpp"
#include "receiver_board.hpp"
#include "receiver_dfu.hpp"
#include "receiver_imu.hpp"
#include "receiver_pgv.hpp"
#include "receiver_uss.hpp"
#include "receiver_gpio.hpp"
#include "receiver_tug_encoder.hpp"
#include "receiver_tof.hpp"
#include "can_ids.hpp"
#include "tof_can_config.hpp"
#include <chrono>
#include <optional>

namespace
{

class handler
{
public:
  handler(ros::NodeHandle& n, ros::NodeHandle& pn)
    : actuator{n, pn}, bmu{n}, board{n}, dfu{n}, imu{n}, pgv{n}, uss{n},
      gpio{n}, tug_encoder{n}, tof{n}
  {
  }

  void enable_tof_can(uint32_t data_id, uint32_t health_id, uint32_t now_ms)
  {
    tof_data_can_id = data_id;
    tof_health_can_id = health_id;
    tof.configure_can(data_id, health_id, now_ms);
  }

  void poll_tof(uint32_t now_ms) { tof.poll(now_ms); }

  void handle_can(const can_frame& frame, uint32_t now_ms)
  {
    if (tof_data_can_id && (frame.can_id == *tof_data_can_id || frame.can_id == *tof_health_can_id))
    {
      tof.handle_can(frame, now_ms);
      return;
    }
    // Routed through the shared table rather than a switch on literals, so an
    // identifier cannot be in the receive filter without also having a handler.
    switch (lexxhard::can_ids::route(frame.can_id))
    {
      using owner = lexxhard::can_ids::owner;
      case owner::bmu:         bmu.handle(frame); break;
      case owner::pgv:         pgv.handle(frame); break;
      case owner::uss:         uss.handle(frame); break;
      case owner::imu:         imu.handle(frame); break;
      case owner::actuator:    actuator.handle(frame); break;
      case owner::board:       board.handle(frame); break;
      case owner::dfu:         dfu.handle(frame); break;
      case owner::tug_encoder: tug_encoder.handle(frame); break;
      case owner::gpio:        gpio.handle(frame); break;
      default:
        break;
    }
  }

  void handle_uart(const std::vector<uint8_t>& packet)
  {
    tof.handle(packet);
  }

private:
  std::optional<uint32_t> tof_data_can_id;
  std::optional<uint32_t> tof_health_can_id;
  receiver_actuator actuator;
  receiver_bmu bmu;
  receiver_board board;
  receiver_dfu dfu;
  receiver_imu imu;
  receiver_pgv pgv;
  receiver_uss uss;
  receiver_gpio gpio;
  receiver_tug_encoder tug_encoder;
  receiver_tof tof;
};

uint32_t monotonic_ms()
{
  using namespace std::chrono;
  return static_cast<uint32_t>(
      duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

tof_transport resolve_tof_transport(ros::NodeHandle& pn)
{
  std::string const mode = pn.param<std::string>("tof_transport", "");
  bool const legacy_flag_present = pn.hasParam("use_tof_sensor_board");
  bool const legacy_flag = pn.param<bool>("use_tof_sensor_board", false);

  if (mode.empty())
  {
    if (legacy_flag_present)
      ROS_WARN("use_tof_sensor_board is deprecated; set tof_transport to "
               "disabled, legacy_uart or scb_can instead");
    return legacy_flag ? tof_transport::legacy_uart : tof_transport::disabled;
  }

  tof_transport resolved;
  if (mode == "disabled")
    resolved = tof_transport::disabled;
  else if (mode == "legacy_uart")
    resolved = tof_transport::legacy_uart;
  else if (mode == "scb_can")
    resolved = tof_transport::scb_can;
  else
  {
    ROS_FATAL("tof_transport must be disabled, legacy_uart or scb_can, got '%s'", mode.c_str());
    std::exit(1);
  }

  // Both set and disagreeing is a configuration the operator cannot have meant either way.
  if (legacy_flag_present && legacy_flag != (resolved == tof_transport::legacy_uart))
  {
    ROS_FATAL("tof_transport='%s' contradicts use_tof_sensor_board=%s; remove the "
              "deprecated parameter", mode.c_str(), legacy_flag ? "true" : "false");
    std::exit(1);
  }
  return resolved;
}

// The identifiers are assigned (wire contract 2026-08-02f: 0x214/0x215, a
// team-authorized self-assigned integration allocation) and are the defaults here.
// The pair is atomic configuration: a launch file overrides both (bench) or neither
// (production). Overriding only one would silently mix an override with a default —
// refused, because the two ends of such a split configuration have never been tested
// together and never will be.
bool read_tof_can_ids(ros::NodeHandle& pn, uint32_t& data_id, uint32_t& health_id)
{
  bool const has_data = pn.hasParam("tof_can_data_id");
  bool const has_health = pn.hasParam("tof_can_health_id");

  int data_raw = lexxhard::TOF_GRID_DATA_ID;
  int health_raw = lexxhard::TOF_GRID_HEALTH_ID;
  bool data_parsed = true, health_parsed = true;
  if (has_data && has_health)
  {
    // getParam returns false for a parameter of the wrong type and leaves the output
    // untouched; ignoring that would silently keep the default and recreate exactly
    // the half-override the pair check below forbids.
    data_parsed = pn.getParam("tof_can_data_id", data_raw);
    health_parsed = pn.getParam("tof_can_health_id", health_raw);
  }

  if (std::string const reason =
          lexxhard::check_tof_id_param_pair(has_data, has_health, data_parsed, health_parsed);
      !reason.empty())
  {
    ROS_FATAL("%s", reason.c_str());
    return false;
  }

  lexxhard::tof_can_ids ids;
  std::string const reason = lexxhard::validate_tof_can_ids(data_raw, health_raw, ids);
  if (!reason.empty())
  {
    ROS_FATAL("%s", reason.c_str());
    return false;
  }
  data_id = ids.data_id;
  health_id = ids.health_id;
  return true;
}

}  // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  std::string const tof_uart_port = pn.param<std::string>("tof_sensor_board_uart_port", "/dev/ttyACM0");
  uint32_t const tof_baudrate = static_cast<uint32_t>(pn.param<int>("tof_sensor_board_baudrate", 115200));

  tof_transport const transport = resolve_tof_transport(pn);
  bool const use_tof_sensor_board = transport == tof_transport::legacy_uart;

  handler handler{n, pn};

  // CAN setup. Note the interface name: the SCB calls this bus CAN2 at 1 Mbit/s, but on
  // the IPC it is can1. The two ends name the same physical bus differently.
  canif::queue_type can_queue;
  canif can{can_queue};
  std::vector<can_filter> filter;
  filter.reserve(lexxhard::can_ids::kTableCount + 2);
  for (size_t i = 0; i < lexxhard::can_ids::kTableCount; ++i)
  {
    const auto& e = lexxhard::can_ids::kTable[i];
    if (e.dir == lexxhard::can_ids::direction::rx)
      filter.push_back({e.id, CAN_SFF_MASK});
  }

  if (transport == tof_transport::scb_can)
  {
    uint32_t data_id = 0, health_id = 0;
    if (!read_tof_can_ids(pn, data_id, health_id))
    {
      // Deliberately fatal rather than falling back to disabled. A configuration that
      // claims the feature is on while nothing is detecting anything is the worst
      // possible outcome for an obstacle sensor.
      return 1;
    }
    filter.push_back({data_id, CAN_SFF_MASK});
    filter.push_back({health_id, CAN_SFF_MASK});
    handler.enable_tof_can(data_id, health_id, monotonic_ms());
    ROS_INFO("ToF transport: scb_can, data id 0x%03x, health id 0x%03x", data_id, health_id);
  }
  else
  {
    ROS_INFO("ToF transport: %s",
             transport == tof_transport::legacy_uart ? "legacy_uart" : "disabled");
  }

  if (can.init("can1", filter.data(), filter.size() * sizeof(can_filter)) < 0)
  {
    return -1;
  }

  // UART setup
  uartif::queue_type uart_queue;
  uartif uart{tof_uart_port, tof_baudrate, uart_queue};
  if (use_tof_sensor_board)
  {
    if (uart.init() < 0)
    {
      return -1;
    }
  }

  uint32_t last_tof_poll_ms = monotonic_ms();
  constexpr uint32_t tof_poll_interval_ms = 100;

  // Start I/O threads
  std::atomic<bool> running{true};
  std::thread can_thread{[&] {
    while (running.load(std::memory_order_relaxed))
    {
      can.poll(1);
    }
  }};
  std::thread uart_thread{[&] {
    while (running.load(std::memory_order_relaxed))
    {
      uart.poll(1);
    }
  }};

  // Main loop
  while (ros::ok())
  {
    can_frame frame;
    while (can_queue.pop(frame))
    {
      handler.handle_can(frame, monotonic_ms());
    }

    std::vector<uint8_t> packet;
    while (uart_queue.pop(packet))
    {
      handler.handle_uart(packet);
    }

    // The watchdog has to run whether or not frames are arriving; a silent source is
    // precisely the case consume() can never see. Throttled because the surrounding loop
    // does not rate limit itself.
    uint32_t const now = monotonic_ms();
    if (now - last_tof_poll_ms >= tof_poll_interval_ms)
    {
      last_tof_poll_ms = now;
      handler.poll_tof(now);
    }

    ros::spinOnce();
  }

  running.store(false, std::memory_order_relaxed);
  can_thread.join();
  uart_thread.join();

  can.term();
  uart.term();

  return 0;
}
