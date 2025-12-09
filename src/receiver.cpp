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

  void handle_can(const can_frame& frame)
  {
    switch (frame.can_id)
    {
      case 0x100:
      case 0x101:
      case 0x103:
      case 0x110:
      case 0x111:
      case 0x112:
      case 0x113:
      case 0x120:
      case 0x130:
        bmu.handle(frame);
        break;
      case 0x200:
      case 0x201:
      case 0x202:
        pgv.handle(frame);
        break;
      case 0x204:
        uss.handle(frame);
        break;
      case 0x206:
      case 0x207:
        imu.handle(frame);
        break;
      case 0x209:
      case 0x20a:
      case 0x213:
        actuator.handle(frame);
        break;
      case 0x20c:
        board.handle(frame);
        break;
      case 0x20e:
        dfu.handle(frame);
        break;
      case 0x210:
        tug_encoder.handle(frame);
        break;
      case 0x212:
        gpio.handle(frame);
        break;
      default:
        break;
    }
  }

  void handle_uart(const std::vector<uint8_t>& packet)
  {
    tof.handle(packet);
  }

private:
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

bool canif_configure(canif& can, handler& handler)
{
  can.set_handler([&](const can_frame& frame) { handler.handle_can(frame); });
  // clang-format off
  can_filter filter[]{
      {0x100, CAN_SFF_MASK},
      {0x101, CAN_SFF_MASK},
      {0x103, CAN_SFF_MASK},
      {0x110, CAN_SFF_MASK},
      {0x111, CAN_SFF_MASK},
      {0x112, CAN_SFF_MASK},
      {0x113, CAN_SFF_MASK},
      {0x120, CAN_SFF_MASK},
      {0x130, CAN_SFF_MASK},
      {0x200, CAN_SFF_MASK},
      {0x201, CAN_SFF_MASK},
      {0x202, CAN_SFF_MASK},
      {0x204, CAN_SFF_MASK},
      {0x206, CAN_SFF_MASK},
      {0x207, CAN_SFF_MASK},
      {0x209, CAN_SFF_MASK},
      {0x20a, CAN_SFF_MASK},
      {0x20c, CAN_SFF_MASK},
      {0x20e, CAN_SFF_MASK},
      {0x210, CAN_SFF_MASK},
      {0x212, CAN_SFF_MASK},
      {0x213, CAN_SFF_MASK},
  };
  // clang-format on

  if (can.init(filter, sizeof filter) < 0)
  {
    std::cerr << "canif::init() failed" << std::endl;
    return false;
  }

  return true;
}

bool uartif_configure(uartif& uart, handler& handler)
{
  uart.set_handler([&](const std::vector<uint8_t>& packet) { handler.handle_uart(packet); });

  if (uart.init() < 0)
  {
    std::cerr << "uartif::init() failed" << std::endl;
    return false;
  }

  return true;
}

}  // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Get ToF sensor board parameters
  bool const use_tof_sensor_board = pn.param<bool>("use_tof_sensor_board", false);
  std::string const tof_sensor_board_uart_port = pn.param<std::string>("tof_sensor_board_uart_port", "/dev/ttyACM0");
  uint32_t const tof_sensor_board_baudrate = static_cast<uint32_t>(pn.param<int>("tof_sensor_board_baudrate", 115200));

  handler handler{n, pn};

  // Initialize CAN interface
  canif can;
  if (!canif_configure(can, handler))
  {
    return -1;
  }

  // Initialize UART interface (for ToF sensor board)
  uartif uart{tof_sensor_board_uart_port, tof_sensor_board_baudrate};
  if (use_tof_sensor_board)
  {
    uartif_configure(uart, handler);
  }

  while (ros::ok())
  {
    can.poll(10);
    uart.poll(10);
    ros::spinOnce();
  }

  can.term();
  uart.term();

  return 0;
}