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
#include "std_msgs/Bool.h"
#include "receiver_gpio.hpp"

namespace
{
bool get_in_port_status(const can_frame& frame, const uint8_t port)
{
  const uint8_t bit_pos = 7 - port;
  return frame.data[0] & (1 << bit_pos);
}
}  // namespace

receiver_gpio::receiver_gpio(ros::NodeHandle& n)
  : pubs{ {
        n.advertise<std_msgs::Bool>("gpio/in_port0", queue_size),
        n.advertise<std_msgs::Bool>("gpio/in_port1", queue_size),
        n.advertise<std_msgs::Bool>("gpio/in_port2", queue_size),
        n.advertise<std_msgs::Bool>("gpio/in_port3", queue_size),
    } }
{
}

void receiver_gpio::handle(const can_frame& frame) const
{
  if (frame.can_id != 0x212)
  {
    return;
  }
  if (frame.can_dlc != 1)
  {
    return;
  }

  for (size_t i = 0; i < 4; ++i)
  {
    std_msgs::Bool msg;
    msg.data = get_in_port_status(frame, i);
    pubs[i].publish(msg);
  }
}
