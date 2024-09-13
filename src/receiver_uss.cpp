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
#include "std_msgs/Float64MultiArray.h"
#include "receiver_uss.hpp"

receiver_uss::receiver_uss(ros::NodeHandle& n)
  : pub{ n.advertise<std_msgs::Float64MultiArray>("/sensor_set/ultrasonic", queue_size) }
{
}

void receiver_uss::handle(const can_frame& frame) const
{
  if (frame.can_dlc != 8)
    return;
  uint64_t work{ 0 };
  for (int i{ 0 }; i < 8; ++i)
  {
    work <<= 8;
    work |= frame.data[i];
  }
  std_msgs::Float64MultiArray msg;
  msg.data.resize(5);
  msg.data[0] = ((work >> 52) & 0xfff) * 2.0 * 1e-3;
  msg.data[1] = ((work >> 40) & 0xfff) * 2.0 * 1e-3;
  msg.data[2] = ((work >> 28) & 0xfff) * 2.0 * 1e-3;
  msg.data[3] = ((work >> 16) & 0xfff) * 2.0 * 1e-3;
  msg.data[4] = ((work >> 4) & 0xfff) * 2.0 * 1e-3;
  pub.publish(msg);
}
