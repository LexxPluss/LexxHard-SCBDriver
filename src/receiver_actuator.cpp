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
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "scbdriver/LinearActuatorServiceResponse.h"
#include "receiver_actuator.hpp"

receiver_actuator::receiver_actuator(ros::NodeHandle& n, ros::NodeHandle& pn)
  : pub_encoder{ n.advertise<std_msgs::Int32MultiArray>("/body_control/encoder_count", queue_size) }
  , pub_current{ n.advertise<std_msgs::Float32MultiArray>("/body_control/linear_actuator_current", queue_size) }
  , pub_src_resp{ n.advertise<scbdriver::LinearActuatorServiceResponse>("scbdriver/linear_actuator_service_response",
                                                                        queue_size) }
{
  pn.param<bool>("invert_center_actuator_direction", invert_center_actuator_direction, false);
  pn.param<bool>("invert_left_actuator_direction", invert_left_actuator_direction, false);
  pn.param<bool>("invert_right_actuator_direction", invert_right_actuator_direction, false);
}

int16_t receiver_actuator::adjust_encoder_count(size_t index, int16_t count) const
{
  // clang-format off
    bool const should_invert_tbl[] = {
        invert_center_actuator_direction,
        invert_left_actuator_direction,
        invert_right_actuator_direction
    };
  // clang-format on
  bool const should_invert{ should_invert_tbl[index] };

  if (should_invert)
  {
    return -count;
  }
  return count;
}

void receiver_actuator::handle(const can_frame& frame) const
{
  if (frame.can_id == 0x209)
  {
    handle_encoder_count(frame);
  }
  else if (frame.can_id == 0x20a)
  {
    handle_current(frame);
  }
  else if (frame.can_id == 0x213)
  {
    handle_service_response(frame);
  }
}

void receiver_actuator::handle_encoder_count(const can_frame& frame) const
{
  if (frame.can_dlc != 6)
    return;
  // ROS:[center,left,right], ROBOT:[left,center,right]
  int16_t C{ static_cast<int16_t>((frame.data[0] << 8) | frame.data[1]) };
  int16_t L{ static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]) };
  int16_t R{ static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]) };
  std_msgs::Int32MultiArray msg;
  msg.data.resize(3);
  msg.data[0] = adjust_encoder_count(0, C);
  msg.data[1] = adjust_encoder_count(1, L);
  msg.data[2] = adjust_encoder_count(2, R);
  pub_encoder.publish(msg);
}

void receiver_actuator::handle_current(const can_frame& frame) const
{
  if (frame.can_dlc != 8)
    return;
  // ROS:[center,left,right], ROBOT:[left,center,right]
  int16_t C{ static_cast<int16_t>((frame.data[0] << 8) | frame.data[1]) };
  int16_t L{ static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]) };
  int16_t R{ static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]) };
  std_msgs::Float32MultiArray msg_current;
  msg_current.data.resize(3);
  msg_current.data[0] = C * 1e-3f;
  msg_current.data[1] = L * 1e-3f;
  msg_current.data[2] = R * 1e-3f;
  pub_current.publish(msg_current);
}

void receiver_actuator::handle_service_response(const can_frame& frame) const
{
  if (frame.can_dlc != 8)
    return;

  scbdriver::LinearActuatorServiceResponse msg;
  msg.mode = frame.data[0];
  msg.success = frame.data[1] == 1;
  msg.detail.resize(3);
  msg.detail[0] = frame.data[2];
  msg.detail[1] = frame.data[3];
  msg.detail[2] = frame.data[4];
  msg.counter = frame.data[7];

  pub_src_resp.publish(msg);
}
