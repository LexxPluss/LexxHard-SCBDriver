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

#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

class canif;

class sender_board
{
public:
  sender_board(ros::NodeHandle& n, canif& can);

private:
  void handle_ems(const std_msgs::Bool::ConstPtr& msg);
  void handle_power_off(const std_msgs::Bool::ConstPtr& msg);
  void handle_wheel_off(const std_msgs::String::ConstPtr& msg);
  void handle_heartbeat(const std_msgs::Bool::ConstPtr& msg);
  void handle_lockdown(const std_msgs::Bool::ConstPtr& msg);
  ros::Subscriber sub_ems, sub_power_off, sub_wheel_off, sub_heartbeat, sub_lockdown;
  canif& can;
  static constexpr uint32_t queue_size{ 10 };
  can_frame frame{ .can_id{ 0x20F }, .can_dlc{ 5 }, .data{ 0 } };
};
