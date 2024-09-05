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
#include "std_msgs/Byte.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "receiver_board.hpp"

receiver_board::receiver_board(ros::NodeHandle &n)
    : pub_bumper{n.advertise<std_msgs::ByteMultiArray>("/sensor_set/bumper", queue_size)},
      pub_emergency_switch{n.advertise<std_msgs::Bool>("/sensor_set/emergency_switch", queue_size)},
      pub_emergency_stop{n.advertise<std_msgs::Bool>("/body_control/emergency_stop", queue_size)},
      pub_charge{n.advertise<std_msgs::Byte>("/body_control/charge_status", queue_size)},
      pub_power{n.advertise<std_msgs::Byte>("/body_control/power_state", queue_size)},
      pub_charge_delay{n.advertise<std_msgs::UInt8>("/body_control/charge_heartbeat_delay", queue_size)},
      pub_charge_voltage{n.advertise<std_msgs::Float32>("/body_control/charge_connector_voltage", queue_size)},
      pub_safety_lidar{n.advertise<std_msgs::Bool>("/sensor_set/safety_lidar", queue_size)}
{
}

void receiver_board::handle(const can_frame &frame) const
{
    if (frame.can_dlc != 6)
        return;
    publish_bumper(frame);
    publish_emergency_switch(frame);
    publish_emergency_stop(frame);
    publish_charge(frame);
    publish_power(frame);
    publish_charge_delay(frame);
    publish_charge_voltage(frame);
    publish_safety_lidar(frame);
}

void receiver_board::publish_bumper(const can_frame &frame) const
{
    std_msgs::ByteMultiArray msg;
    msg.data.resize(2);
    msg.data[0] = (frame.data[0] & 0b10000000) != 0;
    msg.data[1] = (frame.data[0] & 0b01000000) != 0;
    pub_bumper.publish(msg);
}

void receiver_board::publish_emergency_switch(const can_frame &frame) const
{
    std_msgs::Bool msg;
    msg.data = (frame.data[0] & 0b00110000) != 0;
    pub_emergency_switch.publish(msg);
}

void receiver_board::publish_emergency_stop(const can_frame &frame) const
{
    std_msgs::Bool msg;
    msg.data = (frame.data[0] & 0b00000100) != 0;
    pub_emergency_stop.publish(msg);
}

void receiver_board::publish_charge(const can_frame &frame) const
{
    static constexpr uint8_t MANUAL_CHARGE_STATE{2}, AUTO_CHARGE_STATE{1};
    std_msgs::Byte msg;
    msg.data = frame.data[1] == MANUAL_CHARGE_STATE ? 2
             : frame.data[1] == AUTO_CHARGE_STATE   ? 1
                                                    : 0;
    pub_charge.publish(msg);
}

void receiver_board::publish_power(const can_frame &frame) const
{
    std_msgs::Byte msg;
    msg.data = (frame.data[0] & 0b00001000) != 0 ? frame.data[2] : 0;
    pub_power.publish(msg);
}

void receiver_board::publish_charge_delay(const can_frame &frame) const
{
    std_msgs::UInt8 msg;
    msg.data = frame.data[3];
    pub_charge_delay.publish(msg);
}

void receiver_board::publish_charge_voltage(const can_frame &frame) const
{
    std_msgs::Float32 msg;
    msg.data = ((frame.data[4] << 8) | frame.data[5]) * 1e-3f;
    pub_charge_voltage.publish(msg);
}

void receiver_board::publish_safety_lidar(const can_frame &frame) const
{
    std_msgs::Bool msg;
    msg.data = (frame.data[0] & 0b00000010) != 0;
    pub_safety_lidar.publish(msg);
}
