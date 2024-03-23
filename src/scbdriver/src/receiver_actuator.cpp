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
#include "receiver_actuator.hpp"

receiver_actuator::receiver_actuator(ros::NodeHandle &n)
    : pub_encoder{n.advertise<std_msgs::Int32MultiArray>("/body_control/encoder_count", queue_size)},
      pub_current{n.advertise<std_msgs::Float32MultiArray>("/body_control/linear_actuator_current", queue_size)},
      pub_connection{n.advertise<std_msgs::Float32MultiArray>("/body_control/shelf_connection", queue_size)}
{
}

void receiver_actuator::handle(const can_frame &frame) const
{
    if (frame.can_id == 0x209) {
        if (frame.can_dlc != 6)
            return;
        // ROS:[center,left,right], ROBOT:[left,center,right]
        int16_t
            L{static_cast<int16_t>((frame.data[0] << 8) | frame.data[1])},
            C{static_cast<int16_t>((frame.data[2] << 8) | frame.data[3])},
            R{static_cast<int16_t>((frame.data[4] << 8) | frame.data[5])};
        std_msgs::Int32MultiArray msg;
        msg.data.resize(3);
        msg.data[0] = C;
        msg.data[1] = L;
        msg.data[2] = R;
        pub_encoder.publish(msg);
    } else if (frame.can_id == 0x20a) {
        if (frame.can_dlc != 8)
            return;
        // ROS:[center,left,right], ROBOT:[left,center,right]
        int16_t
            L{static_cast<int16_t>((frame.data[0] << 8) | frame.data[1])},
            C{static_cast<int16_t>((frame.data[2] << 8) | frame.data[3])},
            R{static_cast<int16_t>((frame.data[4] << 8) | frame.data[5])};
        std_msgs::Float32MultiArray msg_current;
        msg_current.data.resize(3);
        msg_current.data[0] = C * 1e-3f;
        msg_current.data[1] = L * 1e-3f;
        msg_current.data[2] = R * 1e-3f;
        pub_current.publish(msg_current);
        int16_t con{static_cast<int16_t>((frame.data[6] << 8) | frame.data[7])};
        std_msgs::Float32MultiArray msg_connection;
        msg_connection.data.resize(1);
        msg_connection.data[0] = con * 1e-3f;
        pub_connection.publish(msg_connection);
    }
}
