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
#include "canif.hpp"
#include "sender_actuator.hpp"

sender_actuator::sender_actuator(ros::NodeHandle &n, canif &can)
    : sub_actuator{n.subscribe("/body_control/linear_actuator", queue_size, &sender_actuator::handle, this)},
      sub_encoder{n.subscribe("/body_control/encoder_count", queue_size, &sender_actuator::handle_encoder, this)},
      srv_actuator{n.advertiseService("/body_control/init_linear_actuator", &sender_actuator::handle_init, this)},
      can{can}
{
}

void sender_actuator::handle(const scbdriver::LinearActuatorControlArray::ConstPtr &msg) const
{
    if (in_service)
        return;
    can_frame frame{
        .can_id{0x208},
        .can_dlc{6},
    };
    // ROS:[center,left,right], ROBOT:[left,center,right]
    frame.data[0] = msg->actuators[1].direction;
    frame.data[1] = msg->actuators[0].direction;
    frame.data[2] = msg->actuators[2].direction;
    frame.data[3] = msg->actuators[1].power;
    frame.data[4] = msg->actuators[0].power;
    frame.data[5] = msg->actuators[2].power;
    can.send(frame);
}

void sender_actuator::handle_encoder(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    encoder[0] = msg->data[1];
    encoder[1] = msg->data[0];
    encoder[2] = msg->data[2];
}

bool sender_actuator::handle_init(
    scbdriver::InitLinearActuator::Request& req,
    scbdriver::InitLinearActuator::Response& res)
{
    in_service = true;
    can_frame frame{
        .can_id{0x20b},
        .can_dlc{1},
    };
    frame.data[0] = -1;
    for (int i{0}, count{0}; i < 10; ++i) {
        can.send(frame);
        sleep(1);
        if (encoder[0] == 0 && encoder[1] == 0 && encoder[2] == 0)
            ++count;
        else
            count = 0;
        if (count >= 3)
            break;
    }
    res.success = encoder[0] == 0 && encoder[1] == 0 && encoder[2] == 0;
    in_service = false;
    return true;
}
