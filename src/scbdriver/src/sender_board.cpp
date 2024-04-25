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
#include "sender_board.hpp"

sender_board::sender_board(ros::NodeHandle &n, canif &can)
    : sub_ems{n.subscribe("/control/request_emergency_stop", queue_size, &sender_board::handle_ems, this)},
      sub_power_off{n.subscribe("/control/request_power_off", queue_size, &sender_board::handle_power_off, this)},
      sub_wheel_off{n.subscribe("/lexxhard/setup", queue_size, &sender_board::handle_wheel_off, this)},
      sub_heartbeat{n.subscribe("/lexxhard/mainboard_messenger_heartbeat", queue_size, &sender_board::handle_heartbeat, this)},
      can{can}
{
}

void sender_board::handle_ems(const std_msgs::Bool::ConstPtr &msg) const
{
    can_frame frame {
        .can_id{0x20F},
        .can_dlc{4},
        .data{0}
    };

    frame.data[0] = msg->data;
    can.send(frame);
}

void sender_board::handle_power_off(const std_msgs::Bool::ConstPtr &msg) const
{
    can_frame frame {
        .can_id{0x20F},
        .can_dlc{4},
        .data{0}
    };
    frame.data[1] = msg->data;
    can.send(frame);
}

void sender_board::handle_wheel_off(const std_msgs::String::ConstPtr &msg) const
{
    can_frame frame {
        .can_id{0x20F},
        .can_dlc{4},
        .data{0}
    };

    if (msg->data == "wheel_poweroff") {
        frame.data[2] = 1;
        can.send(frame);
    }
}

void sender_board::handle_heartbeat(const std_msgs::Bool::ConstPtr &msg) const
{
    can_frame frame {
        .can_id{0x20F},
        .can_dlc{4},
        .data{0}
    };
    frame.data[3] = msg->data;
    can.send(frame);
}