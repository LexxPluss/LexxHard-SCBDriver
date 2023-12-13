/*
 * Copyright (c) 2023, LexxPluss Inc.
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
#include "sender_led.hpp"

sender_led::sender_led(ros::NodeHandle &n, canif &can)
    : sub{n.subscribe("/body_control/led", 10, &sender_led::handle, this)},
      can{can}
{
}

void sender_led::handle(const std_msgs::String::ConstPtr& msg) const
{
    uint16_t count_per_minutes{0};
    uint8_t pattern{0}, rgb[3]{0, 0, 0};
    decode(msg->data, pattern, count_per_minutes, rgb);
    can_frame frame{
        .can_id{0x205},
        .can_dlc{6},
    };
    frame.data[0] = pattern;
    frame.data[1] = count_per_minutes >> 8;
    frame.data[2] = count_per_minutes;
    frame.data[3] = rgb[0];
    frame.data[4] = rgb[1];
    frame.data[5] = rgb[2];
    can.send(frame);
}

void sender_led::decode(const std::string &data,
                        uint8_t &pattern,
                        uint16_t &count_per_minutes,
                        uint8_t rgb[3]) const
{
    if      (data == "emergency_stop")  pattern = 1;
    else if (data == "amr_mode")        pattern = 2;
    else if (data == "agv_mode")        pattern = 3;
    else if (data == "mission_pause")   pattern = 4;
    else if (data == "path_blocked")    pattern = 5;
    else if (data == "manual_drive")    pattern = 6;
    else if (data == "charging")        pattern = 10;
    else if (data == "waiting_for_job") pattern = 11;
    else if (data == "left_winker")     pattern = 12;
    else if (data == "right_winker")    pattern = 13;
    else if (data == "both_winker")     pattern = 14;
    else if (data == "move_actuator")   pattern = 15;
    else if (data == "charge_level")    pattern = 16;
    else if (data == "showtime")        pattern = 100;
    else if (data == "lockdown")        pattern = 101;
    else if (data[0] == '#')            decode_rgb(data, pattern, count_per_minutes, rgb);
    else std::cerr << "sender_led::handle(): unknown pattern: " << data << std::endl;
}

void sender_led::decode_rgb(const std::string &data,
                            uint8_t &pattern,
                            uint16_t &count_per_minutes,
                            uint8_t rgb[3]) const
{
    std::istringstream is{data.substr(1)};
    uint32_t rgb24{0};
    std::string type;
    is >> std::hex >> rgb24
                   >> type
       >> std::dec >> count_per_minutes;
    rgb[0] = rgb24 >> 16;
    rgb[1] = rgb24 >>  8;
    rgb[2] = rgb24;
    if (count_per_minutes == 0) {
        pattern = 200;
    } else {
        if (type == "blink") {
            pattern = 201;
        } else if (type == "breath") {
            pattern = 202;
        } else {
            pattern = 200;
            count_per_minutes = 0;
        }
    }
}
