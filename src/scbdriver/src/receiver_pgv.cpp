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
#include <algorithm>
#include "receiver_pgv.hpp"

receiver_pgv::receiver_pgv(ros::NodeHandle &n)
    : pub{n.advertise<scbdriver::PositionGuideVision>("/sensor_set/pgv", 10)}
{
}

void receiver_pgv::handle(const can_frame &frame)
{
    if (fill_buffer(frame)) {
        scbdriver::PositionGuideVision msg;
        decode(msg);
        pub.publish(msg);
    }
}

bool receiver_pgv::fill_buffer(const can_frame &frame)
{
    if (frame.can_dlc != 8)
        return false;
    if (frame.can_id == 0x200) {
        counter[0] = frame.data[7];
        std::copy_n(frame.data, 7, buffer);
    } else if (frame.can_id == 0x201) {
        counter[1] = frame.data[7];
        std::copy_n(frame.data, 7, buffer + 7);
    } else if (frame.can_id == 0x202) {
        counter[2] = frame.data[7];
        std::copy_n(frame.data, 7, buffer + 14);
    }
    return counter[0] == counter[1] && counter[1] == counter[2];
}

void receiver_pgv::decode(scbdriver::PositionGuideVision &msg) const
{
    uint16_t ang_10x{
        static_cast<uint16_t>(
            ((buffer[10] & 0x7f) << 7) |
             (buffer[11] & 0x7f)
        )
    };
    float ang{static_cast<float>(ang_10x) * 0.1f};
    if (ang < 180.0f)
        ang *= -1.0f;
    else
        ang = 360.0f - ang;
    msg.tag_detected = (buffer[1] & 0x40) != 0;
    if (msg.tag_detected) {
        int32_t xps{
            static_cast<int32_t>(
                (buffer[2] & 0x04 ? 0xff000000 : 0) |
                ((buffer[2] & 0x07) << 21) |
                ((buffer[3] & 0x7f) << 14) |
                ((buffer[4] & 0x7f) <<  7) |
                 (buffer[5] & 0x7f)
            )
        };
        msg.x_pos = static_cast<float>(xps) * 1e-4f;
        msg.control_code1_detected = 0;
        msg.control_code2_detected = 0;
    } else {
        uint32_t xp{
            static_cast<uint32_t>(
                ((buffer[2] & 0x07) << 21) |
                ((buffer[3] & 0x7f) << 14) |
                ((buffer[4] & 0x7f) <<  7) |
                 (buffer[5] & 0x7f)
            )
        };
        uint16_t cc1{
            static_cast<uint16_t>(
                ((buffer[14] & 0x07) << 7) |
                 (buffer[15] & 0x7f)
            )
        };
        uint16_t cc2{
            static_cast<uint16_t>(
                ((buffer[16] & 0x07) << 7) |
                 (buffer[17] & 0x7f)
            )
        };
        msg.x_pos = static_cast<float>(xp) * 1e-4f;
        msg.control_code1_detected = cc1;
        msg.control_code2_detected = cc2;
    }
    int32_t yps{
        (buffer[6] & 0x40 ? 0xc000 : 0) |
        ((buffer[6] & 0x7f) << 7) |
        (buffer[7] & 0x7f)
    };
    msg.angle = ang * M_PI / 180.0f;
    msg.y_pos = static_cast<float>(yps) * 1e-4f;
    msg.direction = direction;
    msg.color_lane_count = (buffer[1] & 0x30) >> 4;
    msg.no_color_lane = (buffer[1] & 0x04) != 0;
    msg.no_pos = (buffer[0] & 0x02) != 0;
}
