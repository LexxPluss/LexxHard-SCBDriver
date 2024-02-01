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
#include "receiver_imu.hpp"

receiver_imu::receiver_imu(ros::NodeHandle &n)
    : pub{n.advertise<scbdriver::Imu>("/sensor_set/imu", 10)}
{
}

void receiver_imu::handle(const can_frame &frame)
{
    if (fill_buffer(frame)) {
        scbdriver::Imu msg;
        decode(msg);
        pub.publish(msg);
    }
    scbdriver::Imu msg;
}

bool receiver_imu::fill_buffer(const can_frame &frame)
{
    if (frame.can_dlc != 7)
        return false;
    if (frame.can_id == 0x206) {
        counter[0] = frame.data[6];
        accel[0] = (frame.data[0] << 8) | frame.data[1];
        accel[1] = (frame.data[2] << 8) | frame.data[3];
        accel[2] = (frame.data[4] << 8) | frame.data[5];
    } else if (frame.can_id == 0x207) {
        counter[1] = frame.data[6];
        gyro[0] = (frame.data[0] << 8) | frame.data[1];
        gyro[1] = (frame.data[2] << 8) | frame.data[3];
        gyro[2] = (frame.data[4] << 8) | frame.data[5];
    }
    return counter[0] == counter[1];
}

void receiver_imu::decode(scbdriver::Imu &msg) const
{
    msg.accel.x = accel[0] * 1e-3f;
    msg.accel.y = accel[1] * 1e-3f;
    msg.accel.z = accel[2] * 1e-3f;
    msg.gyro.x = gyro[0] * 1e-3f;
    msg.gyro.y = gyro[1] * 1e-3f;
    msg.gyro.z = gyro[2] * 1e-3f;
    msg.ang.x =
    msg.ang.y =
    msg.ang.z =
    msg.vel.x =
    msg.vel.y =
    msg.vel.z = 0.0;
}
