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

struct can_frame;

class receiver_board {
public:
    receiver_board(ros::NodeHandle &n);
    void handle(const can_frame &frame) const;
private:
    void publish_bumper(const can_frame &frame) const;
    void publish_emergency_switch(const can_frame &frame) const;
    void publish_emergency_stop(const can_frame &frame) const;
    void publish_charge(const can_frame &frame) const;
    void publish_power(const can_frame &frame) const;
    void publish_charge_delay(const can_frame &frame) const;
    void publish_charge_voltage(const can_frame &frame) const;
    void publish_safety_lidar(const can_frame &frame) const;
    ros::Publisher
        pub_bumper, pub_emergency_switch, pub_emergency_stop,
        pub_charge, pub_power, pub_charge_delay, pub_charge_voltage,
        pub_safety_lidar;
    static constexpr uint32_t queue_size{10};
};
