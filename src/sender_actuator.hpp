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
#include "std_msgs/Int32MultiArray.h"
#include "scbdriver/InitLinearActuator.h"
#include "scbdriver/LinearActuatorControlArray.h"

class canif;

class sender_actuator {
public:
    sender_actuator(ros::NodeHandle &n, canif &can);
private:
    void handle(const scbdriver::LinearActuatorControlArray::ConstPtr& msg) const;
    void handle_encoder(const std_msgs::Int32MultiArray::ConstPtr& msg);
    bool handle_init(
        scbdriver::InitLinearActuator::Request& req,
        scbdriver::InitLinearActuator::Response& res);
    ros::Subscriber sub_actuator, sub_encoder;
    ros::ServiceServer srv_actuator;
    canif &can;
    int32_t encoder[3]{0, 0, 0};
    bool in_service{false};
    static constexpr uint32_t queue_size{10};
};
