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

#include <mutex>
#include <optional>
#include <condition_variable>
#include <map>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "scbdriver/InitLinearActuator.h"
#include "scbdriver/LinearActuatorLocation.h"
#include "scbdriver/LinearActuatorControlArray.h"
#include "scbdriver/LinearActuatorServiceResponse.h"

class canif;

class sender_actuator {
public:
    sender_actuator(ros::NodeHandle &n, canif &can);
private:
    class service_response_message_store {
    public:
        using value_type = scbdriver::LinearActuatorServiceResponse;
        using container_type = std::map<uint8_t, value_type>;

        void insert(value_type const& msg)
        {
            std::lock_guard<std::mutex> lock{mtx};
            store[msg.counter] = msg;
        }

        container_type::node_type extract(uint8_t counter)
        {
            std::lock_guard<std::mutex> lock{mtx};
            return store.extract(counter);
        }

    private:
        container_type store;
        std::mutex mtx;
    };

    void handle(const scbdriver::LinearActuatorControlArray::ConstPtr& msg);
    void handle_srv_resp(const scbdriver::LinearActuatorServiceResponse::ConstPtr& msg);
    bool handle_init(
        scbdriver::InitLinearActuator::Request& req,
        scbdriver::InitLinearActuator::Response& res);
    bool handle_location(
        scbdriver::LinearActuatorLocation::Request& req,
        scbdriver::LinearActuatorLocation::Response& res);
    std::optional<scbdriver::LinearActuatorServiceResponse> wait_for_service_response(
        std::unique_lock<std::mutex>& lock,
        uint8_t counter);

    int8_t adjust_direction(size_t index, int8_t direction) const;
    ros::Subscriber sub_actuator;
    ros::Subscriber sub_srv_resp;
    ros::ServiceServer srv_init;
    ros::ServiceServer srv_location;
    canif &can;
    std::mutex actuator_control_mtx;
    std::condition_variable service_resp_cv;
    service_response_message_store  resp_msg_store;
    uint8_t counter{0};
    bool invert_center_actuator_direction;
    bool invert_left_actuator_direction;
    bool invert_right_actuator_direction;

    static constexpr uint32_t queue_size{10};
    static constexpr std::chrono::seconds service_response_timeout{10};
};
