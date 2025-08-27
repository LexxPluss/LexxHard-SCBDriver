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

#include <mutex>
#include <optional>
#include <condition_variable>
#include <map>
#include <chrono>

#include <linux/can.h>

#include "canif.hpp"
#include "scbdriver/LinearActuatorServiceResponse.h"
#include "sender_actuator.hpp"

sender_actuator::sender_actuator(ros::NodeHandle& n, ros::NodeHandle& pn, canif& can)
  : sub_actuator{ n.subscribe("/body_control/linear_actuator", queue_size, &sender_actuator::handle, this) }
  , sub_srv_resp{ n.subscribe("scbdriver/linear_actuator_service_response", queue_size,
                              &sender_actuator::handle_srv_resp, this) }
  , srv_init{ n.advertiseService("/body_control/init_linear_actuator", &sender_actuator::handle_init, this) }
  , srv_location{ n.advertiseService("/body_control/linear_actuator_location", &sender_actuator::handle_location,
                                     this) }
  , can{ can }
{
  pn.param<bool>("invert_center_actuator_direction", invert_center_actuator_direction, false);
  pn.param<bool>("invert_left_actuator_direction", invert_left_actuator_direction, false);
  pn.param<bool>("invert_right_actuator_direction", invert_right_actuator_direction, false);
}

int8_t sender_actuator::adjust_direction(size_t index, int8_t direction) const
{
  // clang-format off
  bool const should_invert_tbl[] = {
      invert_center_actuator_direction,
      invert_left_actuator_direction,
      invert_right_actuator_direction
  };
  // clang-format on
  bool const should_invert{ should_invert_tbl[index] };

  if (should_invert)
  {
    return -direction;
  }
  return direction;
}

void sender_actuator::handle(const scbdriver::LinearActuatorControlArray::ConstPtr& msg)
{
  if (msg->actuators.size() != 3)
  {
    ROS_WARN("Drop message with invalid size: %lu", msg->actuators.size());
    return;
  }

  std::unique_lock<std::mutex> handle_lock{ handle_mtx, std::try_to_lock };
  if (!handle_lock.owns_lock())
  {
    return;
  }

  can_frame frame{
    .can_id{ 0x208 },
    .can_dlc{ 6 },
  };
  frame.data[0] = adjust_direction(0, msg->actuators[0].direction);
  frame.data[1] = adjust_direction(1, msg->actuators[1].direction);
  frame.data[2] = adjust_direction(2, msg->actuators[2].direction);
  frame.data[3] = msg->actuators[0].power;
  frame.data[4] = msg->actuators[1].power;
  frame.data[5] = msg->actuators[2].power;
  can.send(frame);
}

void sender_actuator::handle_srv_resp(const scbdriver::LinearActuatorServiceResponse::ConstPtr& msg)
{
  resp_msg_store.insert(*msg);
  service_resp_cv.notify_all();
}

bool sender_actuator::handle_init(scbdriver::InitLinearActuator::Request& req,
                                  scbdriver::InitLinearActuator::Response& res)
{
  std::unique_lock<std::mutex> handle_lock{ handle_mtx, std::try_to_lock };
  if (!handle_lock.owns_lock())
  {
    return false;
  }

  if (req.directions.data.size() != 3)
  {
    ROS_WARN("Reject request with invalid directions size: %lu", req.directions.data.size());
    return false;
  }

  auto const request_id = counter++;
  // send request
  {
    can_frame frame{
      .can_id{ 0x20b },
      .can_dlc{ 8 },
    };
    frame.data[0] = 1;  // 1 means init
    frame.data[1] = adjust_direction(0, req.directions.data[0]);
    frame.data[2] = adjust_direction(1, req.directions.data[1]);
    frame.data[3] = adjust_direction(2, req.directions.data[2]);
    frame.data[7] = request_id;

    can.send(frame);
  }

  // wait for response
  {
    std::unique_lock<std::mutex> notify_lock{ notify_mtx };
    auto const resp{ wait_for_service_response(notify_lock, request_id) };
    if (!resp.has_value())
    {
      return false;
    }

    res.success = resp->success;
  }
  return true;
}

bool sender_actuator::handle_location(scbdriver::LinearActuatorLocation::Request& req,
                                      scbdriver::LinearActuatorLocation::Response& res)
{
  if (req.location.data.size() != 3)
  {
    ROS_WARN("Reject request with invalid location size: %lu", req.location.data.size());
    return false;
  }
  if (req.power.data.size() != 3)
  {
    ROS_WARN("Reject request with invalid power size: %lu", req.power.data.size());
    return false;
  }

  std::unique_lock<std::mutex> handle_lock{ handle_mtx, std::try_to_lock };
  if (!handle_lock.owns_lock())
  {
    return false;
  }

  auto const request_id = counter++;
  // send request
  {
    can_frame frame{
      .can_id{ 0x20b },
      .can_dlc{ 8 },
    };
    frame.data[0] = 0;  // 0 means location
    frame.data[1] = adjust_direction(0, req.location.data[0]);
    frame.data[2] = adjust_direction(1, req.location.data[1]);
    frame.data[3] = adjust_direction(2, req.location.data[2]);
    frame.data[4] = req.power.data[0];
    frame.data[5] = req.power.data[1];
    frame.data[6] = req.power.data[2];
    frame.data[7] = request_id;

    can.send(frame);
  }

  // wait for response
  {
    std::unique_lock<std::mutex> notify_lock{ notify_mtx };
    auto const resp{ wait_for_service_response(notify_lock, request_id) };
    if (!resp.has_value())
    {
      return false;
    }

    res.success = resp->success;
    res.detail.data.resize(3);
    res.detail.data[0] = resp->detail[0];
    res.detail.data[1] = resp->detail[1];
    res.detail.data[2] = resp->detail[2];
  }

  return true;
}

std::optional<scbdriver::LinearActuatorServiceResponse>
sender_actuator::wait_for_service_response(std::unique_lock<std::mutex>& lock, uint8_t counter)
{
  service_response_message_store::container_type::node_type node;
  auto const is_done{ service_resp_cv.wait_for(lock, service_response_timeout, [this, &counter, &node] {
    node = resp_msg_store.extract(counter);
    return !node.empty();
  }) };
  if (!is_done)
  {
    ROS_WARN("Timeout waiting for service response");
    return std::nullopt;
  }

  // node own data because service was done
  return node.mapped();
}
