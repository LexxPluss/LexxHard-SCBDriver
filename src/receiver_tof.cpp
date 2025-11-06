/*
 * Copyright (c) 2025, LexxPluss Inc.
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

#include <iostream>
#include <cstring>
#include <optional>

#include "std_msgs/Float32MultiArray.h"

#include "receiver_tof.hpp"

namespace
{
    constexpr uint8_t PKT_TOF_DATA = 0x01;

    struct __attribute__((packed))  ToF_Packet {
        uint8_t type;
        uint8_t sensor_id;
        uint32_t timestamp_ms;
        uint8_t num_of_targets;
        uint32_t distances[4];
    };

    uint32_t read_le32(const std::vector<uint8_t>& packet, size_t offset)
    {
      return packet[offset] | (packet[offset+1] << 8) | (packet[offset+2] << 16) | (packet[offset+3] << 24);
    }

    float conv_from_raw_distance(uint32_t raw_distance)
    {
        return static_cast<float>(raw_distance) * 0.001f; // Convert mm to meters
    }

    std::optional<ToF_Packet> parse_frame(const std::vector<uint8_t>& frame)
    {
        if (frame.size() < sizeof(ToF_Packet))
        {
            std::cerr << "ToF packet too short: " << frame.size() << " bytes" << std::endl;
            return std::nullopt;
        }

        ToF_Packet const packet{
            .type = frame[0],
            .sensor_id = frame[1],
            .timestamp_ms = read_le32(frame, 2),
            .num_of_targets = frame[6],
            .distances = {
                read_le32(frame, 7),
                read_le32(frame, 11),
                read_le32(frame, 15),
                read_le32(frame, 19)
            }
        };

        if (4 < packet.num_of_targets)
        {
            std::cerr << "Invalid number of targets in ToF packet: "
                      << static_cast<int>(packet.num_of_targets) << std::endl;
            return std::nullopt;
        }

        return packet;
    }

    bool decode(std_msgs::Float32MultiArray& msg, ToF_Packet const& packet)
    {
        if (packet.type != PKT_TOF_DATA)
        {
            std::cerr << "Unknown ToF packet type: " << static_cast<int>(packet.type) << std::endl;
            return false;
        }

        msg.data.clear();
        for (uint8_t i = 0; i < packet.num_of_targets; ++i)
        {
            msg.data.push_back(conv_from_raw_distance(packet.distances[i]));
        }

        return true;
    }
}  // namespace

receiver_tof::receiver_tof(ros::NodeHandle& n)
{
  pub_front = n.advertise<std_msgs::Float32MultiArray>("/sensor_set/tof_front", queue_size);
  pub_rear = n.advertise<std_msgs::Float32MultiArray>("/sensor_set/tof_rear", queue_size);
}

void receiver_tof::handle(const std::vector<uint8_t>& frame)
{
  const std::optional<ToF_Packet> packet = parse_frame(frame);
  if(!packet)
  {
    return;
  }

  std_msgs::Float32MultiArray msg;
  if(!decode(msg, *packet))
  {
    return;
  }

  if(packet->sensor_id == 0)
  {
    pub_front.publish(msg);
  }
  else if(packet->sensor_id == 1)
  {
    pub_rear.publish(msg);
  }
  else
  {
    std::cerr << "Invalid sensor ID in ToF packet: "
              << static_cast<int>(packet->sensor_id) << std::endl;
  }
}
