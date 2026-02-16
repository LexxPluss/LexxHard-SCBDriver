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
    constexpr uint8_t TOF_PACKET_HEADER_SIZE = 7;
    constexpr uint8_t MAX_ZONES = 64;
    constexpr uint8_t MAX_TARGETS_PER_ZONE = 4;
    // VL53L7CX 8x8 grid constants
    constexpr uint8_t VL53L7_GRID_SIZE = 64;
    constexpr uint8_t VL53L7_SENSOR_ID_LEFT = 2;
    constexpr uint8_t VL53L7_SENSOR_ID_RIGHT = 3;

    struct __attribute__((packed)) ToF_ZoneResult {
        uint8_t num_of_targets;
        uint32_t distance[MAX_TARGETS_PER_ZONE];
        uint8_t status[MAX_TARGETS_PER_ZONE];
    };
    struct __attribute__((packed)) ToF_Packet {
        uint8_t type;
        uint8_t sensor_id;
        uint32_t timestamp_ms;
        uint8_t num_of_zones;
        ToF_ZoneResult zone_results[MAX_ZONES];
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
        if (frame.size() < TOF_PACKET_HEADER_SIZE)
        {
            std::cerr << "ToF packet too short: " << frame.size() << " bytes" << std::endl;
            return std::nullopt;
        }

        ToF_Packet packet{
            .type = frame[0],
            .sensor_id = frame[1],
            .timestamp_ms = read_le32(frame, 2),
            .num_of_zones = frame[6],
        };

        if (packet.num_of_zones > MAX_ZONES)
        {
            std::cerr << "ToF num_of_zones exceeds max: "
                      << static_cast<int>(packet.num_of_zones) << std::endl;
            return std::nullopt;
        }

        size_t offset = TOF_PACKET_HEADER_SIZE;
        for (int i = 0; i < packet.num_of_zones; ++i)
        {
            if (offset >= frame.size())
            {
                std::cerr << "ToF frame truncated at zone " << i << std::endl;
                return std::nullopt;
            }
            packet.zone_results[i].num_of_targets = frame[offset++];

            if (packet.zone_results[i].num_of_targets > MAX_TARGETS_PER_ZONE)
            {
                std::cerr << "ToF num_of_targets exceeds max at zone " << i
                          << ": " << static_cast<int>(packet.zone_results[i].num_of_targets)
                          << std::endl;
                return std::nullopt;
            }

            for (int j = 0; j < packet.zone_results[i].num_of_targets; ++j)
            {
                // Need 4 bytes for distance + 1 byte for status
                if (offset + 5 > frame.size())
                {
                    std::cerr << "ToF frame truncated at zone " << i
                              << " target " << j << std::endl;
                    return std::nullopt;
                }
                packet.zone_results[i].distance[j] = read_le32(frame, offset);
                offset += 4;
                packet.zone_results[i].status[j] = frame[offset++];
            }
        }

        return packet;
    }

    /**
     * Decode ToF packet into Float32MultiArray.
     *
     * VL53L7CX (sensor_id 2/3, 64-zone grid):
     *   Output is always exactly 64 floats in row-major order.
     *   Index convention: data[row * 8 + col], row 0..7, col 0..7.
     *   Per zone: minimum distance (meters) across all targets.
     *   Zones with no targets are encoded as -1.0.
     *
     * VL53L4CX (sensor_id 0/1, single-zone):
     *   Output is variable-length (typically 1 element).
     *   -1.0 for zones with no targets, distance(m) per target otherwise.
     */
    bool decode(std_msgs::Float32MultiArray& msg, ToF_Packet const& packet)
    {
        if (packet.type != PKT_TOF_DATA)
        {
            std::cerr << "Unknown ToF packet type: " << static_cast<int>(packet.type) << std::endl;
            return false;
        }

        msg.data.clear();

        const bool is_grid_sensor = (packet.sensor_id == VL53L7_SENSOR_ID_LEFT ||
                                     packet.sensor_id == VL53L7_SENSOR_ID_RIGHT);

        if (is_grid_sensor)
        {
            if (packet.num_of_zones != VL53L7_GRID_SIZE)
            {
                std::cerr << "Unexpected VL53L7 zone count: "
                          << static_cast<int>(packet.num_of_zones) << std::endl;
                return false;
            }
            msg.data.resize(VL53L7_GRID_SIZE, -1.0f);
            for (uint8_t i = 0; i < packet.num_of_zones && i < VL53L7_GRID_SIZE; ++i)
            {
                if (packet.zone_results[i].num_of_targets == 0)
                {
                    continue; // already -1.0 from resize
                }
                float min_dist = conv_from_raw_distance(packet.zone_results[i].distance[0]);
                for (uint8_t j = 1; j < packet.zone_results[i].num_of_targets; ++j)
                {
                    float dist = conv_from_raw_distance(packet.zone_results[i].distance[j]);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                    }
                }
                msg.data[i] = min_dist;
            }
        }
        else
        {
            for (uint8_t i = 0; i < packet.num_of_zones; ++i)
            {
                if (packet.zone_results[i].num_of_targets == 0)
                {
                    msg.data.push_back(-1.0f);
                    continue;
                }
                for (uint8_t j = 0; j < packet.zone_results[i].num_of_targets; ++j)
                {
                    msg.data.push_back(conv_from_raw_distance(packet.zone_results[i].distance[j]));
                }
            }
        }

        return true;
    }
}  // namespace

receiver_tof::receiver_tof(ros::NodeHandle& n)
{
  pub_tof_front = n.advertise<std_msgs::Float32MultiArray>("/sensor_set/tof_front", queue_size);
  pub_tof_rear = n.advertise<std_msgs::Float32MultiArray>("/sensor_set/tof_rear", queue_size);
  // NOTE: Design doc specifies /tof_raw_left and /tof_raw_right, but we intentionally keep
  // the existing topic names to avoid breaking the established SCBDriver interface.
  // The downstream tof_hanging_detector_node uses parameterized subscription to these topics.
  pub_low_object_left = n.advertise<std_msgs::Float32MultiArray>("/sensor_set/low_object_left", queue_size);
  pub_low_object_right = n.advertise<std_msgs::Float32MultiArray>("/sensor_set/low_object_right", queue_size);
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
    pub_tof_front.publish(msg);
  }
  else if(packet->sensor_id == 1)
  {
    pub_tof_rear.publish(msg);
  }
  else if(packet->sensor_id == 2)
  {
    pub_low_object_left.publish(msg);
  }
  else if(packet->sensor_id == 3)
  {
    pub_low_object_right.publish(msg);
  }
  else
  {
    std::cerr << "Invalid sensor ID in ToF packet: "
              << static_cast<int>(packet->sensor_id) << std::endl;
  }
}
