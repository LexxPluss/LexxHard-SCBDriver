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

#include <cinttypes>
#include <cstring>
#include <iostream>
#include <optional>

#include "std_msgs/Float32MultiArray.h"

#include <linux/can.h>

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

struct __attribute__((packed)) ToF_ZoneResult
{
  uint8_t num_of_targets;
  uint32_t distance[MAX_TARGETS_PER_ZONE];
  uint8_t status[MAX_TARGETS_PER_ZONE];
};
struct __attribute__((packed)) ToF_Packet
{
  uint8_t type;
  uint8_t sensor_id;
  uint32_t timestamp_ms;
  uint8_t num_of_zones;
  ToF_ZoneResult zone_results[MAX_ZONES];
};

uint32_t read_le32(const std::vector<uint8_t>& packet, size_t offset)
{
  return packet[offset] | (packet[offset + 1] << 8) | (packet[offset + 2] << 16) | (packet[offset + 3] << 24);
}

float conv_from_raw_distance(uint32_t raw_distance)
{
  return static_cast<float>(raw_distance) * 0.001f;  // Convert mm to meters
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
    std::cerr << "ToF num_of_zones exceeds max: " << static_cast<int>(packet.num_of_zones) << std::endl;
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
      std::cerr << "ToF num_of_targets exceeds max at zone " << i << ": "
                << static_cast<int>(packet.zone_results[i].num_of_targets) << std::endl;
      return std::nullopt;
    }

    for (int j = 0; j < packet.zone_results[i].num_of_targets; ++j)
    {
      // Need 4 bytes for distance + 1 byte for status
      if (offset + 5 > frame.size())
      {
        std::cerr << "ToF frame truncated at zone " << i << " target " << j << std::endl;
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

  const bool is_grid_sensor = (packet.sensor_id == VL53L7_SENSOR_ID_LEFT || packet.sensor_id == VL53L7_SENSOR_ID_RIGHT);

  if (is_grid_sensor)
  {
    if (packet.num_of_zones != VL53L7_GRID_SIZE)
    {
      std::cerr << "Unexpected VL53L7 zone count: " << static_cast<int>(packet.num_of_zones) << std::endl;
      return false;
    }
    msg.data.resize(VL53L7_GRID_SIZE, -1.0f);
    for (uint8_t i = 0; i < packet.num_of_zones && i < VL53L7_GRID_SIZE; ++i)
    {
      if (packet.zone_results[i].num_of_targets == 0)
      {
        continue;  // already -1.0 from resize
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
  if (!packet)
  {
    return;
  }

  std_msgs::Float32MultiArray msg;
  if (!decode(msg, *packet))
  {
    return;
  }

  if (packet->sensor_id == 0)
  {
    pub_tof_front.publish(msg);
  }
  else if (packet->sensor_id == 1)
  {
    pub_tof_rear.publish(msg);
  }
  else if (packet->sensor_id == 2)
  {
    pub_low_object_left.publish(msg);
  }
  else if (packet->sensor_id == 3)
  {
    pub_low_object_right.publish(msg);
  }
  else
  {
    std::cerr << "Invalid sensor ID in ToF packet: " << static_cast<int>(packet->sensor_id) << std::endl;
  }
}

namespace
{

const char* event_text(lexxhard::tof_grid_assembler::event e)
{
  using ev = lexxhard::tof_grid_assembler::event;
  switch (e)
  {
    case ev::INCOMPLETE_BY_TIMEOUT:
      return "grid incomplete at the 300 ms timeout";
    case ev::INCOMPLETE_BY_GENERATION_CHANGE:
      return "grid abandoned, next generation started";
    case ev::DUPLICATE_CHUNK_IDENTICAL:
      return "duplicate chunk (identical, ignored)";
    case ev::CONFLICTING_CHUNK:
      return "conflicting chunk, generation retired";
    case ev::DUPLICATE_HEALTH_IDENTICAL:
      return "duplicate health frame (identical, ignored)";
    case ev::CONFLICTING_HEALTH:
      return "conflicting health frame, generation retired";
    case ev::MALFORMED_HEADER:
      return "malformed frame header";
    case ev::HEALTH_COUNT_MISMATCH:
      return "health valid_zone_count disagrees with the grid";
    case ev::ORPHAN_HEALTH_TIMEOUT:
      return "health frame arrived with no data";
    case ev::FRAME_FOR_RETIRED_GENERATION:
      return "frame for an already retired generation";
    case ev::SOURCE_NEVER_SEEN:
      return "no ToF frame has EVER arrived from this source";
    case ev::SOURCE_STALE:
      return "ToF source has stopped producing usable grids";
    case ev::SOURCE_RECOVERED:
      return "ToF source recovered";
    default:
      return nullptr;
  }
}

}  // namespace

void receiver_tof::configure_can(uint32_t data_can_id, uint32_t health_can_id, uint64_t now_ms)
{
  assembler = std::make_unique<lexxhard::tof_grid_assembler>(data_can_id, health_can_id, now_ms);
}

void receiver_tof::handle_can(const can_frame& frame, uint64_t now_ms)
{
  if (!assembler)
    return;
  if (auto g = assembler->consume(frame.can_id, frame.can_dlc, frame.data, now_ms))
    publish_grid(*g);
  drain_diagnostics(now_ms);
}

void receiver_tof::poll(uint64_t now_ms)
{
  if (!assembler)
    return;
  assembler->poll(now_ms);
  drain_diagnostics(now_ms);
  report_persistent_state(now_ms);
}

void receiver_tof::publish_grid(const lexxhard::tof_grid_assembler::grid& g)
{
  using asm_t = lexxhard::tof_grid_assembler;

  std_msgs::Float32MultiArray msg;
  msg.data.reserve(asm_t::ZONES);
  for (uint16_t mm : g.zones_mm)
  {
    // The published contract is unchanged from the UART path: exactly 64 floats,
    // row-major, metres, -1.0 where there is no valid target.
    msg.data.push_back(mm == asm_t::INVALID_MM ? -1.0f : conv_from_raw_distance(mm));
  }

  // Mapped through the source table, never by arithmetic on the id. source 0 is the
  // RIGHT sensor, which reverses the legacy sensor_id convention.
  if (g.source == asm_t::SOURCE_FRONT_RIGHT)
    pub_low_object_right.publish(msg);
  else if (g.source == asm_t::SOURCE_FRONT_LEFT)
    pub_low_object_left.publish(msg);
}

void receiver_tof::drain_diagnostics(uint64_t now_ms)
{
  using asm_t = lexxhard::tof_grid_assembler;
  using ev = asm_t::event;

  report_health_reports(now_ms);

  for (const auto& d : assembler->drain_events())
  {
    const char* text = event_text(d.kind);
    if (text == nullptr)
      continue;
    const char* who = asm_t::source_name(d.source);

    // The watchdog events name the source and the state. "a ToF source stopped" tells an
    // operator neither which sensor nor whether frames are still arriving, and those two
    // point at completely different faults, which is the whole reason the state is
    // four-valued rather than a boolean.
    if (d.kind == ev::SOURCE_NEVER_SEEN || d.kind == ev::SOURCE_STALE)
      ROS_ERROR("ToF %s: %s (%s)", who, asm_t::state_name(d.state), text);
    else if (d.kind == ev::SOURCE_RECOVERED)
      ROS_INFO("ToF %s: recovered", who);
    else if (report_throttle_.should_report(asm_t::report_slot(d.kind, d.source), now_ms))
      ROS_WARN("ToF %s: %s", who, text);
  }
}

// The other half of "diagnostic and do not gate". The gate is untouched: the assembler queues these
// only after a grid has been accepted, and this formats what it queued so the condition is
// observable instead of being parsed and discarded.
//
// Every line carries the chain context, because "chain length differs from the configured
// expectation" without the number of boards the sensor saw, or where it sits, sends an operator
// looking with no starting point.
void receiver_tof::report_health_reports(uint64_t now_ms)
{
  using asm_t = lexxhard::tof_grid_assembler;
  using note = asm_t::health_note;

  for (const auto& r : assembler->drain_health_reports())
  {
    for (uint8_t i = 0; i < static_cast<uint8_t>(note::HEALTH_NOTE_COUNT); ++i)
    {
      const note n = static_cast<note>(i);
      if ((r.notes & static_cast<uint8_t>(1u << i)) == 0)
        continue;
      if (!report_throttle_.should_report(asm_t::report_slot(n, r.source), now_ms))
        continue;
      ROS_WARN("ToF %s: %s (chain position %u of %u boards detected, last error 0x%02x)", asm_t::source_name(r.source),
               asm_t::health_note_text(n), r.chain_position, r.boards_detected, r.last_error);
    }
  }
}

void receiver_tof::report_persistent_state(uint64_t now_ms)
{
  using asm_t = lexxhard::tof_grid_assembler;

  // An edge-triggered message in rosout is a record that something happened once, not a
  // statement of what is true now. Anyone attaching after the transition, or scrolling
  // past it, sees a healthy-looking log for a sensor that is still down. Until this
  // publishes a real diagnostic_msgs/DiagnosticArray, re-stating the current fault on a
  // throttle is the minimum that makes the condition observable rather than historical.
  for (uint8_t src = 0; src < asm_t::SOURCE_COUNT; ++src)
  {
    const auto st = assembler->source_status(src, now_ms);
    // Gate on the raised alarm, not on the state. Within the startup grace a source is
    // NEVER_SEEN and that is normal; reporting it would put two sensor faults in the log
    // about 100 ms into every boot, and an error that always appears is one nobody reads.
    if (!st.alarm_active)
    {
      state_throttle.clear(src);
      continue;
    }
    if (!state_throttle.should_report(src, now_ms))
      continue;
    ROS_ERROR("ToF %s: still %s (%" PRIu64 " ms since last frame, %" PRIu64 " ms since last grid)",
              asm_t::source_name(src), asm_t::state_name(st.state), st.since_last_frame_ms,
              st.ever_published ? st.since_last_publish_ms : UINT64_C(0));
  }
}
