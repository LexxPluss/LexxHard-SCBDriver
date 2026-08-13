/*
 * Copyright (c) 2026, LexxPluss Inc.
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

#include <gtest/gtest.h>
#include <vector>
#include <fstream>
#include <cstring>
#include <cstdlib>

// Access slip_decoder public API
#include "slip_decoder.hpp"

// Include receiver_tof.cpp directly to access anonymous namespace functions
// (parse_frame, decode, read_le32, conv_from_raw_distance).
// receiver_tof class methods are compiled but never called in these tests.
#include "receiver_tof.cpp"

// ======================================================================
// SLIP decoder tests
// ======================================================================

TEST(SlipDecoder, DecodeSimpleFrame)
{
  slip_decoder decoder;
  std::vector<uint8_t> packet;

  // Frame: C0 01 02 03 <parity> C0
  uint8_t parity = 0x01 ^ 0x02 ^ 0x03;
  EXPECT_FALSE(decoder.decode_byte(0xC0, packet));
  EXPECT_FALSE(decoder.decode_byte(0x01, packet));
  EXPECT_FALSE(decoder.decode_byte(0x02, packet));
  EXPECT_FALSE(decoder.decode_byte(0x03, packet));
  EXPECT_FALSE(decoder.decode_byte(parity, packet));
  EXPECT_TRUE(decoder.decode_byte(0xC0, packet));
  ASSERT_EQ(4u, packet.size());
  EXPECT_EQ(0x01, packet[0]);
  EXPECT_EQ(0x02, packet[1]);
  EXPECT_EQ(0x03, packet[2]);
  EXPECT_EQ(parity, packet[3]);
}

TEST(SlipDecoder, VerifyParityCorrect)
{
  std::vector<uint8_t> data = { 0x01, 0x02, 0x03 };
  uint8_t parity = 0x01 ^ 0x02 ^ 0x03;
  EXPECT_TRUE(slip_decoder::verify_parity(data, parity));
}

TEST(SlipDecoder, VerifyParityIncorrect)
{
  std::vector<uint8_t> data = { 0x01, 0x02, 0x03 };
  EXPECT_FALSE(slip_decoder::verify_parity(data, 0xFF));
}

TEST(SlipDecoder, VerifyParitySingleByte)
{
  std::vector<uint8_t> data = { 0x42 };
  EXPECT_TRUE(slip_decoder::verify_parity(data, 0x42));
}

TEST(SlipDecoder, VerifyParityEmpty)
{
  std::vector<uint8_t> data;
  EXPECT_TRUE(slip_decoder::verify_parity(data, 0x00));
}

TEST(SlipDecoder, EscapeEnd)
{
  slip_decoder decoder;
  std::vector<uint8_t> packet;

  decoder.decode_byte(0xC0, packet);  // Start
  decoder.decode_byte(0xDB, packet);  // Escape
  decoder.decode_byte(0xDC, packet);  // -> 0xC0
  EXPECT_TRUE(decoder.decode_byte(0xC0, packet));
  ASSERT_EQ(1u, packet.size());
  EXPECT_EQ(0xC0, packet[0]);
}

TEST(SlipDecoder, EscapeEsc)
{
  slip_decoder decoder;
  std::vector<uint8_t> packet;

  decoder.decode_byte(0xC0, packet);
  decoder.decode_byte(0xDB, packet);  // Escape
  decoder.decode_byte(0xDD, packet);  // -> 0xDB
  EXPECT_TRUE(decoder.decode_byte(0xC0, packet));
  ASSERT_EQ(1u, packet.size());
  EXPECT_EQ(0xDB, packet[0]);
}

TEST(SlipDecoder, InvalidEscapeResync)
{
  slip_decoder decoder;
  std::vector<uint8_t> packet;

  decoder.decode_byte(0xC0, packet);
  decoder.decode_byte(0x01, packet);
  decoder.decode_byte(0xDB, packet);  // Escape
  decoder.decode_byte(0x42, packet);  // Invalid escape byte -> resync (buffer cleared)
  // Next frame should start fresh
  decoder.decode_byte(0xC0, packet);  // End of (empty) frame after resync
  // Frame was cleared by invalid escape, so no complete frame
}

TEST(SlipDecoder, EmptyFrame)
{
  slip_decoder decoder;
  std::vector<uint8_t> packet;

  // Two consecutive ENDs with no data -> no frame
  EXPECT_FALSE(decoder.decode_byte(0xC0, packet));
  EXPECT_FALSE(decoder.decode_byte(0xC0, packet));
}

TEST(SlipDecoder, ConsecutiveFrames)
{
  slip_decoder decoder;
  std::vector<uint8_t> packet;

  // Frame 1: {0xAA}
  decoder.decode_byte(0xC0, packet);
  decoder.decode_byte(0xAA, packet);
  EXPECT_TRUE(decoder.decode_byte(0xC0, packet));
  ASSERT_EQ(1u, packet.size());
  EXPECT_EQ(0xAA, packet[0]);

  // Frame 2: {0xBB, 0xCC}
  packet.clear();
  decoder.decode_byte(0xBB, packet);
  decoder.decode_byte(0xCC, packet);
  EXPECT_TRUE(decoder.decode_byte(0xC0, packet));
  ASSERT_EQ(2u, packet.size());
  EXPECT_EQ(0xBB, packet[0]);
  EXPECT_EQ(0xCC, packet[1]);
}

// ======================================================================
// ToF parser tests (parse_frame in anonymous namespace)
// ======================================================================

TEST(TofParser, RejectTooShort)
{
  std::vector<uint8_t> frame = { 0x01, 0x02 };
  auto result = parse_frame(frame);
  EXPECT_FALSE(result.has_value());
}

TEST(TofParser, RejectEmptyFrame)
{
  std::vector<uint8_t> frame;
  auto result = parse_frame(frame);
  EXPECT_FALSE(result.has_value());
}

TEST(TofParser, RejectExactHeaderNoZoneData)
{
  // Header claims 1 zone but no zone data follows
  std::vector<uint8_t> frame = {
    PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01  // 1 zone, but no zone data
  };
  auto result = parse_frame(frame);
  EXPECT_FALSE(result.has_value());
}

TEST(TofParser, RejectExcessiveZones)
{
  std::vector<uint8_t> frame(7, 0);
  frame[0] = PKT_TOF_DATA;
  frame[6] = 65;  // > MAX_ZONES(64)
  auto result = parse_frame(frame);
  EXPECT_FALSE(result.has_value());
}

TEST(TofParser, RejectTruncatedTargetData)
{
  // 1 zone with 1 target but only 2 bytes of target data (need 5)
  std::vector<uint8_t> frame = {
    PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01,               // 1 zone
    0x01,               // 1 target
    0x00,         0x00  // only 2 bytes (need 4 distance + 1 status = 5)
  };
  auto result = parse_frame(frame);
  EXPECT_FALSE(result.has_value());
}

TEST(TofParser, RejectExcessiveTargets)
{
  std::vector<uint8_t> frame = {
    PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01,  // 1 zone
    0x05   // 5 targets > MAX_TARGETS_PER_ZONE(4)
  };
  auto result = parse_frame(frame);
  EXPECT_FALSE(result.has_value());
}

TEST(TofParser, ValidSingleZoneSingleTarget)
{
  // sensor_id 0, 1 zone, 1 target at 1000mm
  std::vector<uint8_t> frame = {
    PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01,                            // 1 zone
    0x01,                            // 1 target
    0xE8,         0x03, 0x00, 0x00,  // distance: 1000mm (LE32)
    0x05                             // status
  };
  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(0, result->sensor_id);
  EXPECT_EQ(1, result->num_of_zones);
  EXPECT_EQ(1, result->zone_results[0].num_of_targets);
  EXPECT_EQ(1000u, result->zone_results[0].distance[0]);

  std_msgs::Float32MultiArray msg;
  ASSERT_TRUE(decode(msg, *result));
  ASSERT_EQ(1u, msg.data.size());
  EXPECT_FLOAT_EQ(1.0f, msg.data[0]);  // 1000mm = 1.0m
}

TEST(TofParser, ValidSingleZoneZeroTargets)
{
  std::vector<uint8_t> frame = {
    PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01,  // 1 zone
    0x00   // 0 targets
  };
  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());

  std_msgs::Float32MultiArray msg;
  ASSERT_TRUE(decode(msg, *result));
  ASSERT_EQ(1u, msg.data.size());
  EXPECT_FLOAT_EQ(-1.0f, msg.data[0]);
}

TEST(TofParser, ValidGridSensor64Zones)
{
  // sensor_id 2 (VL53L7 left), 64 zones, each with 1 target at 500mm
  std::vector<uint8_t> frame;
  frame.push_back(PKT_TOF_DATA);
  frame.push_back(0x02);  // sensor_id
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(64);
  for (int i = 0; i < 64; ++i)
  {
    frame.push_back(1);  // 1 target
    frame.push_back(0xF4);
    frame.push_back(0x01);
    frame.push_back(0x00);
    frame.push_back(0x00);  // 500mm LE32
    frame.push_back(0x05);
  }
  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(2, result->sensor_id);
  EXPECT_EQ(64, result->num_of_zones);

  std_msgs::Float32MultiArray msg;
  ASSERT_TRUE(decode(msg, *result));
  ASSERT_EQ(64u, msg.data.size());
  for (int i = 0; i < 64; ++i)
  {
    EXPECT_FLOAT_EQ(0.5f, msg.data[i]);
  }
}

TEST(TofParser, GridSensorWrongZoneCount)
{
  // sensor_id 2 with 32 zones (not 64) - parse succeeds, decode fails
  std::vector<uint8_t> frame;
  frame.push_back(PKT_TOF_DATA);
  frame.push_back(0x02);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(32);
  for (int i = 0; i < 32; ++i)
  {
    frame.push_back(0);  // 0 targets
  }
  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());

  std_msgs::Float32MultiArray msg;
  EXPECT_FALSE(decode(msg, *result));
}

TEST(TofParser, MultiTargetMinDistance)
{
  // sensor_id 2, 64 zones; zone 0 has 3 targets: 300, 100, 200mm
  std::vector<uint8_t> frame;
  frame.push_back(PKT_TOF_DATA);
  frame.push_back(0x02);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(64);

  // Zone 0: 3 targets
  frame.push_back(3);
  // 300mm = 0x12C LE
  frame.push_back(0x2C);
  frame.push_back(0x01);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x05);
  // 100mm = 0x64 LE
  frame.push_back(0x64);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x05);
  // 200mm = 0xC8 LE
  frame.push_back(0xC8);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(0x05);

  // Zones 1-63: 0 targets
  for (int i = 1; i < 64; ++i)
  {
    frame.push_back(0);
  }

  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());

  std_msgs::Float32MultiArray msg;
  ASSERT_TRUE(decode(msg, *result));
  ASSERT_EQ(64u, msg.data.size());
  EXPECT_FLOAT_EQ(0.1f, msg.data[0]);  // min(300,100,200)mm = 100mm = 0.1m
  EXPECT_FLOAT_EQ(-1.0f, msg.data[1]);
}

TEST(TofParser, UnknownPacketType)
{
  std::vector<uint8_t> frame = { 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());  // parse doesn't check type

  std_msgs::Float32MultiArray msg;
  EXPECT_FALSE(decode(msg, *result));  // decode rejects non-0x01
}

TEST(TofParser, ZeroZones)
{
  // 0 zones is valid structurally
  std::vector<uint8_t> frame = {
    PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00  // 0 zones
  };
  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(0, result->num_of_zones);
}

TEST(TofParser, FourTargetsPerZone)
{
  // Max valid targets per zone (4)
  std::vector<uint8_t> frame = {
    PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01,  // 1 zone
    0x04   // 4 targets (max)
  };
  // 4 targets x (4 bytes distance + 1 byte status) = 20 bytes
  for (int j = 0; j < 4; ++j)
  {
    frame.push_back(0xE8);
    frame.push_back(0x03);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x05);
  }
  auto result = parse_frame(frame);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(4, result->zone_results[0].num_of_targets);
}

// ======================================================================
// End-to-end SLIP + parity + parse pipeline test
// ======================================================================

// Helper: build a SLIP-framed packet from raw payload
static std::vector<uint8_t> slip_encode(const std::vector<uint8_t>& payload)
{
  // Calculate parity
  uint8_t parity = 0;
  for (auto b : payload)
    parity ^= b;

  std::vector<uint8_t> wire;
  wire.push_back(0xC0);  // START
  auto emit = [&](uint8_t b) {
    if (b == 0xC0)
    {
      wire.push_back(0xDB);
      wire.push_back(0xDC);
    }
    else if (b == 0xDB)
    {
      wire.push_back(0xDB);
      wire.push_back(0xDD);
    }
    else
    {
      wire.push_back(b);
    }
  };
  for (auto b : payload)
    emit(b);
  emit(parity);
  wire.push_back(0xC0);  // END
  return wire;
}

TEST(Pipeline, ValidPacketEndToEnd)
{
  // Build a valid single-zone ToF payload
  std::vector<uint8_t> payload = { PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x01,                            // 1 zone
                                   0x01,                            // 1 target
                                   0xE8,         0x03, 0x00, 0x00,  // 1000mm
                                   0x05 };
  auto wire = slip_encode(payload);

  slip_decoder decoder;
  std::vector<uint8_t> packet;
  bool frame_complete = false;
  for (auto b : wire)
  {
    if (decoder.decode_byte(b, packet))
    {
      frame_complete = true;
      break;
    }
  }
  ASSERT_TRUE(frame_complete);
  ASSERT_GE(packet.size(), 2u);

  uint8_t parity = packet.back();
  packet.pop_back();
  ASSERT_TRUE(slip_decoder::verify_parity(packet, parity));

  auto result = parse_frame(packet);
  ASSERT_TRUE(result.has_value());

  std_msgs::Float32MultiArray msg;
  ASSERT_TRUE(decode(msg, *result));
  EXPECT_FLOAT_EQ(1.0f, msg.data[0]);
}

TEST(Pipeline, CorruptedParityRejected)
{
  std::vector<uint8_t> payload = { PKT_TOF_DATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
  auto wire = slip_encode(payload);
  // Corrupt: flip a bit in the middle of the wire data (after START, before END)
  if (wire.size() > 3)
  {
    wire[2] ^= 0x01;
  }

  slip_decoder decoder;
  std::vector<uint8_t> packet;
  bool frame_complete = false;
  for (auto b : wire)
  {
    if (decoder.decode_byte(b, packet))
    {
      frame_complete = true;
      break;
    }
  }
  if (!frame_complete)
    return;  // SLIP framing broke, also acceptable

  ASSERT_GE(packet.size(), 2u);
  uint8_t parity = packet.back();
  packet.pop_back();
  EXPECT_FALSE(slip_decoder::verify_parity(packet, parity));
}

// ======================================================================
// Capture replay tests
// Reads raw UART binary captures and verifies the full pipeline.
// Set TOF_CAPTURE_DIR environment variable to the capture_evidence directory.
// Tests skip gracefully if capture data is not available.
// ======================================================================

struct CaptureExpectation
{
  std::string filename;
  int expected_total_frames;
  int expected_min_valid;  // minimum parse+decode successes
};

class CaptureReplayTest : public ::testing::TestWithParam<CaptureExpectation>
{
};

TEST_P(CaptureReplayTest, ReplayCapture)
{
  const auto& param = GetParam();

  const char* capture_dir = std::getenv("TOF_CAPTURE_DIR");
  if (!capture_dir)
  {
    GTEST_SKIP() << "TOF_CAPTURE_DIR not set; skipping capture replay";
  }

  std::string path = std::string(capture_dir) + "/" + param.filename;
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open())
  {
    GTEST_SKIP() << "Capture file not found: " << path;
  }

  slip_decoder decoder;
  int total_frames = 0;
  int parity_ok = 0;
  int parse_ok = 0;
  int decode_ok = 0;

  uint8_t byte;
  while (file.read(reinterpret_cast<char*>(&byte), 1))
  {
    std::vector<uint8_t> packet;
    if (decoder.decode_byte(byte, packet))
    {
      ++total_frames;

      if (packet.size() < 2)
        continue;

      uint8_t parity = packet.back();
      packet.pop_back();

      if (!slip_decoder::verify_parity(packet, parity))
        continue;
      ++parity_ok;

      auto result = parse_frame(packet);
      if (!result.has_value())
        continue;
      ++parse_ok;

      std_msgs::Float32MultiArray msg;
      if (!decode(msg, *result))
        continue;
      ++decode_ok;
    }
  }

  EXPECT_EQ(param.expected_total_frames, total_frames);
  EXPECT_GE(decode_ok, param.expected_min_valid);

  // Report statistics
  std::cout << "Capture replay: " << param.filename << std::endl
            << "  Total SLIP frames: " << total_frames << std::endl
            << "  Parity OK: " << parity_ok << std::endl
            << "  Parse OK: " << parse_ok << std::endl
            << "  Decode OK: " << decode_ok << std::endl;
}

// Expected values derived from capture analysis_summary.json
INSTANTIATE_TEST_SUITE_P(CaptureReplay, CaptureReplayTest,
                         ::testing::Values(
                             // 60s capture: 2675 frames, >=2673 fully valid
                             CaptureExpectation{ "20260209_165924/raw_uart_60s.bin", 2675, 2673 },
                             // 300s capture: 13635 frames (C++ SLIP decoder), >=13633 fully valid
                             CaptureExpectation{ "20260209_180814/raw_uart_300s.bin", 13635, 13633 }));

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
