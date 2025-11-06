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

#pragma once
#include <functional>
#include <string>
#include <vector>
#include <optional>
#include <cstdint>

class slip_decoder
{
public:
  slip_decoder();
  bool decode_byte(uint8_t byte, std::vector<uint8_t>& packet);
  void reset();
  static bool verify_parity(const std::vector<uint8_t>& data, uint8_t parity);

private:
  std::vector<uint8_t> buffer;
  bool escape_next{false};
  
  static constexpr uint8_t SLIP_END = 0xC0;
  static constexpr uint8_t SLIP_ESC = 0xDB;
  static constexpr uint8_t SLIP_ESC_END = 0xDC;
  static constexpr uint8_t SLIP_ESC_ESC = 0xDD;
  static constexpr size_t MAX_BUFFER_SIZE = 50;
};

class uartif
{
public:
  uartif(const std::string& device, uint32_t baudrate);
  ~uartif();

  void set_handler(std::function<void(const std::vector<uint8_t>& packet)> handler);
  int init();
  void term();
  int poll(int timeout_ms) const;

private:
  const std::string device;
  const uint32_t baudrate;
  int fd{-1};
  std::function<void(const std::vector<uint8_t>& packet)> handler;

  mutable slip_decoder decoder;
};