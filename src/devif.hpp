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

#include <linux/can.h>
#include <cstdint>
#include <string>
#include <vector>
#include <variant>
#include "slip_decoder.hpp"
#include "spsc_queue.hpp"

struct can_message
{
  can_frame frame;
};

struct uart_message
{
  std::vector<uint8_t> packet;
};

using device_message = std::variant<can_message, uart_message>;

class devif
{
public:
  static constexpr size_t QUEUE_CAPACITY = 1024;
  using queue_type = spsc_queue<device_message, QUEUE_CAPACITY>;

  devif() = default;
  explicit devif(queue_type& queue);
  ~devif();

  devif(const devif&) = delete;
  devif& operator=(const devif&) = delete;

  int add_can(const std::string& ifname, const can_filter* filter, size_t nfilter);
  int add_uart(const std::string& device, uint32_t baudrate);

  int poll(int timeout_ms);
  void term();

  int send_can(const can_frame& frame) const;

private:
  struct can_device
  {
    int fd{-1};
  };

  struct uart_device
  {
    int fd{-1};
    slip_decoder decoder;
  };

  int read_can(can_device& dev);
  int read_uart(uart_device& dev);

  queue_type* queue{nullptr};
  std::vector<can_device> can_devices;
  std::vector<uart_device> uart_devices;
};
