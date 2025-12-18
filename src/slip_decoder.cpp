// slip_decoder.cpp
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

#include "slip_decoder.hpp"
#include <iostream>

void slip_decoder::reset()
{
  buffer.clear();
  escape_next = false;
}

bool slip_decoder::decode_byte(uint8_t byte, std::vector<uint8_t>& packet)
{
  if (byte == SLIP_END)
  {
    const bool is_frame_complete = !buffer.empty();
    if (is_frame_complete)
    {
      packet = buffer;
    }

    buffer.clear();
    escape_next = false;
    return is_frame_complete;
  }

  if (!escape_next && (byte == SLIP_ESC))
  {
    escape_next = true;
    return false;
  }

  if (escape_next)
  {
    if (byte == SLIP_ESC_END)
    {
      buffer.push_back(SLIP_END);
    }
    else if (byte == SLIP_ESC_ESC)
    {
      buffer.push_back(SLIP_ESC);
    }
    else
    {
      // Invalid escape sequence - clear buffer to resync
      std::cerr << "SLIP: Invalid escape sequence (0x"
                << std::hex << static_cast<int>(byte) << std::dec
                << "), resync"
                << std::endl;
      buffer.clear();
    }
    escape_next = false;
  }
  else
  {
    buffer.push_back(byte);
  }

  // Overflow protection
  if (MAX_BUFFER_SIZE < buffer.size())
  {
    std::cerr << "SLIP: Buffer overflow (" << buffer.size() << " bytes), resync" << std::endl;
    buffer.clear();
    escape_next = false;
  }

  return false;
}

bool slip_decoder::verify_parity(const std::vector<uint8_t>& data, uint8_t parity)
{
  uint8_t calc = 0;
  for (auto byte : data)
  {
    calc ^= byte;
  }

  return calc == parity;
}
