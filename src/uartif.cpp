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

#include "uartif.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <cstring>

namespace
{
struct packet_with_parity
{
  std::vector<uint8_t> payload;
  uint8_t parity;
};

int configure_tty(int fd, int baudrate)
{
  // Get current tty settings
  termios tty;
  if (tcgetattr(fd, &tty) != 0)
  {
    std::cerr << "tcgetattr() failed" << std::endl;
    return -1;
  }

  // Set baudrate
  speed_t speed = B115200;
  switch (baudrate)
  {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    default:
      std::cerr << "Unsupported baudrate: " << baudrate << std::endl;
      return -1;
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Configure 8N1
  tty.c_cflag &= ~PARENB;         // No parity
  tty.c_cflag &= ~CSTOPB;         // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;             // 8 data bits
  tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem control lines

  // Configure raw mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  // Set non-blocking read
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  // Apply settings
  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    std::cerr << "tcsetattr() failed" << std::endl;
    return -1;
  }

  return 0;
}

bool has_data(int fd, int timeout_ms)
{
  pollfd fds{.fd{fd}, .events{POLLIN}};
  
  if (auto ret{::poll(&fds, 1, timeout_ms)}; ret < 0)
  {
    std::cerr << "poll(UART) failed" << std::endl;
    return false;
  }
  else if (ret == 0)
  {
    // Timeout - no data available
    return false;
  }
  
  return true;
}

std::optional<uint8_t> read_byte(pollfd &fds)
{
  if(::poll(&fds, 1, 0) <= 0)
  {
    return std::nullopt;
  }

  if (!(fds.revents & POLLIN))
  {
    return std::nullopt;
  }

  uint8_t byte;
  if (read(fds.fd, &byte, 1) <= 0)
  {
    return std::nullopt;
  }

  return byte;
}

std::optional<packet_with_parity> read_packet(int fd, slip_decoder& decoder)
{
  pollfd fds{.fd{fd}, .events{POLLIN}};
  std::vector<uint8_t> packet;
  
  while (true)
  {
    const auto byte_opt = read_byte(fds);
    if (!byte_opt)
    {
      return std::nullopt;
    }

    if (decoder.decode_byte(*byte_opt, packet))
    {
      break;
    }
  }

  if (packet.size() < 2)
  {
    std::cerr << "UART: packet too short (" << packet.size() << " bytes)" << std::endl;
    return std::nullopt;
  }
  
  // Separate parity from payload
  uint8_t parity = packet.back();
  packet.pop_back();
  
  return packet_with_parity{packet, parity};
}

}  // namespace

slip_decoder::slip_decoder()
{
}

void slip_decoder::reset()
{
  buffer.clear();
  escape_next = false;
}

bool slip_decoder::decode_byte(uint8_t byte, std::vector<uint8_t>& packet)
{
  if (byte == SLIP_END)
  {
    const bool is_frame_complete = (0 < buffer.size());
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

uartif::uartif(const std::string& device, uint32_t baudrate)
  : device{device}
  , baudrate{baudrate}
{
}

uartif::~uartif()
{
  term();
}

void uartif::set_handler(std::function<void(const std::vector<uint8_t>& packet)> handler)
{
  this->handler = handler;
}

int uartif::init()
{
  fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    std::cerr << "Failed to open UART device: " << device << std::endl;
    return -1;
  }

  if (configure_tty(fd, baudrate) < 0)
  {
    term();
    return -1;
  }

  decoder.reset();
  
  return 0;
}

void uartif::term()
{
  if (0 <= fd)
  {
    close(fd);
    fd = -1;
  }

  decoder.reset();
}

int uartif::poll(int timeout_ms) const
{
  if (!handler || fd < 0)
  {
    return 0;
  }
  
  if (!has_data(fd, timeout_ms))
  {
    return 0;
  }
  
  auto result = read_packet(fd, decoder);
  if (!result)
  {
    return 0;
  }
  
  if (!slip_decoder::verify_parity(result->payload, result->parity))
  {
    std::cerr << "UART: parity error: " << std::hex << static_cast<int>(result->parity) << std::dec << "size: " << result->payload.size() << std::endl;
    return 0;
  }

  handler(result->payload);
  return 0;
}