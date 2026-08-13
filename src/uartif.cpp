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
#include <cerrno>
#include <cstring>

namespace
{

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
    case 9600:
      speed = B9600;
      break;
    case 19200:
      speed = B19200;
      break;
    case 38400:
      speed = B38400;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    case 230400:
      speed = B230400;
      break;
    case 460800:
      speed = B460800;
      break;
    default:
      std::cerr << "Unsupported baudrate: " << baudrate << std::endl;
      return -1;
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Configure 8N1
  tty.c_cflag &= ~PARENB;  // No parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;             // 8 data bits
  tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem control lines

  // Configure raw mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
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
  pollfd fds{ .fd{ fd }, .events{ POLLIN } };

  if (auto ret{ ::poll(&fds, 1, timeout_ms) }; ret < 0)
  {
    if (errno != EINTR)
    {
      std::cerr << "poll(UART) failed" << std::endl;
    }
    return false;
  }
  else if (ret == 0)
  {
    // Timeout - no data available
    return false;
  }

  return true;
}

}  // namespace

uartif::uartif(const std::string& device, uint32_t baudrate) : device{ device }, baudrate{ baudrate }
{
}

uartif::uartif(const std::string& device, uint32_t baudrate, queue_type& q)
  : device{ device }, baudrate{ baudrate }, queue{ &q }
{
}

uartif::~uartif()
{
  term();
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
  if (!queue || fd < 0)
  {
    return 0;
  }

  if (!has_data(fd, timeout_ms))
  {
    return 0;
  }

  uint8_t buf[4096];
  while (true)
  {
    auto n = read(fd, buf, sizeof buf);
    if (n < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return 0;
      }
      std::cerr << "read(UART) failed" << std::endl;
      return -1;
    }

    if (n == 0)
    {
      return 0;
    }

    for (ssize_t i = 0; i < n; ++i)
    {
      std::vector<uint8_t> packet;
      if (decoder.decode_byte(buf[i], packet))
      {
        if (packet.size() < 2)
        {
          std::cerr << "UART: packet too short (" << packet.size() << " bytes)" << std::endl;
          continue;
        }

        uint8_t parity = packet.back();
        packet.pop_back();
        if (slip_decoder::verify_parity(packet, parity))
        {
          if (!queue->push(std::move(packet)))
          {
            std::cerr << "UART queue full, dropping packet" << std::endl;
          }
        }
        else
        {
          std::cerr << "UART: parity error" << std::endl;
        }
      }
    }
  }
}
