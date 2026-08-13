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

#include "devif.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>

namespace
{

int configure_tty(int fd, uint32_t baudrate)
{
  termios tty;
  if (tcgetattr(fd, &tty) != 0)
  {
    std::cerr << "tcgetattr() failed" << std::endl;
    return -1;
  }

  speed_t speed;
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

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    std::cerr << "tcsetattr() failed" << std::endl;
    return -1;
  }

  return 0;
}

}  // namespace

devif::devif(queue_type& queue) : queue{ &queue }
{
}

devif::~devif()
{
  term();
}

int devif::add_can(const std::string& ifname, const can_filter* filter, size_t nfilter)
{
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0)
  {
    std::cerr << "socket(CAN) failed" << std::endl;
    return -1;
  }

  ifreq ifr;
  strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
  {
    std::cerr << "ioctl(SIOCGIFINDEX) failed" << std::endl;
    close(sock);
    return -1;
  }

  if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, filter, nfilter) < 0)
  {
    std::cerr << "setsockopt(CAN_RAW_FILTER) failed" << std::endl;
    close(sock);
    return -1;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0)
  {
    std::cerr << "bind(CAN) failed" << std::endl;
    close(sock);
    return -1;
  }

  if (unsigned long nonblock{ 1 }; ioctl(sock, FIONBIO, &nonblock) < 0)
  {
    std::cerr << "ioctl(FIONBIO) failed" << std::endl;
    close(sock);
    return -1;
  }

  can_devices.push_back({ sock });
  return 0;
}

int devif::add_uart(const std::string& device, uint32_t baudrate)
{
  int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    std::cerr << "Failed to open UART device: " << device << std::endl;
    return -1;
  }

  if (configure_tty(fd, baudrate) < 0)
  {
    close(fd);
    return -1;
  }

  uart_devices.push_back({ fd, {} });
  return 0;
}

void devif::term()
{
  for (auto& dev : can_devices)
  {
    if (dev.fd >= 0)
    {
      close(dev.fd);
      dev.fd = -1;
    }
  }
  can_devices.clear();

  for (auto& dev : uart_devices)
  {
    if (dev.fd >= 0)
    {
      close(dev.fd);
      dev.fd = -1;
    }
  }
  uart_devices.clear();
}

int devif::poll(int timeout_ms)
{
  if (!queue)
  {
    return 0;
  }

  std::vector<pollfd> fds;
  fds.reserve(can_devices.size() + uart_devices.size());

  for (const auto& dev : can_devices)
  {
    fds.push_back({ dev.fd, POLLIN, 0 });
  }
  for (const auto& dev : uart_devices)
  {
    fds.push_back({ dev.fd, POLLIN, 0 });
  }

  if (fds.empty())
  {
    return 0;
  }

  int ret = ::poll(fds.data(), fds.size(), timeout_ms);
  if (ret < 0)
  {
    if (errno != EINTR)
    {
      std::cerr << "poll() failed" << std::endl;
    }
    return -1;
  }

  if (ret == 0)
  {
    return 0;
  }

  size_t idx = 0;
  for (auto& dev : can_devices)
  {
    if (fds[idx].revents & POLLIN)
    {
      read_can(dev);
    }
    ++idx;
  }
  for (auto& dev : uart_devices)
  {
    if (fds[idx].revents & POLLIN)
    {
      read_uart(dev);
    }
    ++idx;
  }

  return 0;
}

int devif::read_can(can_device& dev)
{
  while (true)
  {
    can_frame frame;
    auto ret = read(dev.fd, &frame, sizeof frame);
    if (ret < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return 0;
      }

      std::cerr << "read(CAN) failed" << std::endl;
      return -1;
    }

    if (!queue->push(can_message{ frame }))
    {
      std::cerr << "CAN queue full, dropping frame" << std::endl;
    }
  }
}

int devif::read_uart(uart_device& dev)
{
  uint8_t buf[4096];

  while (true)
  {
    auto ret = read(dev.fd, buf, sizeof buf);
    if (ret < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return 0;
      }
      std::cerr << "read(UART) failed" << std::endl;
      return -1;
    }

    if (ret == 0)
    {
      return 0;
    }

    for (ssize_t i = 0; i < ret; ++i)
    {
      std::vector<uint8_t> packet;
      if (dev.decoder.decode_byte(buf[i], packet))
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
          if (!queue->push(uart_message{ std::move(packet) }))
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

int devif::send_can(const can_frame& frame) const
{
  if (can_devices.empty())
  {
    return -1;
  }

  if (write(can_devices[0].fd, &frame, sizeof frame) < 0)
  {
    std::cerr << "write(CAN) failed" << std::endl;
    return -1;
  }
  return 0;
}
