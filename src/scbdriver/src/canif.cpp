/*
 * Copyright (c) 2023, LexxPluss Inc.
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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include "canif.hpp"

canif::canif()
{
}

canif::~canif()
{
    term();
}

void canif::set_handler(std::function<void(const can_frame &frame)> handler)
{
    this->handler = handler;
}

int canif::init()
{
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        std::cerr << "socket(CAN) failed" << std::endl;
        return -1;
    }
    ifreq ifr;
    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "ioctl(SIOCGIFINDEX) failed" << std::endl;
        term();
        return -1;
    }
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) < 0) {
        std::cerr << "setsockopt(CAN_RAW_FILTER) failed" << std::endl;
        term();
        return -1;
    }
    sockaddr_can addr{
        .can_family{AF_CAN},
        .can_ifindex{ifr.ifr_ifindex}
    };
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
        std::cerr << "bind(CAN) failed" << std::endl;
        term();
        return -1;
    }
    if (unsigned long nonblock{1}; ioctl(sock, FIONBIO, &nonblock) < 0) {
        std::cerr << "ioctl(FIONBIO) failed" << std::endl;
        term();
        return -1;
    }
    return 0;
}

void canif::term()
{
    if (sock >= 0) {
        close(sock);
        sock = -1;
    }
}

int canif::poll(int timeout_ms) const
{
    pollfd fds{
        .fd{sock},
        .events{POLLIN}
    };
    for (int i{0}; i < 10; ++i) {
        if (auto ret{::poll(&fds, 1, timeout_ms)}; ret < 0) {
            std::cerr << "poll(CAN) failed" << std::endl;
            return -1;
        } else if (ret == 0) {
            return 0;
        } else if (fds.revents & POLLIN) {
            can_frame frame;
            if (read(sock, &frame, sizeof frame) < 0) {
                std::cerr << "read(CAN) failed" << std::endl;
                return -1;
            }
            if (handler)
                handler(frame);
        }
    }
    return 0;
}

int canif::send(const can_frame &frame) const
{
    if (write(sock, &frame, sizeof frame) < 0) {
        std::cerr << "write(CAN) failed" << std::endl;
        return -1;
    }
    return 0;
}
