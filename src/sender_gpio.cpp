/*
 * Copyright (c) 2024, LexxPluss Inc.
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
#include "canif.hpp"
#include "sender_gpio.hpp"

namespace {
    uint8_t set_out_port_status(const uint8_t status, const uint8_t port, const bool value)
    {
        const uint8_t bit_pos = 7 - port;
        return (status & ~(1 << bit_pos)) | (value << bit_pos);
    }
}

sender_gpio::sender_gpio(ros::NodeHandle &n, canif &can)
    : subs{{
        n.subscribe("gpio/out_port0", queue_size, &sender_gpio::handle<0>, this),
        n.subscribe("gpio/out_port1", queue_size, &sender_gpio::handle<1>, this),
        n.subscribe("gpio/out_port2", queue_size, &sender_gpio::handle<2>, this),
        n.subscribe("gpio/out_port3", queue_size, &sender_gpio::handle<3>, this),
      }},
      can{can}
{
}

template<uint8_t N>
void sender_gpio::handle(const std_msgs::Bool::ConstPtr& msg)
{
    frame.data[0] = set_out_port_status(frame.data[0], N, msg->data);
    can.send(frame);
}
