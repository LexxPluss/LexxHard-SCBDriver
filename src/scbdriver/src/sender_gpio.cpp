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

#include <gpiod.hpp>
#include "sender_gpio.hpp"

sender_gpio::sender_gpio(ros::NodeHandle &n)
    : sub_wheelmotor{n.subscribe("/gpio_control/wheelmotor", queue_size, &sender_gpio::handle_wheelmotor, this)},
      sub_autocharge{n.subscribe("/gpio_control/autocharge", queue_size, &sender_gpio::handle_autocharge, this)}
{
    gpiod::chip chip{"gpiochip1"};
    line_wheelmotor = chip.get_line(0);
    line_autocharge = chip.get_line(1);
    line_wheelmotor.request({"sender_gpio", gpiod::line_request::DIRECTION_OUTPUT, 0});
    line_autocharge.request({"sender_gpio", gpiod::line_request::DIRECTION_OUTPUT, 0});
}

sender_gpio::~sender_gpio()
{
    line_wheelmotor.release();
    line_autocharge.release();
}

void sender_gpio::handle_wheelmotor(const std_msgs::Bool::ConstPtr &msg) const
{
    line_wheelmotor.set_value(msg->data);
}

void sender_gpio::handle_autocharge(const std_msgs::Bool::ConstPtr &msg) const
{
    line_autocharge.set_value(msg->data);
}
