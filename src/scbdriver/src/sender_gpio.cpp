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

#include <gpiod.h>
#include "sender_gpio.hpp"

sender_gpio::sender_gpio(ros::NodeHandle &n)
    : sub_wheelmotor{n.subscribe("/gpio_control/wheelmotor", queue_size, &sender_gpio::handle_wheelmotor, this)},
      sub_autocharge{n.subscribe("/gpio_control/autocharge", queue_size, &sender_gpio::handle_autocharge, this)}
{
    chip = gpiod_chip_open_by_name("gpiochip1");
    if (chip != nullptr) {
        line_wheelmotor = gpiod_chip_get_line(chip, 0);
        line_autocharge = gpiod_chip_get_line(chip, 1);
    }
    if (line_wheelmotor != nullptr)
        gpiod_line_request_output(line_wheelmotor, "sender_gpio", 0);
    if (line_autocharge != nullptr)
        gpiod_line_request_output(line_autocharge, "sender_gpio", 0);
}

sender_gpio::~sender_gpio()
{
    if (line_wheelmotor != nullptr) {
        gpiod_line_release(line_wheelmotor);
        line_wheelmotor = nullptr;
    }
    if (line_autocharge != nullptr) {
        gpiod_line_release(line_autocharge);
        line_autocharge = nullptr;
    }
    if (chip != nullptr) {
        gpiod_chip_close(chip);
        chip = nullptr;
    }
}

void sender_gpio::handle_wheelmotor(const std_msgs::Bool::ConstPtr &msg) const
{
    if (line_wheelmotor != nullptr)
        gpiod_line_set_value(line_wheelmotor, msg->data);
}

void sender_gpio::handle_autocharge(const std_msgs::Bool::ConstPtr &msg) const
{
    if (line_autocharge != nullptr)
        gpiod_line_set_value(line_autocharge, msg->data);
}
