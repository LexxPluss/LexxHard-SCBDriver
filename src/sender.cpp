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

#include <iostream>
#include "ros/ros.h"
#include "canif.hpp"
#include "sender_actuator.hpp"
#include "sender_board.hpp"
#include "sender_dfu.hpp"
#include "sender_gpio.hpp"
#include "sender_led.hpp"
#include "sender_pgv.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sender");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    canif can;
    if (can.init(nullptr, 0) < 0) {
        std::cerr << "canif::init() failed" << std::endl;
        return -1;
    }
    sender_actuator actuator{n, pn, can};
    sender_board board{n, can};
    sender_dfu dfu{n, can};
    sender_gpio gpio{n, can};
    sender_led led{n, can};
    sender_pgv pgv{n, can};
    ros::MultiThreadedSpinner spinner{3};
    spinner.spin();
    can.term();
    return 0;
}
