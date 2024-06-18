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

#include <algorithm>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

namespace {

class updater {
public:
    updater(ros::NodeHandle &n)
        : pub{n.advertise<std_msgs::UInt8MultiArray>("/lexxhard/dfu_data", queue_size)},
          sub{n.subscribe("/lexxhard/dfu_response", queue_size, &updater::callback, this)}
    {
    }
    void start() {
        command(0, CMD::START, nullptr);
        address = 0;
    }
    void send(const uint8_t *data) {
        command(address, CMD::SEND, data);
        ++address;
    }
    void end() {
        command(0, CMD::END, nullptr);
    }
    void reset() {
        command(0, CMD::RESET, nullptr);
    }
private:
    enum class CMD {
        START = 0,
        SEND = 1,
        END = 2,
        RESET = 3,
    };
    enum class RESP {
        OK    = 0,
        ERROR = 1,
        AGAIN = 2,
    };
    void command(uint16_t address, CMD cmd, const uint8_t *data) {
        std_msgs::UInt8MultiArray msg;
        msg.data.resize(8);
        msg.data[0] = address >> 8;
        msg.data[1] = address;
        msg.data[2] = static_cast<uint8_t>(cmd);
        if (data != nullptr) {
            msg.data[3] = 4;
            std::copy_n(data, 4, std::begin(msg.data) + 4);
        } else {
            std::fill_n(std::begin(msg.data) + 3, 5, 0);
        }
        send_and_wait(msg);
    }
    void callback(const std_msgs::UInt8MultiArray::ConstPtr &msg) {
        response.received = true;
        std::copy_n(std::begin(msg->data), 4, std::begin(response.data));
    }
    void send_and_wait(const std_msgs::UInt8MultiArray &msg) {
        response.received = false;
        for (int retry{0}; retry < 5; ++retry) {
            pub.publish(msg);
            if (auto resp{wait_response(msg)};
                resp == static_cast<int>(RESP::OK) ||
                resp == static_cast<int>(RESP::ERROR))
                break;
        }
    }
    int wait_response(const std_msgs::UInt8MultiArray &msg) {
        ros::Rate cycle{1000};
        ros::Time start{ros::Time::now()};
        while (ros::ok() && (ros::Time::now() - start).toSec() < 5.0) {
            ros::spinOnce();
            cycle.sleep();
            if (response.received) {
                if (response.data[0] == msg.data[0] &&
                    response.data[1] == msg.data[1] &&
                    response.data[2] == msg.data[2]) {
                    return response.data[3];
                }
            }
        }
        ROS_INFO("response timeout");
        return -1;
    }
    ros::Publisher pub;
    ros::Subscriber sub;
    struct {
        uint8_t data[4];
        bool received{false};
    } response;
    uint16_t address{0};
    static constexpr uint32_t queue_size{10};
};

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "updater");
    ros::NodeHandle nh;
    updater up{nh};
    if (argc < 2) {
        ROS_INFO("Usage: %s <file>", argv[0]);
        return 0;
    }
    std::ifstream f{argv[1]};
    if (!f) {
        ROS_ERROR("ERROR: unable to open %s", argv[1]);
        return 0;
    }
    up.start();
    size_t sent{0};
    while (ros::ok()) {
        uint8_t buf[4];
        f.read(reinterpret_cast<char*>(buf), sizeof buf);
        if (f.eof()) {
            break;
        } else if (!f) {
            ROS_ERROR("ERROR: failed to read");
            return 0;
        }
        up.send(buf);
        sent += sizeof buf;
        if (sent % 4096 == 0)
            ROS_INFO("sent %lu bytes", sent);
    }
    up.end();
    up.reset();
    return 0;
}
