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
#include <sstream>
#include "receiver_bmu.hpp"

receiver_bmu::receiver_bmu(ros::NodeHandle& n)
  : pub{ n.advertise<scbdriver::Battery>("/sensor_set/battery", queue_size) }
{
}

void receiver_bmu::handle(const can_frame& frame)
{
  if (frame.can_id == 0x100)
  {
    bmudata.mod_status1 = frame.data[0];
    bmudata.bmu_status = frame.data[1];
    bmudata.asoc = frame.data[2];
    bmudata.rsoc = frame.data[3];
    bmudata.soh = frame.data[4];
    bmudata.fet_temp = (frame.data[5] << 8) | frame.data[6];
  }
  else if (frame.can_id == 0x101)
  {
    bmudata.pack_current = (frame.data[0] << 8) | frame.data[1];
    bmudata.charging_current = (frame.data[2] << 8) | frame.data[3];
    bmudata.pack_voltage = (frame.data[4] << 8) | frame.data[5];
    bmudata.mod_status2 = frame.data[6];
  }
  else if (frame.can_id == 0x103)
  {
    bmudata.design_capacity = (frame.data[0] << 8) | frame.data[1];
    bmudata.full_charge_capacity = (frame.data[2] << 8) | frame.data[3];
    bmudata.remain_capacity = (frame.data[4] << 8) | frame.data[5];
  }
  else if (frame.can_id == 0x110)
  {
    bmudata.max_voltage.value = (frame.data[0] << 8) | frame.data[1];
    bmudata.max_voltage.id = frame.data[2];
    bmudata.min_voltage.value = (frame.data[4] << 8) | frame.data[5];
    bmudata.min_voltage.id = frame.data[6];
  }
  else if (frame.can_id == 0x111)
  {
    bmudata.max_temp.value = (frame.data[0] << 8) | frame.data[1];
    bmudata.max_temp.id = frame.data[2];
    bmudata.min_temp.value = (frame.data[4] << 8) | frame.data[5];
    bmudata.min_temp.id = frame.data[6];
  }
  else if (frame.can_id == 0x112)
  {
    bmudata.max_current.value = (frame.data[0] << 8) | frame.data[1];
    bmudata.max_current.id = frame.data[2];
    bmudata.min_current.value = (frame.data[4] << 8) | frame.data[5];
    bmudata.min_current.id = frame.data[6];
  }
  else if (frame.can_id == 0x113)
  {
    bmudata.bmu_fw_ver = frame.data[0];
    bmudata.mod_fw_ver = frame.data[1];
    bmudata.serial_config = frame.data[2];
    bmudata.parallel_config = frame.data[3];
    bmudata.bmu_alarm1 = frame.data[4];
    bmudata.bmu_alarm2 = frame.data[5];
  }
  else if (frame.can_id == 0x120)
  {
    bmudata.min_cell_voltage.value = (frame.data[0] << 8) | frame.data[1];
    bmudata.min_cell_voltage.id = frame.data[2];
    bmudata.max_cell_voltage.value = (frame.data[4] << 8) | frame.data[5];
    bmudata.max_cell_voltage.id = frame.data[6];
  }
  else if (frame.can_id == 0x130)
  {
    bmudata.manufacturing = (frame.data[0] << 8) | frame.data[1];
    bmudata.inspection = (frame.data[2] << 8) | frame.data[3];
    bmudata.serial = (frame.data[4] << 8) | frame.data[5];
    scbdriver::Battery msg;
    decode(msg);
    pub.publish(msg);
  }
}

void receiver_bmu::decode(scbdriver::Battery& msg) const
{
  std::ostringstream os;
  os << std::hex << std::setfill('0') << std::setw(4) << bmudata.serial;
  msg.state.voltage = bmudata.pack_voltage * 1e-3f;
  msg.state.temperature = bmudata.fet_temp * 1e-2f;
  msg.state.current = bmudata.pack_current * 1e-2f;
  msg.state.charge = bmudata.remain_capacity * 1e-2f;
  msg.state.capacity = bmudata.full_charge_capacity * 1e-2f;
  msg.state.design_capacity = bmudata.design_capacity * 1e-2f;
  msg.state.percentage = bmudata.rsoc * 1e-2f;
  msg.state.power_supply_status = (bmudata.mod_status1 & 0b01000000) != 0 ?
                                      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL :
                                  msg.state.current > 0.0f ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
                                                             sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.state.power_supply_health =
      ((bmudata.mod_status1 & 0b00100000) != 0 || bmudata.bmu_status == 0x07 || bmudata.bmu_status == 0x09 ||
       (bmudata.bmu_alarm1 & 0b10000010) != 0) ?
          sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT :
      (bmudata.mod_status1 & 0b00011000) != 0 ? sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE :
      (bmudata.mod_status2 & 0b11100000) != 0 ? sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD :
      ((bmudata.mod_status1 & 0b10000111) != 0 || (bmudata.mod_status2 & 0b00000001) != 0 ||
       bmudata.bmu_status == 0xf0 || bmudata.bmu_status == 0xf1 || (bmudata.bmu_alarm1 & 0b01111101) != 0 ||
       (bmudata.bmu_alarm2 & 0b00000001) != 0) ?
                                                sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE :
                                                sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.state.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.state.present = true;
  msg.state.cell_voltage.resize(2);
  msg.state.cell_voltage[0] = bmudata.max_cell_voltage.value * 1e-3f;
  msg.state.cell_voltage[1] = bmudata.min_cell_voltage.value * 1e-3f;
  msg.state.cell_temperature.resize(2);
  msg.state.cell_temperature[0] = bmudata.max_temp.value * 1e-2f;
  msg.state.cell_temperature[1] = bmudata.min_temp.value * 1e-2f;
  msg.state.location = "0";
  msg.state.serial_number = os.str();
}
