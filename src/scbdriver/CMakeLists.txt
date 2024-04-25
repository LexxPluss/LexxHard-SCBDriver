# Copyright (c) 2024, LexxPluss Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.0.2)
project(scbdriver)

add_compile_options(-std=c++17)

find_package(PkgConfig)
pkg_check_modules(libgpiod REQUIRED libgpiod)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Battery.msg
  Imu.msg
  Led.msg
  LinearActuatorControl.msg
  LinearActuatorControlArray.msg
  PositionGuideVision.msg
)

add_service_files(
  FILES
  InitLinearActuator.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  receiver
  src/canif.cpp
  src/receiver.cpp
  src/receiver_actuator.cpp
  src/receiver_bmu.cpp
  src/receiver_board.cpp
  src/receiver_dfu.cpp
  src/receiver_imu.cpp
  src/receiver_pgv.cpp
  src/receiver_uss.cpp
)
add_executable(
  sender
  src/canif.cpp
  src/sender.cpp
  src/sender_actuator.cpp
  src/sender_board.cpp
  src/sender_dfu.cpp
  src/sender_gpio.cpp
  src/sender_led.cpp
  src/sender_pgv.cpp
)
add_executable(
  updater
  src/updater.cpp
)

add_dependencies(receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updater ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(receiver
  ${catkin_LIBRARIES}
)
target_link_libraries(sender
  ${catkin_LIBRARIES}
  ${libgpiod_LIBRARIES}
)
target_link_libraries(updater
  ${catkin_LIBRARIES}
)
