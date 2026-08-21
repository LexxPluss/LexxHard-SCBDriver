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
  LinearActuatorServiceResponse.msg
  PositionGuideVision.msg
  BoardTemperatures.msg
  CliffTofSafetyHealth.msg
)

add_service_files(
  FILES
  InitLinearActuator.srv
  LinearActuatorLocation.srv
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
  src/uartif.cpp
  src/receiver.cpp
  src/receiver_actuator.cpp
  src/receiver_bmu.cpp
  src/receiver_board.cpp
  src/receiver_dfu.cpp
  src/receiver_imu.cpp
  src/receiver_pgv.cpp
  src/receiver_uss.cpp
  src/receiver_gpio.cpp
  src/receiver_tug_encoder.cpp
  src/receiver_tof.cpp
  src/tof_grid_assembler.cpp
  src/tof_can_config.cpp
  src/tof_cliff_frame.cpp
  src/receiver_cliff.cpp
  src/slip_decoder.cpp
)
# src/vendor holds the production half of the cliff wire contract, which
# tof_cliff_frame.hpp includes by bare name. Only the production half is reachable this
# way: the vector header lives in test/vendor, which is on no production include path.
target_include_directories(receiver PRIVATE src/vendor)
add_executable(
  sender
  src/devif.cpp
  src/sender.cpp
  src/sender_actuator.cpp
  src/sender_board.cpp
  src/sender_dfu.cpp
  src/sender_gpio.cpp
  src/sender_led.cpp
  src/sender_pgv.cpp
  src/slip_decoder.cpp
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

install(TARGETS receiver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS sender
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # test_tof_pipeline includes receiver_tof.cpp directly, which now reaches into the
  # assembler, so it needs that translation unit as well.
  catkin_add_gtest(test_tof_pipeline
    test/test_tof_pipeline.cpp
    src/slip_decoder.cpp
    src/tof_grid_assembler.cpp
  )
  target_include_directories(test_tof_pipeline PRIVATE src)
  target_link_libraries(test_tof_pipeline ${catkin_LIBRARIES})
  add_dependencies(test_tof_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # Driven entirely by the golden vectors vendored from the firmware repository, so this
  # target needs neither ROS nor a CAN interface.
  catkin_add_gtest(test_tof_grid_assembler
    test/test_tof_grid_assembler.cpp
    src/tof_grid_assembler.cpp
    src/tof_can_config.cpp
  )
  target_include_directories(test_tof_grid_assembler PRIVATE src test)
  target_link_libraries(test_tof_grid_assembler ${catkin_LIBRARIES})
  add_dependencies(test_tof_grid_assembler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # Specification test for the cliff safety-health message. It asserts against the
  # generated header rather than the .msg text, so message generation, the constant
  # block and the field types are covered together, and it carries the reference
  # readiness mapping that the future publisher must agree with.
  catkin_add_gtest(test_cliff_tof_safety_health test/test_cliff_tof_safety_health.cpp)
  target_compile_definitions(test_cliff_tof_safety_health
    PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_cliff_tof_safety_health ${catkin_LIBRARIES})
  add_dependencies(test_cliff_tof_safety_health ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # Frame-level cliff decode: validation and field unpacking only, no state machine, so
  # like the grid assembler it needs neither ROS nor a CAN interface. Driven entirely by
  # the layout vectors vendored from the firmware repository.
  #
  # src/vendor holds the contract's PRODUCTION half; test/vendor holds the vector header.
  # The separation is structural rather than a convention: production code cannot reach
  # the vectors, because test/vendor is not on any production include path. src/vendor is
  # listed alongside src here because the vector header includes the production header by
  # bare name -- correct in the firmware repository, where the generator puts both in one
  # directory, and the reason this repository supplies both directories rather than keep a
  # second copy of the production header to drift.
  catkin_add_gtest(test_tof_cliff_frame
    test/test_tof_cliff_frame.cpp
    src/tof_cliff_frame.cpp
  )
  target_include_directories(test_tof_cliff_frame PRIVATE src src/vendor test)
  target_link_libraries(test_tof_cliff_frame ${catkin_LIBRARIES})
  add_dependencies(test_tof_cliff_frame ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # Non-ROS cliff ingress: the same shared CAN registry route used by receiver.cpp, the
  # production frame decoder, and a fake typed sink. Captured dasher1 payloads make the
  # producer/consumer splice executable without granting an unproven frame access to the
  # live robot bus.
  catkin_add_gtest(test_receiver_cliff
    test/test_receiver_cliff.cpp
    src/receiver_cliff.cpp
    src/tof_cliff_frame.cpp
  )
  target_include_directories(test_receiver_cliff PRIVATE src src/vendor)
  target_link_libraries(test_receiver_cliff ${catkin_LIBRARIES})
  add_dependencies(test_receiver_cliff ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
