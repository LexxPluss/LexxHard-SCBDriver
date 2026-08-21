// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <linux/can.h>

#include <cstdint>

#include "tof_cliff_frame.hpp"

namespace lexxhard
{

// The boundary between SCBDriver's CAN ingress and the future ROS/readiness layer.
//
// This interface deliberately carries decoded frame facts only. It does not assemble
// cycles, authorise epochs, infer staleness, compute READY, or publish a ROS message. A
// caller that wants those behaviours must build them above this boundary once the event
// vectors and timing values exist; putting even one of them here would create an
// unreviewed second state machine.
class receiver_cliff_sink
{
public:
  virtual ~receiver_cliff_sink() = default;

  virtual void on_measurement(const tof_cliff_frame::measurement& value) = 0;
  virtual void on_health(const tof_cliff_frame::health& value) = 0;
  virtual void on_rejection(uint32_t can_id, tof_cliff_contract::verdict reason) = 0;
};

class receiver_cliff
{
public:
  explicit receiver_cliff(receiver_cliff_sink& sink);

  // Returns true exactly when the identifier belongs to the cliff contract. A malformed
  // cliff frame is still ours and returns true after reporting its rejection; a neighbour
  // is untouched and returns false.
  bool handle(const can_frame& frame);

private:
  receiver_cliff_sink& sink_;
};

// The production dispatch seam. Both receiver.cpp and the integration tests call this
// function, so the test cannot accidentally bypass the CAN registry while production uses
// it. Returns true when the shared registry routed the frame to the cliff receiver.
bool route_cliff_frame(const can_frame& frame, receiver_cliff& receiver);

}  // namespace lexxhard
