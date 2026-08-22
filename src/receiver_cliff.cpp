// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "receiver_cliff.hpp"

#include "can_ids.hpp"

namespace lexxhard
{

namespace ctr = tof_cliff_contract;
namespace fr = tof_cliff_frame;

// The receive filter/registry and decoder must not carry independent copies of the wire
// identifiers. If either generated contract value moves, this translation unit refuses to
// compile until the registry moves in the same reviewed change.
static_assert(can_ids::TOF_CLIFF_MEASUREMENT == ctr::kMeasId, "cliff measurement CAN ID drifted from the wire "
                                                              "contract");
static_assert(can_ids::TOF_CLIFF_HEALTH == ctr::kHealthId, "cliff health CAN ID drifted from the wire contract");

receiver_cliff::receiver_cliff(receiver_cliff_sink& sink) : sink_{ sink }
{
}

bool receiver_cliff::handle(const can_frame& frame)
{
  const fr::decoded decoded = fr::decode(frame.can_id, frame.can_dlc, frame.data);
  if (decoded.which == fr::arrival::not_ours)
    return false;

  // A cliff identifier always produces a verdict. Keeping the optional check here makes
  // the fail-closed behaviour explicit if that decoder invariant is ever broken: no typed
  // event escapes without an accept verdict.
  if (!decoded.result.has_value())
    return true;

  if (*decoded.result != ctr::verdict::accept)
  {
    sink_.on_rejection(frame.can_id, *decoded.result);
    return true;
  }

  if (decoded.which == fr::arrival::measurement)
    sink_.on_measurement(decoded.meas);
  else
    sink_.on_health(decoded.state);
  return true;
}

bool route_cliff_frame(const can_frame& frame, receiver_cliff& receiver)
{
  if (can_ids::route(frame.can_id) != can_ids::owner::cliff)
    return false;

  // The static assertions above make a registry/decoder disagreement impossible for the
  // two current IDs. A cliff-owned frame is claimed even if it is malformed; otherwise it
  // could fall through into another receiver after already being recognised as ours.
  (void)receiver.handle(frame);
  return true;
}

}  // namespace lexxhard
