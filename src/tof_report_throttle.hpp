/*
 * Copyright (c) 2026, LexxPluss Inc.
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

// Per-source rate limiting for the persistent fault report.
//
// Header-only and free of ROS so the behaviour can be tested. It exists because
// ROS_ERROR_THROTTLE keeps its state at the call site: a single macro inside a loop over
// the sources is one shared timestamp, so the first source to report suppresses every
// other one on the same round, for as long as the fault lasts. With both sensors down an
// operator would only ever be told about one of them.

#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace lexxhard
{

template <size_t N>
class report_throttle
{
public:
  explicit report_throttle(uint64_t interval_ms) : interval_ms_{ interval_ms }
  {
  }

  // True when this slot is due to report. Each slot keeps its own clock.
  bool should_report(size_t slot, uint64_t now_ms)
  {
    if (slot >= N)
      return false;
    auto& last = last_ms_[slot];
    if (last && now_ms - *last < interval_ms_)
      return false;
    last = now_ms;
    return true;
  }

  // Called when a slot returns to health, so a fault that comes back is reported at once
  // rather than waiting out the remainder of an interval that began while it was broken.
  void clear(size_t slot)
  {
    if (slot < N)
      last_ms_[slot].reset();
  }

private:
  uint64_t interval_ms_;
  std::array<std::optional<uint64_t>, N> last_ms_{};
};

}  // namespace lexxhard
