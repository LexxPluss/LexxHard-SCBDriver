/*
 * Copyright (c) 2025, LexxPluss Inc.
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

#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <utility>

template <typename T, size_t Capacity>
class spsc_queue
{
public:
  spsc_queue() = default;

  template <typename U>
  bool push(U&& item)
  {
    const size_t head_idx = head.load(std::memory_order_relaxed);
    const size_t next_idx = (head_idx + 1) % Capacity;

    if (next_idx == tail.load(std::memory_order_acquire))
    {
      return false;
    }

    buffer[head_idx] = std::forward<U>(item);
    head.store(next_idx, std::memory_order_release);
    return true;
  }

  bool pop(T& item)
  {
    const size_t tail_idx = tail.load(std::memory_order_relaxed);

    if (tail_idx == head.load(std::memory_order_acquire))
    {
      return false;
    }

    item = std::move(buffer[tail]);
    tail.store((tail_idx + 1) % Capacity, std::memory_order_release);
    return true;
  }

  bool empty() const
  {
    return head.load(std::memory_order_acquire) == tail.load(std::memory_order_acquire);
  }

private:
  std::array<T, Capacity> buffer;
  std::atomic<size_t> head{0};
  std::atomic<size_t> tail{0};
};
