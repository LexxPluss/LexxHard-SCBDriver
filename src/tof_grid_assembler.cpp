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

#include "tof_grid_assembler.hpp"

namespace lexxhard
{

namespace
{

constexpr uint8_t DLC = 8;

uint32_t elapsed(uint32_t from, uint32_t to)
{
  // Monotonic, but the caller's clock may wrap. Unsigned subtraction handles the wrap; a
  // "to" that precedes "from" would otherwise become an enormous elapsed time and expire
  // every slot at once.
  return static_cast<uint32_t>(to - from);
}

}  // namespace

// Bytes 6-7 and status flag bits 4-7 are reserved. They are dropped here rather than
// stored, so nothing downstream can branch on a field whose meaning is undefined, and two
// health frames that differ only in reserved fields compare equal.
tof_grid_assembler::health_info tof_grid_assembler::parse_health(const uint8_t* p)
{
  health_info h;
  h.valid_zone_count = p[2];
  h.flags = static_cast<uint8_t>(p[3] & STATUS_FLAG_MASK);
  h.chain_position = static_cast<uint8_t>(p[4] & 0x0F);
  h.boards_detected = static_cast<uint8_t>(p[4] >> 4);
  h.last_error = p[5];
  return h;
}

bool tof_grid_assembler::normalised_equal(const health_info& a, const health_info& b)
{
  // Comparing the eight raw bytes instead would, as soon as a firmware started populating
  // the reserved fields, read two semantically identical frames as contradicting each
  // other and retire a perfectly good grid. Forward compatibility must not cost data.
  return a.valid_zone_count == b.valid_zone_count && a.flags == b.flags && a.chain_position == b.chain_position &&
         a.boards_detected == b.boards_detected && a.last_error == b.last_error;
}

tof_grid_assembler::tof_grid_assembler(uint32_t data_can_id, uint32_t health_can_id, uint32_t startup_time_ms)
  : data_can_id_{ data_can_id }, health_can_id_{ health_can_id }, startup_time_ms_{ startup_time_ms }
{
}

void tof_grid_assembler::emit(event e, uint8_t source, source_state state)
{
  ++counters_[static_cast<size_t>(e)];
  events_.push_back(diagnostic{ e, source, state });
}

void tof_grid_assembler::retire(uint8_t source)
{
  slot& s = slots_[source];
  tracker& t = trackers_[source];
  t.retired_valid = true;
  t.retired_generation = s.generation;
  s = slot{};
}

void tof_grid_assembler::expire_stale_slots(uint32_t now_ms)
{
  for (uint8_t src = 0; src < SOURCE_COUNT; ++src)
  {
    slot& s = slots_[src];
    if (!s.active || elapsed(s.first_seen_ms, now_ms) <= ASSEMBLY_TIMEOUT_MS)
      continue;
    // An assembly holding only a health frame timed out without its data; that is a
    // different diagnosis from losing chunks, so it gets its own counter.
    emit(s.bitmap == 0 && s.health_seen ? event::ORPHAN_HEALTH_TIMEOUT : event::INCOMPLETE_BY_TIMEOUT, src);
    retire(src);
  }
}

std::optional<tof_grid_assembler::grid> tof_grid_assembler::try_complete(uint8_t source, uint32_t now_ms)
{
  slot& s = slots_[source];
  if (!s.active || s.bitmap != COMPLETE_BITMAP || !s.health_seen)
    return std::nullopt;

  uint8_t decoded_valid = 0;
  for (uint16_t z : s.zones_mm)
  {
    if (z != INVALID_MM)
      ++decoded_valid;
  }

  if (s.health.valid_zone_count != decoded_valid)
  {
    // The packer contradicts its own summary. One of the two is wrong and nothing here
    // identifies which, so the zone data is not trustworthy either. Rejecting keeps the
    // field meaningful; publishing with a warning would leave it with no contract value.
    emit(event::HEALTH_COUNT_MISMATCH, source);
    retire(source);
    return std::nullopt;
  }

  grid g;
  g.source = source;
  g.generation = s.generation;
  g.zones_mm = s.zones_mm;
  g.health = s.health;

  tracker& t = trackers_[source];
  t.ever_published = true;
  t.last_publish_ms = now_ms;
  if (t.alarm_active)
  {
    // Recovery is reported wherever it happens, including the first grid ever seen from
    // a source that had already been alarmed as NEVER_SEEN.
    emit(event::SOURCE_RECOVERED, source, source_state::HEALTHY);
    t.alarm_active = false;
  }

  emit(event::GRID_PUBLISHED, source, source_state::HEALTHY);
  retire(source);
  return g;
}

std::optional<tof_grid_assembler::grid> tof_grid_assembler::consume(uint32_t can_id, uint8_t dlc,
                                                                    const uint8_t* payload, uint32_t now_ms)
{
  const bool is_data = can_id == data_can_id_;
  const bool is_health = can_id == health_can_id_;
  if (!is_data && !is_health)
    return std::nullopt;

  expire_stale_slots(now_ms);

  if (dlc != DLC || payload == nullptr)
  {
    emit(event::MALFORMED_HEADER);
    return std::nullopt;
  }

  const uint8_t generation = payload[0];
  const uint8_t source = static_cast<uint8_t>(payload[1] >> 4);
  const uint8_t low_nibble = static_cast<uint8_t>(payload[1] & 0x0F);

  // An unknown source cannot be attributed to a tracker at all, so it stops here.
  if (source >= SOURCE_COUNT)
  {
    emit(event::MALFORMED_HEADER);
    return std::nullopt;
  }

  slot& s = slots_[source];
  tracker& t = trackers_[source];

  // Recorded before the structural checks below: a frame that names a real source but is
  // otherwise corrupt is still evidence that the transport is alive. Counting it as no
  // frame at all would report STALE_NO_FRAMES and send an investigation towards the
  // chain, power or CAN filter, when the actual fault is corruption.
  t.ever_seen_frame = true;
  t.last_frame_ms = now_ms;

  if (is_health)
  {
    // Reserved low nibble, and a count that cannot describe a 64-zone grid, are both
    // structural faults. 0xFF is reserved for a future status-only frame and is not
    // defined yet, so it lands here too.
    if (low_nibble != 0 || payload[2] > ZONES)
    {
      emit(event::MALFORMED_HEADER, source);
      return std::nullopt;
    }
  }

  // A generation is single use: once it has ended, by publication or by any rejection, a
  // late or retransmitted frame must not restart it. Without this, a timeout would clear
  // the rejection and the very frames that were judged untrustworthy could reassemble and
  // publish.
  if (t.retired_valid && t.retired_generation == generation)
  {
    emit(event::FRAME_FOR_RETIRED_GENERATION, source);
    return std::nullopt;
  }

  // Generation equality, not ordering, is what prevents two grids being spliced. A
  // differing generation never fills a hole in the current slot; it replaces it. Because
  // this is an equality test the 255->0 wrap is an ordinary new grid, not a regression.
  if (s.active && s.generation != generation)
  {
    emit(event::INCOMPLETE_BY_GENERATION_CHANGE, source);
    retire(source);
  }

  if (!s.active)
  {
    s = slot{};
    s.active = true;
    s.generation = generation;
    s.first_seen_ms = now_ms;
    s.zones_mm.fill(INVALID_MM);
  }

  if (is_health)
  {
    const health_info incoming = parse_health(payload);
    if (s.health_seen)
    {
      if (normalised_equal(s.health, incoming))
      {
        emit(event::DUPLICATE_HEALTH_IDENTICAL, source);
        return std::nullopt;
      }
      // Same rule as a conflicting chunk: two different summaries of one grid give no
      // basis for choosing either.
      emit(event::CONFLICTING_HEALTH, source);
      retire(source);
      return std::nullopt;
    }
    s.health_seen = true;
    s.health = incoming;
    return try_complete(source, now_ms);
  }

  const uint8_t chunk = low_nibble;  // a nibble is always a valid chunk index
  const uint16_t zones[ZONES_PER_CHUNK] = {
    static_cast<uint16_t>((payload[2] << 4) | (payload[3] >> 4)),
    static_cast<uint16_t>(((payload[3] & 0x0F) << 8) | payload[4]),
    static_cast<uint16_t>((payload[5] << 4) | (payload[6] >> 4)),
    static_cast<uint16_t>(((payload[6] & 0x0F) << 8) | payload[7]),
  };

  const uint16_t bit = static_cast<uint16_t>(1u << chunk);
  const size_t base = static_cast<size_t>(chunk) * ZONES_PER_CHUNK;

  if (s.bitmap & bit)
  {
    bool identical = true;
    for (size_t j = 0; j < ZONES_PER_CHUNK; ++j)
    {
      if (s.zones_mm[base + j] != zones[j])
      {
        identical = false;
        break;
      }
    }
    if (identical)
    {
      emit(event::DUPLICATE_CHUNK_IDENTICAL, source);
      return std::nullopt;
    }
    // Two different payloads for one (source, generation, chunk) mean a firmware fault or
    // two streams mixing. There is no basis for choosing between them, so the generation
    // is retired outright rather than left in the slot to time out later, which would
    // both double-count and risk being revived.
    emit(event::CONFLICTING_CHUNK, source);
    retire(source);
    return std::nullopt;
  }

  s.bitmap = static_cast<uint16_t>(s.bitmap | bit);
  for (size_t j = 0; j < ZONES_PER_CHUNK; ++j)
    s.zones_mm[base + j] = zones[j];

  return try_complete(source, now_ms);
}

tof_grid_assembler::status tof_grid_assembler::source_status(uint8_t source, uint32_t now_ms) const
{
  status out;
  if (source >= SOURCE_COUNT)
    return out;

  const tracker& t = trackers_[source];
  out.alarm_active = t.alarm_active;
  out.ever_published = t.ever_published;
  out.since_last_frame_ms = t.ever_seen_frame ? elapsed(t.last_frame_ms, now_ms) : 0;
  out.since_last_publish_ms = t.ever_published ? elapsed(t.last_publish_ms, now_ms) : 0;

  if (!t.ever_seen_frame)
  {
    out.state = source_state::NEVER_SEEN;
    return out;
  }

  // A source that has never published is measured from the end of the startup grace, so a
  // sensor that simply takes a moment to warm up is not reported as a fault.
  const uint32_t reference = t.ever_published ? t.last_publish_ms : startup_time_ms_ + STARTUP_GRACE_MS;
  if (static_cast<int32_t>(now_ms - reference) <= static_cast<int32_t>(SOURCE_STALE_MS))
  {
    out.state = source_state::HEALTHY;
    return out;
  }

  out.state = elapsed(t.last_frame_ms, now_ms) <= SOURCE_STALE_MS ? source_state::STALE_NOT_COMPLETING :
                                                                    source_state::STALE_NO_FRAMES;
  return out;
}

void tof_grid_assembler::poll(uint32_t now_ms)
{
  expire_stale_slots(now_ms);

  for (uint8_t src = 0; src < SOURCE_COUNT; ++src)
  {
    tracker& t = trackers_[src];
    const source_state state = source_status(src, now_ms).state;

    if (state == source_state::HEALTHY)
    {
      if (t.alarm_active)
      {
        emit(event::SOURCE_RECOVERED, src, state);
        t.alarm_active = false;
      }
      continue;
    }

    if (t.alarm_active)
      continue;

    if (state == source_state::NEVER_SEEN)
    {
      // Before the grace expires this is simply a system still starting up, and alarming
      // on it would make the alarm meaningless at every boot.
      if (elapsed(startup_time_ms_, now_ms) > STARTUP_GRACE_MS)
      {
        emit(event::SOURCE_NEVER_SEEN, src, state);
        t.alarm_active = true;
      }
      continue;
    }

    emit(event::SOURCE_STALE, src, state);
    t.alarm_active = true;
  }
}

}  // namespace lexxhard
