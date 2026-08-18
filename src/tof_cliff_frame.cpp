// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "tof_cliff_frame.hpp"

namespace tof_cliff_frame {

namespace {

namespace ctr = tof_cliff_contract;

const ctr::status_row* classify(uint8_t raw)
{
  for (std::size_t i = 0; i < ctr::kStatusRowCount; ++i)
    if (ctr::kStatusTable[i].raw == raw) return &ctr::kStatusTable[i];
  return nullptr;
}

}  // namespace

arrival classify_identifier(uint32_t can_id)
{
  if (can_id == ctr::kMeasId) return arrival::measurement;
  if (can_id == ctr::kHealthId) return arrival::health;
  return arrival::not_ours;
}

verdict decode_measurement(uint8_t dlc, const uint8_t* data, measurement& out)
{
  // Length first: everything below indexes into the payload.
  if (dlc != ctr::kDlc) return verdict::dlc_not_8;
  if (data == nullptr) return verdict::dlc_not_8;

  // frame_type exists even though the identifiers are distinct, because a mis-routed
  // filter is otherwise a silent mis-decode rather than a rejection.
  if (static_cast<uint8_t>(data[0] >> 4) != ctr::kMeasFrameType)
    return verdict::frame_type_mismatch;

  const uint8_t source_id = static_cast<uint8_t>(data[0] & 0x0F);
  if (source_id >= ctr::kSourceCount) return verdict::source_id_out_of_range;

  // Byte 7 is reserved for a future capture tick. Using it is a version bump, so a
  // non-zero value here is either a newer producer or a defect -- either way not
  // decodable under this version.
  if (data[7] != 0) return verdict::reserved_field_nonzero;

  const uint8_t target_count = data[6];
  if (target_count > ctr::kSourceCount) return verdict::target_count_malformed;

  const uint8_t raw_status = data[5];
  const ctr::status_row* row = classify(raw_status);
  // A status with no row cannot be reduced to a safety outcome, so there is nothing
  // safe to do with the range that accompanies it.
  if (row == nullptr) return verdict::status_undefined;
  // The two NO_SAMPLE statuses are never transmitted. A frame carrying one is a
  // producer defect, not a sample that happens to be unusable.
  if (row->cls == status_class::no_sample) return verdict::status_not_transmissible;

  const uint16_t range_mm = static_cast<uint16_t>((data[3] << 8) | data[4]);
  const bool is_sentinel = (range_mm == ctr::kSentinelInvalid);
  if (row->cls == status_class::valid_range && is_sentinel)
    return verdict::range_contradicts_status;
  if (row->cls != status_class::valid_range && !is_sentinel)
    return verdict::range_contradicts_status;

  // target_count == 0 if and only if the frame carries status 255 and the sentinel.
  // Checked in both directions, because either half alone would let one of the two
  // no-target encodings through.
  const bool zero_targets = (target_count == 0);
  const bool encoded_none = (raw_status == 255 && is_sentinel);
  if (zero_targets != encoded_none) return verdict::no_target_encoding_inconsistent;

  out.source_id = source_id;
  out.mapping_epoch = data[1];
  out.cycle_seq = data[2];
  out.range_mm = range_mm;
  out.raw_status = raw_status;
  out.target_count = target_count;
  out.cls = row->cls;
  out.no_target = is_sentinel;
  return verdict::accept;
}

verdict decode_health(uint8_t dlc, const uint8_t* data, health& out)
{
  if (dlc != ctr::kDlc) return verdict::dlc_not_8;
  if (data == nullptr) return verdict::dlc_not_8;

  if (static_cast<uint8_t>(data[0] >> 4) != ctr::kHealthFrameType)
    return verdict::frame_type_mismatch;

  const uint8_t protocol_version = static_cast<uint8_t>(data[0] & 0x0F);
  // Zero is not a valid version, so an all-zero byte cannot pass as one. Separated from
  // "unsupported" because an all-zero payload usually means a different bug than a
  // producer from a later contract revision.
  if (protocol_version == 0) return verdict::protocol_version_zero;
  if (protocol_version != ctr::kProtocolVersion) return verdict::protocol_version_unsupported;

  const uint8_t mapping_state = static_cast<uint8_t>(data[3] >> 4);
  if (mapping_state > 0x3) return verdict::mapping_state_malformed;

  const uint8_t failing_chain_position = data[6];
  if (failing_chain_position != ctr::kChainPositionNone &&
      !(failing_chain_position >= 1 && failing_chain_position <= 6))
    return verdict::chain_position_malformed;

  const uint8_t flags = static_cast<uint8_t>(data[3] & 0x0F);
  const uint8_t enumerated_mask = static_cast<uint8_t>(data[4] >> 4);
  const uint8_t model_verified_mask = static_cast<uint8_t>(data[4] & 0x0F);
  const uint8_t sample_produced_mask = static_cast<uint8_t>(data[5] >> 4);
  const uint8_t sensor_fault_mask = static_cast<uint8_t>(data[5] & 0x0F);
  const bool cycle_valid = (flags & ctr::kCycleValidBit) != 0;

  // With cycle_valid clear the frame describes no cycle, so both per-cycle fields must
  // be empty. sensor_fault_mask is per cycle too, which is easy to overlook.
  if (!cycle_valid && (data[7] != 0 || sample_produced_mask != 0 || sensor_fault_mask != 0))
    return verdict::cycle_fields_inconsistent;

  // sensor_fault_mask classifies a produced sample, so a fault bit outside the produced
  // mask classifies a sample that does not exist.
  if ((sensor_fault_mask & static_cast<uint8_t>(~sample_produced_mask) & 0x0F) != 0)
    return verdict::mask_fault_without_sample;

  // The contract's rule is compound: naming a position is contradictory only with no
  // chain fault AND complete masks. An incomplete enumeration is itself a reason to
  // name one.
  const bool chain_fault = (flags & ctr::kChainFaultBits) != 0;
  if (failing_chain_position != ctr::kChainPositionNone && !chain_fault &&
      enumerated_mask == 0x0F && model_verified_mask == 0x0F)
    return verdict::chain_position_without_fault;

  out.protocol_version = protocol_version;
  out.mapping_epoch = data[1];
  out.health_seq = data[2];
  out.mapping_state = mapping_state;
  out.flags = flags;
  out.enumerated_mask = enumerated_mask;
  out.model_verified_mask = model_verified_mask;
  out.sample_produced_mask = sample_produced_mask;
  out.sensor_fault_mask = sensor_fault_mask;
  out.failing_chain_position = failing_chain_position;
  out.cycle_seq = data[7];
  out.cycle_valid = cycle_valid;
  out.chain_fault = chain_fault;
  return verdict::accept;
}

}  // namespace tof_cliff_frame
