# Copyright (c) 2024-2026, LexxPluss Inc.
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

# Authoritative safety readiness of the cliff ToF path, computed by SCBDriver.
#
# This is NOT a mirror of the CAN health frame. The driver owns the readiness
# decision; the mirrored wire fields below exist only so an operator or a log can
# explain a decision after the fact.
#
# CONSUMER CONTRACT
#   - The cliff stop may be released ONLY while readiness == READINESS_READY.
#     NOT_READY, FAULT, an unknown readiness value, and no fresh message at all
#     must all keep the robot stopped. An all-zero or default-constructed message
#     is NOT_READY, so a consumer that fails to receive anything fails safe.
#   - reason_mask MUST NOT take part in the safety decision. It exists so the
#     reason can be shown, not so a consumer can rebuild the readiness rule.
#     Re-deriving readiness downstream would create a second, diverging copy of
#     the criteria that live in the CAN wire contract.
#   - Freshness is measured on the CONSUMER's monotonic clock against the arrival
#     of a NEW heartbeat_seq. A repeated heartbeat_seq must not refresh it, and
#     header.stamp must not be used for it: the stamp is for observation and log
#     correlation only. NEW means "differs from the previous value", never a
#     numeric increase, so the 32-bit wrap is an ordinary new snapshot.
#   - There is deliberately no DEGRADED readiness. A source that is temporarily
#     short of a sample is still READY, and says so through
#     degraded_source_mask. Splitting readiness further would invite a consumer
#     to invent its own policy for the middle state.
#
# READINESS MEANING
#   NOT_READY  initialisation or authorisation is not complete and no definite
#              fault has been found. Nothing is wrong yet; nothing is proven.
#   READY      every condition required by the wire contract holds now.
#   FAULT      a configuration error, a protocol error, a definite hardware
#              fault, a timeout, or the loss of a required condition after
#              having been READY.
#
# THE ABSENCE OF CAN HEALTH IS RESOLVED IN TIME, NOT LEFT PENDING
#   Inside the startup grace, no health frame at all is NOT_READY. Once the grace
#   expires it becomes FAULT, because a permanently broken link must not read as
#   "still starting up" forever. Health that arrived and then stopped is FAULT on
#   its own bound. A timeout fault is recomputed from current state and clears by
#   itself once conditions hold again; only a protocol fault latches.

uint8 READINESS_NOT_READY = 0
uint8 READINESS_READY = 1
uint8 READINESS_FAULT = 2

# reason_mask bits. Explanatory only; see the consumer contract above.
uint32 REASON_STARTUP = 1                       # within the startup grace, nothing received yet
uint32 REASON_CONFIG_ERROR = 2                  # missing required parameter, or cliff with legacy_uart
uint32 REASON_NO_CAN_HEALTH = 4                 # no CAN health frame has ever been received
uint32 REASON_HEALTH_STALE = 8                  # no new health_seq within the configured bound
uint32 REASON_PROTOCOL_VERSION_UNSUPPORTED = 16
uint32 REASON_PROTOCOL_FAULT = 32               # malformed frame or a contract contradiction, latched
uint32 REASON_CYCLE_INVALID = 64                # newest health carries no completed cycle
uint32 REASON_MAPPING_NOT_PROVEN = 128          # mapping_state is UNKNOWN, LOST or FAULT
uint32 REASON_EPOCH_UNAUTHORISED = 256          # epoch changed and its cycle is not yet complete
uint32 REASON_CHAIN_FAULT = 512                 # chain_flags bits 0-2
uint32 REASON_CHAIN_POSITION_IMPLICATED = 1024  # failing_chain_position is not CHAIN_POSITION_NONE
uint32 REASON_NOT_ENUMERATED = 2048             # enumerated_mask is not all four
uint32 REASON_MODEL_UNVERIFIED = 4096           # model_verified_mask is not all four
uint32 REASON_SENSOR_FAULT = 8192               # a source reported SENSOR_FAULT in the newest cycle
uint32 REASON_SOURCE_STALE = 16384              # a source exceeded its measurement age bound
uint32 REASON_SAMPLE_MISS_FAULT = 32768         # a source reached the consecutive missed-cycle fault

# Mirrors of the wire enum. The CAN wire contract is authoritative; these are for
# display and log correlation, and must be regenerated if the contract changes.
uint8 MAPPING_STATE_UNKNOWN = 0
uint8 MAPPING_STATE_PROVEN = 1
uint8 MAPPING_STATE_LOST = 2
uint8 MAPPING_STATE_FAULT = 3

# chain_flags bits, as carried in the health frame's low nibble.
uint8 CHAIN_FLAG_LENGTH_MISMATCH = 1
uint8 CHAIN_FLAG_ENUMERATION_FROZEN = 2
uint8 CHAIN_FLAG_BUS_FAULT = 4
uint8 CHAIN_FLAG_CYCLE_VALID = 8

# Number of cliff sources, and the "no position implicated" marker.
uint8 SOURCE_COUNT = 4
uint8 CHAIN_POSITION_NONE = 255

Header header

# Increments on every new snapshot this node publishes, wrapping at 2^32. A
# consumer decides "new" by inequality with the previous value, never by numeric
# comparison, so the wrap is not a regression. A repeated value carries no new
# information and must not refresh a consumer's freshness.
uint32 heartbeat_seq

# The decision, and why. Only readiness is safety-bearing.
uint8 readiness
uint32 reason_mask

# Mirrored from the newest accepted CAN health frame. While can_health_seen is
# false the fields below carry no meaning: a ROS message field cannot be unset, so
# they hold default zeros, and a consumer must not interpret those zeros as
# observations. Read them only when can_health_seen is true.
bool can_health_seen
uint8 protocol_version
uint8 mapping_state
uint8 mapping_epoch
uint8 health_seq
uint8 cycle_seq
uint8 chain_flags
uint8 enumerated_mask
uint8 model_verified_mask
uint8 sample_produced_mask
uint8 sensor_fault_mask
uint8 failing_chain_position

# Computed by the driver, indexed by source_id in bits 0-3.
#   fresh     accepted a measurement from a new cycle within its age bound
#   degraded  short of a sample in the newest cycle but still inside the bound
#   stale     exceeded the age bound
#   miss      reached the consecutive missed-cycle fault count
uint8 fresh_source_mask
uint8 degraded_source_mask
uint8 stale_source_mask
uint8 sample_miss_fault_mask

# Ages measured on the publisher's monotonic clock, in seconds, for observation
# only: the safety timeout is the consumer's own monotonic clock against a new
# heartbeat_seq, never these fields. Seconds rather than a duration type because
# nothing here needs duration arithmetic and a plain double is what a log or a
# plot wants. A negative value means never observed.
float64 health_age
float64[4] measurement_age
uint16[4] consecutive_missed_cycles
