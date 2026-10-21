# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import struct
import time

import pytest
import zenoh
from twister_harness import DeviceAdapter

TIMEOUT_SEC = 5.0
MATCHING_TIMEOUT_SEC = 10.0
COMMAND_EFFORT = 4.0
STATE_FORMAT = "<dd"


def test_tier2_ping_pong(zenoh_router, dut: DeviceAdapter):
    """TEST: Tier 2 Raw Zenoh Ping-Pong E2E Communication."""
    # 1. BOOT MCU FIRST: Let it claim the network ports and enter the loop
    dut.readlines_until(regex=r"\[SYS\] Entering Main Event Loop", timeout=TIMEOUT_SEC)

    # 2. Initialize Zenoh Python session pointing to the local test router
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')

    z = zenoh.open(conf)

    latest_state = None
    bad_payload_size = None

    def state_callback(sample):
        nonlocal latest_state, bad_payload_size
        payload_bytes = bytes(sample.payload)
        try:
            latest_state = struct.unpack(STATE_FORMAT, payload_bytes)
        except struct.error:
            bad_payload_size = len(payload_bytes)

    pub = z.declare_publisher("test_motor/cmd")
    sub = z.declare_subscriber("test_motor/state", state_callback)

    try:
        # declare_publisher returns before the router has propagated the firmware's
        # subscription, so publishing immediately can drop the sample. MatchingStatus
        # defines no __bool__, so .matching must be read explicitly to get a real answer.
        deadline = time.monotonic() + MATCHING_TIMEOUT_SEC
        while not pub.matching_status.matching:
            assert (
                time.monotonic() < deadline
            ), f"no matching subscriber within {MATCHING_TIMEOUT_SEC}s"
            time.sleep(0.05)

        pub.put(struct.pack("<d", COMMAND_EFFORT))

        # The firmware publishes state at 100Hz whether or not a command ever lands,
        # so the command round-trip is only proven by the echoed velocity.
        expected_velocity = COMMAND_EFFORT * 0.5
        deadline = time.monotonic() + TIMEOUT_SEC
        while True:
            state = latest_state
            if state is not None and state[1] == pytest.approx(expected_velocity):
                break

            assert bad_payload_size is None, (
                f"state payload was {bad_payload_size} bytes, "
                f"expected {struct.calcsize(STATE_FORMAT)}"
            )
            assert time.monotonic() < deadline, (
                f"MCU did not echo effort {COMMAND_EFFORT} as velocity "
                f"{expected_velocity} within {TIMEOUT_SEC}s; last state={state}"
            )
            time.sleep(0.05)

        assert state[0] > 0.0, f"position never advanced: {state}"

    finally:
        sub.undeclare()
        pub.undeclare()
        z.close()
