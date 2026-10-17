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

import time
import pytest
from twister_harness import DeviceAdapter

TIMEOUT_SEC = 10.0


def test_tier2_mcu_publishes_multi_channel(
    zenoh_router, dut: DeviceAdapter, zenoh_multi_channel_client
):
    """TEST 1: MCU to Host Multi-Channel Raw Zenoh Publication."""
    # 1. Wait for MCU to enter loop
    dut.readlines_until(regex=r"\[SYS\] Entering Multi-Channel Event Loop", timeout=TIMEOUT_SEC)

    # 2. Verify Motor State (<dd>)
    motor_st = zenoh_multi_channel_client.motor_state_queue.get(timeout=TIMEOUT_SEC)
    assert "position" in motor_st and "velocity" in motor_st
    assert motor_st["position"] >= 0.0

    # 3. Verify Telemetry State (<ff>)
    telem = zenoh_multi_channel_client.telemetry_queue.get(timeout=TIMEOUT_SEC)
    assert "voltage" in telem and "temperature" in telem
    assert telem["voltage"] == pytest.approx(12.4, rel=1e-2)
    assert telem["temperature"] >= 25.0


def test_tier2_host_commands_multi_channel(
    zenoh_router, dut: DeviceAdapter, zenoh_multi_channel_client
):
    """TEST 2: Host to MCU Multi-Channel Commands."""
    dut.readlines_until(regex=r"\[SYS\] Entering Multi-Channel Event Loop", timeout=TIMEOUT_SEC)

    # Wait for router to match subscribers
    zenoh_multi_channel_client.wait_for_matching()

    # Publish commands periodically until acknowledged
    deadline = time.monotonic() + TIMEOUT_SEC
    while time.monotonic() < deadline:
        zenoh_multi_channel_client.publish_motor_cmd(4.5)
        zenoh_multi_channel_client.publish_mode_cmd(3, 0x1A)
        time.sleep(0.1)

    dut.readlines_until(
        regex=r"\[CB 1\] Motor Command received: effort=4\.50",
        timeout=TIMEOUT_SEC,
    )
    dut.readlines_until(
        regex=r"\[CB 2\] Mode Command received: mode=3 flags=0x1a",
        timeout=TIMEOUT_SEC,
    )


def test_tier2_bidirectional_roundtrip(
    zenoh_router, dut: DeviceAdapter, zenoh_multi_channel_client
):
    """TEST 3: Full Roundtrip State Loopback."""
    dut.readlines_until(regex=r"\[SYS\] Entering Multi-Channel Event Loop", timeout=TIMEOUT_SEC)

    zenoh_multi_channel_client.wait_for_matching()

    target_effort = 6.0
    expected_velocity = target_effort * 0.5

    deadline = time.monotonic() + TIMEOUT_SEC
    matched = False

    while time.monotonic() < deadline:
        zenoh_multi_channel_client.publish_motor_cmd(target_effort)
        time.sleep(0.05)

        while not zenoh_multi_channel_client.motor_state_queue.empty():
            sample = zenoh_multi_channel_client.motor_state_queue.get_nowait()
            if sample["velocity"] == pytest.approx(expected_velocity):
                matched = True
                break

        if matched:
            break

    assert matched, f"MCU did not echo velocity {expected_velocity} within {TIMEOUT_SEC}s"
