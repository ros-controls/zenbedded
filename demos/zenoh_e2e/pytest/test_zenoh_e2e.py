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

import json
import queue
import time

import pytest
import zenoh
from twister_harness import DeviceAdapter

ROUTER_ENDPOINT = "tcp/127.0.0.1:7447"
INGRESS_TOPIC = "zenbedded/e2e/ingress"
EGRESS_TOPIC = "zenbedded/e2e/egress"
HOST_MESSAGE_PREFIX = "hello from the host: "
FIRMWARE_MESSAGE_PREFIX = "hello from the firmware: "
READY_TIMEOUT_SEC = 10
MATCHING_TIMEOUT_SEC = 10
FREQUENT_PERIOD_MS = 1000
INFREQUENT_PERIOD_MS = 15000
FREQUENT_MESSAGE_COUNT = 3
INFREQUENT_MESSAGE_COUNT = 2
LEASE_TIMEOUT_SEC = 10
INGRESS_IDLE_INTERVAL_SEC = LEASE_TIMEOUT_SEC + 1
IDLE_10HZ_PERIOD_MS = 100
ISSUE_65_OBSERVATION_SEC = 10 * LEASE_TIMEOUT_SEC + 5
ISSUE_65_MINIMUM_MESSAGES = 800
ISSUE_65_MAX_FINAL_SILENCE_SEC = 2


def _wait_for_ready(dut: DeviceAdapter, ingress: bool, egress: bool, period_ms: int):
    expected = (
        "Zenoh client connected "
        f"(ingress={'on' if ingress else 'off'}, "
        f"egress={'on' if egress else 'off'}, period_ms={period_ms})"
    )
    lines = dut.readlines_until(regex=r"Zenoh client connected", timeout=READY_TIMEOUT_SEC)
    assert any(expected in line for line in lines), f"expected firmware ready line: {expected}"


def _open_session():
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([ROUTER_ENDPOINT]))
    return zenoh.open(config)


def _wait_for_matching_subscriber(publisher):
    deadline = time.monotonic() + MATCHING_TIMEOUT_SEC
    while not publisher.matching_status.matching:
        assert (
            time.monotonic() < deadline
        ), f"no matching firmware subscriber within {MATCHING_TIMEOUT_SEC}s"
        time.sleep(0.05)


def _publish_host_message(publisher, dut: DeviceAdapter, sequence: int):
    message = f"{HOST_MESSAGE_PREFIX}{sequence}"
    publisher.put(message)

    lines = dut.readlines_until(regex=r"Received test message:", timeout=READY_TIMEOUT_SEC)
    assert any(f"Received test message: {message}" in line for line in lines)


def _publish_host_messages(publisher, dut: DeviceAdapter, period_ms: int, message_count: int):
    for sequence in range(1, message_count + 1):
        _publish_host_message(publisher, dut, sequence)

        if sequence < message_count:
            time.sleep(period_ms / 1000)


def _collect_firmware_messages(messages: queue.Queue, period_ms: int, message_count: int):
    period_sec = period_ms / 1000
    deadline = time.monotonic() + MATCHING_TIMEOUT_SEC + message_count * period_sec + 5
    sequences = []

    while len(sequences) < message_count:
        remaining = deadline - time.monotonic()
        assert remaining > 0, f"received only {len(sequences)} firmware messages: {sequences}"

        try:
            payload = messages.get(timeout=min(remaining, 0.5))
        except queue.Empty:
            continue

        message = payload.decode("utf-8")
        assert message.startswith(
            FIRMWARE_MESSAGE_PREFIX
        ), f"unexpected firmware message: {message}"

        sequence_text = message.removeprefix(FIRMWARE_MESSAGE_PREFIX)
        assert sequence_text.isdecimal(), f"missing firmware sequence number: {message}"

        sequence = int(sequence_text)
        if sequence not in sequences:
            sequences.append(sequence)

    assert sequences == sorted(sequences), f"firmware messages arrived out of order: {sequences}"


def _run_ingress_test(dut: DeviceAdapter, period_ms: int, message_count: int):
    session = _open_session()
    publisher = None

    try:
        _wait_for_ready(dut, ingress=True, egress=False, period_ms=period_ms)
        publisher = session.declare_publisher(INGRESS_TOPIC)
        _wait_for_matching_subscriber(publisher)
        _publish_host_messages(publisher, dut, period_ms, message_count)
    finally:
        if publisher is not None:
            publisher.undeclare()
        session.close()


def _run_ingress_after_lease_idle_test(dut: DeviceAdapter):
    session = _open_session()
    publisher = None

    try:
        _wait_for_ready(dut, ingress=True, egress=False, period_ms=FREQUENT_PERIOD_MS)
        publisher = session.declare_publisher(INGRESS_TOPIC)
        _wait_for_matching_subscriber(publisher)

        _publish_host_message(publisher, dut, 1)
        time.sleep(INGRESS_IDLE_INTERVAL_SEC)
        _publish_host_message(publisher, dut, 2)
    finally:
        if publisher is not None:
            publisher.undeclare()
        session.close()


def _run_egress_test(dut: DeviceAdapter, period_ms: int, message_count: int):
    session = _open_session()
    messages = queue.Queue()
    subscriber = None

    try:
        subscriber = session.declare_subscriber(
            EGRESS_TOPIC, lambda sample: messages.put(bytes(sample.payload))
        )
        _wait_for_ready(dut, ingress=False, egress=True, period_ms=period_ms)
        _collect_firmware_messages(messages, period_ms, message_count)
    finally:
        if subscriber is not None:
            subscriber.undeclare()
        session.close()


def _run_issue_65_test(dut: DeviceAdapter):
    session = _open_session()
    messages = queue.Queue()
    subscriber = None

    try:
        subscriber = session.declare_subscriber(
            EGRESS_TOPIC, lambda sample: messages.put(bytes(sample.payload))
        )
        _wait_for_ready(dut, ingress=True, egress=True, period_ms=IDLE_10HZ_PERIOD_MS)

        start = time.monotonic()
        deadline = start + ISSUE_65_OBSERVATION_SEC
        sequences = set()
        last_message_at = start
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                payload = messages.get(timeout=min(remaining, 0.5))
            except queue.Empty:
                continue

            message = payload.decode("utf-8")
            assert message.startswith(
                FIRMWARE_MESSAGE_PREFIX
            ), f"unexpected firmware message: {message}"
            sequence_text = message.removeprefix(FIRMWARE_MESSAGE_PREFIX)
            assert sequence_text.isdecimal(), f"missing firmware sequence number: {message}"
            sequences.add(int(sequence_text))
            last_message_at = time.monotonic()

        elapsed = time.monotonic() - start
        assert elapsed >= 2 * LEASE_TIMEOUT_SEC, f"test ran for only {elapsed:.2f}s"
        assert (
            len(sequences) >= ISSUE_65_MINIMUM_MESSAGES
        ), f"received only {len(sequences)} firmware messages in {elapsed:.2f}s"
        final_silence = time.monotonic() - last_message_at
        assert final_silence <= ISSUE_65_MAX_FINAL_SILENCE_SEC, (
            f"firmware stopped publishing {final_silence:.2f}s before the end "
            f"of the {elapsed:.2f}s observation"
        )
    finally:
        if subscriber is not None:
            subscriber.undeclare()
        session.close()


def _run_bidirectional_test(dut: DeviceAdapter, period_ms: int, message_count: int):
    session = _open_session()
    messages = queue.Queue()
    publisher = None
    subscriber = None

    try:
        subscriber = session.declare_subscriber(
            EGRESS_TOPIC, lambda sample: messages.put(bytes(sample.payload))
        )
        _wait_for_ready(dut, ingress=True, egress=True, period_ms=period_ms)
        publisher = session.declare_publisher(INGRESS_TOPIC)
        _wait_for_matching_subscriber(publisher)
        _publish_host_messages(publisher, dut, period_ms, message_count)
        _collect_firmware_messages(messages, period_ms, message_count)
    finally:
        if subscriber is not None:
            subscriber.undeclare()
        if publisher is not None:
            publisher.undeclare()
        session.close()


def test_ingress_frequent(zenoh_router, dut: DeviceAdapter):
    _run_ingress_test(dut, FREQUENT_PERIOD_MS, FREQUENT_MESSAGE_COUNT)


def test_ingress_infrequent(zenoh_router, dut: DeviceAdapter):
    _run_ingress_test(dut, INFREQUENT_PERIOD_MS, INFREQUENT_MESSAGE_COUNT)


def test_ingress_after_lease_idle(zenoh_router, dut: DeviceAdapter):
    _run_ingress_after_lease_idle_test(dut)


def test_egress_frequent(zenoh_router, dut: DeviceAdapter):
    _run_egress_test(dut, FREQUENT_PERIOD_MS, FREQUENT_MESSAGE_COUNT)


def test_egress_infrequent(zenoh_router, dut: DeviceAdapter):
    _run_egress_test(dut, INFREQUENT_PERIOD_MS, INFREQUENT_MESSAGE_COUNT)


@pytest.mark.xfail(reason="Issue #65")
def test_issue_65_outbound_only_reconnect(zenoh_router, dut: DeviceAdapter):
    _run_issue_65_test(dut)


def test_bidirectional_frequent(zenoh_router, dut: DeviceAdapter):
    _run_bidirectional_test(dut, FREQUENT_PERIOD_MS, FREQUENT_MESSAGE_COUNT)


def test_bidirectional_infrequent(zenoh_router, dut: DeviceAdapter):
    _run_bidirectional_test(dut, INFREQUENT_PERIOD_MS, INFREQUENT_MESSAGE_COUNT)
