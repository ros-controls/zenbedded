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

import zenoh
from twister_harness import DeviceAdapter

ROUTER_ENDPOINT = "tcp/127.0.0.1:7447"
INGRESS_TOPIC = "zenbedded/e2e/ingress"
EGRESS_TOPIC = "zenbedded/e2e/egress"
FIRMWARE_MESSAGE = "hello from the firmware"


def _wait_for_ready(dut: DeviceAdapter, ingress: bool, egress: bool):
    expected = (
        "Zenoh client connected "
        f"(ingress={'on' if ingress else 'off'}, egress={'on' if egress else 'off'})"
    )
    lines = dut.readlines_until(regex=r"Zenoh client connected", timeout=10)
    assert any(expected in line for line in lines), f"expected firmware ready line: {expected}"


def _open_session():
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([ROUTER_ENDPOINT]))
    return zenoh.open(config)


def test_ingress_message_is_printed(zenoh_router, dut: DeviceAdapter):
    message = "hello from the host"
    session = _open_session()
    try:
        _wait_for_ready(dut, ingress=True, egress=False)

        publisher = session.declare_publisher(INGRESS_TOPIC)

        # declare_publisher returns before the router has propagated the firmware's
        # subscription, so publishing immediately can drop the sample. MatchingStatus
        # defines no __bool__, so .matching must be read explicitly to get a real answer.
        deadline = time.monotonic() + 10
        while not publisher.matching_status.matching:
            assert time.monotonic() < deadline, "no matching subscriber within 10s"
            time.sleep(0.05)

        publisher.put(message)

        lines = dut.readlines_until(regex=r"Received test message:", timeout=10)
        assert any(f"Received test message: {message}" in line for line in lines)
    finally:
        session.close()


def test_egress_message_is_received(zenoh_router, dut: DeviceAdapter):
    session = _open_session()
    messages = queue.Queue()
    subscriber = None

    try:
        subscriber = session.declare_subscriber(
            EGRESS_TOPIC, lambda sample: messages.put(bytes(sample.payload))
        )
        _wait_for_ready(dut, ingress=False, egress=True)

        payload = messages.get(timeout=10)
        assert payload.decode("utf-8") == FIRMWARE_MESSAGE
    finally:
        if subscriber is not None:
            subscriber.undeclare()
        session.close()


def test_bidirectional_messages_are_exchanged(zenoh_router, dut: DeviceAdapter):
    message = "hello from the host"
    session = _open_session()
    messages = queue.Queue()
    publisher = None
    subscriber = None

    try:
        subscriber = session.declare_subscriber(
            EGRESS_TOPIC, lambda sample: messages.put(bytes(sample.payload))
        )
        _wait_for_ready(dut, ingress=True, egress=True)

        publisher = session.declare_publisher(INGRESS_TOPIC)

        deadline = time.monotonic() + 10
        while not publisher.matching_status.matching:
            assert time.monotonic() < deadline, "no matching subscriber within 10s"
            time.sleep(0.05)

        publisher.put(message)

        lines = dut.readlines_until(regex=r"Received test message:", timeout=10)
        assert any(f"Received test message: {message}" in line for line in lines)

        payload = messages.get(timeout=10)
        assert payload.decode("utf-8") == FIRMWARE_MESSAGE
    finally:
        if subscriber is not None:
            subscriber.undeclare()
        if publisher is not None:
            publisher.undeclare()
        session.close()
