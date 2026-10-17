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
import os
import queue
import signal
import struct
import subprocess
import time

import pytest
import zenoh

ROUTER_ENDPOINT = "tcp/127.0.0.1:7447"
ROUTER_READY_TIMEOUT = 30
MATCHING_TIMEOUT_SEC = 10.0


def _stop_process_group(process):
    if process is None or process.poll() is not None:
        return

    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass

    os.killpg(process.pid, signal.SIGKILL)
    process.wait()


def _router_is_reachable(endpoint):
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", json.dumps([endpoint]))

    session = zenoh.open(conf)
    try:
        return len(session.info.routers_zid()) > 0
    finally:
        session.close()


def _wait_until_router_ready(process, log_path):
    deadline = time.monotonic() + ROUTER_READY_TIMEOUT
    last_error = None

    while time.monotonic() < deadline:
        if process.poll() is not None:
            pytest.fail(
                f"rmw_zenohd exited with code {process.returncode}:\n{log_path.read_text()}"
            )

        try:
            if _router_is_reachable(ROUTER_ENDPOINT):
                return
        except Exception as error:
            last_error = error

        time.sleep(0.1)

    pytest.fail(
        f"could not open a zenoh session to {ROUTER_ENDPOINT} within "
        f"{ROUTER_READY_TIMEOUT} seconds (last error: {last_error}):\n{log_path.read_text()}"
    )


@pytest.fixture(scope="session")
def zenoh_router(tmp_path_factory):
    log_path = tmp_path_factory.mktemp("zenoh") / "rmw_zenohd.log"

    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        try:
            _wait_until_router_ready(process, log_path)
            yield process
        finally:
            _stop_process_group(process)


class ZenohMultiChannelClient:
    def __init__(self, session):
        self.session = session
        self.motor_state_queue = queue.Queue()
        self.telemetry_queue = queue.Queue()

        self.sub_motor = session.declare_subscriber("robot/motor/state", self._on_motor_sample)
        self.sub_telem = session.declare_subscriber("robot/telemetry", self._on_telem_sample)

        self.pub_motor = session.declare_publisher("robot/motor/cmd")
        self.pub_mode = session.declare_publisher("robot/mode/cmd")

    def _on_motor_sample(self, sample):
        payload_bytes = bytes(sample.payload)
        if len(payload_bytes) == struct.calcsize("<dd"):
            pos, vel = struct.unpack("<dd", payload_bytes)
            self.motor_state_queue.put({"position": pos, "velocity": vel})

    def _on_telem_sample(self, sample):
        payload_bytes = bytes(sample.payload)
        if len(payload_bytes) == struct.calcsize("<ff"):
            volt, temp = struct.unpack("<ff", payload_bytes)
            self.telemetry_queue.put({"voltage": volt, "temperature": temp})

    def wait_for_matching(self, timeout=MATCHING_TIMEOUT_SEC):
        deadline = time.monotonic() + timeout
        while not (
            self.pub_motor.matching_status.matching and self.pub_mode.matching_status.matching
        ):
            if time.monotonic() > deadline:
                break
            time.sleep(0.05)

    def publish_motor_cmd(self, effort: float):
        self.pub_motor.put(struct.pack("<d", float(effort)))

    def publish_mode_cmd(self, mode_id: int, flags: int):
        self.pub_mode.put(struct.pack("<ii", int(mode_id), int(flags)))

    def close(self):
        self.sub_motor.undeclare()
        self.sub_telem.undeclare()
        self.pub_motor.undeclare()
        self.pub_mode.undeclare()


@pytest.fixture
def zenoh_multi_channel_client(zenoh_router):
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')

    session = zenoh.open(conf)
    client = ZenohMultiChannelClient(session)

    try:
        yield client
    finally:
        client.close()
        session.close()
