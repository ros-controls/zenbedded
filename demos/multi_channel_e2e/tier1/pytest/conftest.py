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
import subprocess
import threading
import time

import pytest
import rclpy
from control_msgs.msg import JointCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
import zenoh

ROUTER_ENDPOINT = "tcp/127.0.0.1:7447"
ROUTER_READY_TIMEOUT = 30


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

    try:
        session = zenoh.open(conf)
        try:
            return len(session.info.routers_zid()) > 0
        finally:
            session.close()
    except Exception:
        return False


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


class MultiChannelHostNode(Node):
    def __init__(self):
        super().__init__("multi_channel_host_node")
        self.arm_state_queue = queue.Queue()
        self.gripper_state_queue = queue.Queue()

        self.sub_arm = self.create_subscription(
            JointState,
            "/robot/arm/joint_states",
            lambda msg: self.arm_state_queue.put(msg),
            10,
        )
        self.sub_gripper = self.create_subscription(
            JointState,
            "/robot/gripper/joint_states",
            lambda msg: self.gripper_state_queue.put(msg),
            10,
        )

        self.pub_arm_cmd = self.create_publisher(JointCommand, "/robot/arm/joint_cmd", 10)
        self.pub_gripper_cmd = self.create_publisher(JointCommand, "/robot/gripper/joint_cmd", 10)

    def publish_arm_cmd(self, joint_names, interface_name, values):
        cmd = JointCommand(
            joint_names=joint_names,
            interface_name=interface_name,
            values=values,
        )
        cmd.header.frame_id = "arm_base"
        self.pub_arm_cmd.publish(cmd)

    def publish_gripper_cmd(self, joint_names, interface_name, values):
        cmd = JointCommand(
            joint_names=joint_names,
            interface_name=interface_name,
            values=values,
        )
        cmd.header.frame_id = "gripper_base"
        self.pub_gripper_cmd.publish(cmd)


class MultiChannelHostContext:
    def __enter__(self):
        rclpy.init()
        self.node = MultiChannelHostNode()
        self.executor_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.executor_thread.start()
        return self.node

    def __exit__(self, exc_type, exc_val, exc_tb):
        rclpy.shutdown()
        self.executor_thread.join(timeout=1.0)


@pytest.fixture
def host_context(zenoh_router):
    return MultiChannelHostContext
