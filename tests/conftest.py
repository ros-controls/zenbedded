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
import re
import signal
import subprocess
import time

import pytest
import zenoh

ROUTER_READY_TIMEOUT = 30


def _stop_process_group(process):
    if process.poll() is not None:
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


def _extract_router_endpoint(process, log_path):
    deadline = time.monotonic() + ROUTER_READY_TIMEOUT
    with log_path.open("r") as log_file:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                pytest.fail(
                    f"rmw_zenohd exited with code {process.returncode}:\n{log_path.read_text()}"
                )
            line = log_file.readline()
            if not line:
                time.sleep(0.02)
                continue
            match = re.search(r"Zenoh can be reached at: tcp/127\.0\.0\.1:(\d+)", line)
            if match:
                port = match.group(1)
                return f"tcp/127.0.0.1:{port}", port

    pytest.fail(
        f"Failed to extract assigned port from router within {ROUTER_READY_TIMEOUT}s:\n"
        f"{log_path.read_text()}"
    )


@pytest.fixture(scope="session")
def zenoh_router(tmp_path_factory):
    router_dir = tmp_path_factory.mktemp("zenoh")
    router_cfg_path = router_dir / "router_config.json5"
    router_cfg_path.write_text(
        "{\n"
        '  mode: "router",\n'
        "  listen: {\n"
        '    endpoints: ["tcp/127.0.0.1:0"]\n'
        "  },\n"
        "  scouting: {\n"
        "    multicast: { enabled: false },\n"
        "    gossip: { enabled: false }\n"
        "  }\n"
        "}\n"
    )
    log_path = router_dir / "rmw_zenohd.log"

    env = os.environ.copy()
    env["ZENOH_ROUTER_CONFIG_URI"] = str(router_cfg_path)

    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        try:
            endpoint, port = _extract_router_endpoint(process, log_path)

            session_cfg_path = router_dir / "session_config.json5"
            session_cfg_path.write_text(
                f'{{\n  mode: "client",\n  connect: {{\n    endpoints: ["{endpoint}"]\n  }}\n}}\n'
            )

            os.environ["ZENOH_SESSION_CONFIG_URI"] = str(session_cfg_path)
            os.environ["ZENBEDDED_ZENOH_PORT"] = port
            os.environ["ZENBEDDED_ZENOH_ENDPOINT"] = endpoint

            # Wait until router is responsive
            deadline = time.monotonic() + ROUTER_READY_TIMEOUT
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    pytest.fail(f"rmw_zenohd exited:\n{log_path.read_text()}")
                if _router_is_reachable(endpoint):
                    break
                time.sleep(0.05)
            else:
                pytest.fail(f"Could not connect to {endpoint}:\n{log_path.read_text()}")

            yield endpoint
        finally:
            _stop_process_group(process)
            os.environ.pop("ZENOH_SESSION_CONFIG_URI", None)
            os.environ.pop("ZENBEDDED_ZENOH_PORT", None)
            os.environ.pop("ZENBEDDED_ZENOH_ENDPOINT", None)


@pytest.fixture(autouse=True)
def _sync_dut_env(zenoh_router, request):
    """Ensure DUT (zephyr.exe) process inherits dynamic Zenoh port in process_kwargs."""
    if "device_object" in request.fixturenames:
        device_object = request.getfixturevalue("device_object")
        if hasattr(device_object, "process_kwargs"):
            if (
                "env" in device_object.process_kwargs
                and device_object.process_kwargs["env"] is not None
            ):
                device_object.process_kwargs["env"]["ZENBEDDED_ZENOH_PORT"] = os.environ.get(
                    "ZENBEDDED_ZENOH_PORT", ""
                )
                device_object.process_kwargs["env"]["ZENBEDDED_ZENOH_ENDPOINT"] = os.environ.get(
                    "ZENBEDDED_ZENOH_ENDPOINT", ""
                )
                device_object.process_kwargs["env"]["ZENOH_SESSION_CONFIG_URI"] = os.environ.get(
                    "ZENOH_SESSION_CONFIG_URI", ""
                )
