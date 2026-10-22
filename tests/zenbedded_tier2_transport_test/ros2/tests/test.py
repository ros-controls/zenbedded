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

import zenoh
import struct
import time
import threading

# --- THREAD SYNCHRONIZATION ---
pong_received = threading.Event()
latest_state = (0.0, 0.0)

# --- DEBUG COUNTERS ---
total_callbacks_received = 0
last_callback_timestamp = 0.0


def state_callback(sample):
    global latest_state, total_callbacks_received, last_callback_timestamp
    try:
        payload_bytes = bytes(sample.payload)
        latest_state = struct.unpack("<dd", payload_bytes)

        total_callbacks_received += 1
        last_callback_timestamp = time.perf_counter()

        pong_received.set()

    except struct.error:
        pass


if __name__ == "__main__":
    conf = zenoh.Config()
    conf.insert_json5("mode", '"peer"')
    conf.insert_json5("listen/endpoints", '["udp/0.0.0.0:7447"]')

    print("[INFO] Zenbedded Tier 2 Ping-Pong Benchmark Server")
    z = zenoh.open(conf)

    pub = z.declare_publisher("test_motor/cmd")
    sub = z.declare_subscriber("test_motor/state", state_callback)

    try:
        effort = 0.0
        ping_count = 0
        total_rtt = 0.0
        consecutive_timeouts = 0
        last_report_time = time.time()

        print("\n[INFO] Sending initial command frame....")
        pong_received.clear()
        start_rtt = time.perf_counter()
        pub.put(struct.pack("<d", effort))
        effort += 0.1

        while True:
            wait_entry_time = time.perf_counter()

            if pong_received.wait(timeout=1.0):
                end_rtt = time.perf_counter()
                consecutive_timeouts = 0

                rtt = end_rtt - start_rtt
                total_rtt += rtt
                ping_count += 1

                pong_received.clear()

                start_rtt = time.perf_counter()
                pub.put(struct.pack("<d", effort))
                effort += 0.1

            else:
                wait_exit_time = time.perf_counter()
                consecutive_timeouts += 1
                actual_wait_duration_ms = (wait_exit_time - wait_entry_time) * 1000

                time_since_last_packet_ms = (
                    (wait_exit_time - last_callback_timestamp) * 1000
                    if last_callback_timestamp > 0
                    else float("inf")
                )

                print(f"\n [STALL DETECTED] Timeout Event #{consecutive_timeouts} Triggered!")
                print(
                    f"   ├─ Spent precisely {actual_wait_duration_ms:.3f}ms waiting inside the lock."
                )
                print(
                    f"   ├─ Time since the last data packet actually reached the host callback: {time_since_last_packet_ms:.3f}ms"
                )
                print(
                    f"   ├─ Current tracked position state register inside memory context: {latest_state[0]:.2f}"
                )
                print(
                    f"   └─ Total standalone packet frames handled by Python callback since boot: {total_callbacks_received}"
                )

                # RECOVERY / RESYNC PHASE
                print(
                    "   [RECOVERY] Injecting a fresh asynchronous command frame to kick-start loop alignment..."
                )
                pong_received.clear()
                start_rtt = time.perf_counter()
                pub.put(struct.pack("<d", effort))
                effort += 0.1

                time.sleep(0.1)
                continue

            current_time = time.time()
            elapsed = current_time - last_report_time

            if elapsed >= 1.0:
                closed_loop_hz = ping_count / elapsed
                avg_rtt_ms = (total_rtt / ping_count) * 1000 if ping_count > 0 else 0
                pos, vel = latest_state

                print(
                    f"[METRIC] Sync Speed: {closed_loop_hz:.2f} Hz | Avg Latency (RTT): {avg_rtt_ms:.3f} ms | Pos: {pos:.2f} | Total Frames: {total_callbacks_received}"
                )

                ping_count = 0
                total_rtt = 0.0
                last_report_time = current_time

    except KeyboardInterrupt:
        print("\nShutting down Host Peer.")
    finally:
        z.close()
