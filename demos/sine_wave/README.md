# Zenbedded Sine Wave Demo

A complete end-to-end demo demonstrating both **Tier 1** (native ROS 2 CDR serialization) and **Tier 2** (high-throughput packed numeric schema for `ros2_control`) using Zenbedded on Zephyr RTOS.

The firmware generates a sinusoidal wave signal (`position = amplitude * sin(2 * pi * f * t)`) and dynamically accepts command messages to adjust the wave amplitude in real time.

```
┌─────────────────────────────────┐                       ┌──────────────────────────────────────────────┐
│ ESP32-S3 (Zephyr)               │   Zenoh (tcp/7447)    │ Host PC (ROS 2)                              │
│                                 │ <───────────────────> │                                              │
│ - sine_wave_tier1 (ROS 2 CDR)   │  Wi-Fi / USB CDC ECM  │ • rqt_plot / ros2 topic echo                 │
│ - sine_wave_tier2 (No-CDR POD)  │                       │ • zenbedded_hardware_interface (ros2_control)│
└─────────────────────────────────┘                       └──────────────────────────────────────────────┘
```

---

## Architecture Variants

- **Tier 1 (`sine_wave_tier1`)**:
  - Implements standard ROS 2 messages using `JointStateCodec` (`sensor_msgs/msg/JointState`) and `JointCommandCodec` (`control_msgs/msg/JointCommand`).
  - Directly compatible with all standard ROS 2 nodes, CLI commands (`ros2 topic echo`), and visualization tools (`rqt_plot`) without requiring a custom hardware interface on the host PC.
  - Paired with the host-side `sine_wave_tier1` (`SineWaveTier1Node`) C++ commander running at 50 Hz.

- **Tier 2 (`sine_wave_tier2`)**:
  - Uses zero-copy packed numeric C structs generated from `interface_schema.yaml`.
  - Interfaces directly with `zenbedded_hardware_interface` for seamless `ros2_control` integration.
  - Paired with the host-side `sine_wave_tier2` (`SineWaveTier2Controller`) controller plugin running at 50 Hz.

---

## Hardware Support

| Board | Target Qualifier | Supported Transports |
|---|---|---|
| **ESP32-S3 DevKitC** | `esp32s3_devkitc/esp32s3/procpu` | USB CDC ECM (`-S cdc-ecm`), Wi-Fi (`-S wifi`), or Dual Transport (`-S cdc-ecm -S wifi`) |

---

## Step 1: Start the Zenoh Router

On your Host PC, start the Zenoh router. If using `rmw_zenoh_cpp`:

```bash
export ZENOH_CONFIG_OVERRIDE='mode="peer";listen/endpoints=["tcp/0.0.0.0:7447"]'
export RUST_LOG=zenoh=debug
ros2 run rmw_zenoh_cpp rmw_zenohd
```

*(Alternatively, run the standalone router: `zenohd --listen tcp/0.0.0.0:7447`).*

---

## Step 2: Configure Firmware Target Locator

The firmware defaults to `127.0.0.1:7447` in `prj.conf`.
- **USB CDC ECM**: Uses static peer IP `192.0.2.1:7447` automatically when the USB carrier is active.
- **Wi-Fi**: Requires your host PC's IP address on the local network (find via `ip addr` on Linux or `ipconfig` on Windows).

To avoid committing personal network IPs to version control, supply the host IP via an `extra.conf` file:

```ini
CONFIG_ZENBEDDED_ZENOH_LOCATOR="192.168.1.50:7447"
```

And pass `-- -DEXTRA_CONF_FILE=extra.conf` when building. *(Alternatively, edit `prj.conf` directly for local testing)*.

---

## Step 3: Building and Flashing (Firmware)

### For ESP32-S3 DevKitC

```bash
# Option A: Wired USB CDC ECM (Ethernet over USB)
west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -S cdc-ecm
west flash

# Option B: Wireless Wi-Fi
west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -S wifi -- -DEXTRA_CONF_FILE=extra.conf
west flash

# Option C: Dual Transport (USB CDC ECM prioritized + Wi-Fi fallback)
west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -S cdc-ecm -S wifi -- -DEXTRA_CONF_FILE=extra.conf
west flash
```

*(For Tier 2, replace `sine_wave_tier1` with `sine_wave_tier2` in the build commands above).*

---

## Step 4: Configure Wi-Fi Credentials (Wi-Fi Builds Only)

Open a serial monitor (e.g., `tio /dev/ttyACM1` or `picocom -b 115200 /dev/ttyACM1`) to access the Zephyr shell:

```text
wifi cred add -s "<MY_WIFI_SSID>" -k 1 -p "<MY_WIFI_PASSWORD>"
wifi cred auto_connect on
```

Credentials are saved in non-volatile flash storage (NVS) and persist across power cycles.

---

## Step 5: Host Control & Real-Time Visualization

Build the host ROS 2 package:

```bash
cd <ros2_ws>
colcon build --packages-select zenbedded_sine_wave zenbedded_hardware_interface zenbedded_transport
source install/setup.bash
```

### 1. Launching Tier 1 (Native ROS 2 CDR)

Launch the Tier 1 commander and live visualization:

```bash
ros2 launch zenbedded_sine_wave sine_wave.launch.py tier:=1
# Or shorthand:
ros2 launch zenbedded_sine_wave sine_wave_tier1.launch.py
```

This starts:
- `sine_wave_tier1` node streaming oscillating amplitude commands at 50 Hz.
- `rqt_plot` automatically graphing:
  - `/zenbedded/sine_wave/state/position[0]` (firmware sine wave)
  - `/zenbedded/sine_wave/cmd/values[0]` (commanded amplitude)

### 2. Launching Tier 2 (`ros2_control`)

Launch the Tier 2 controller stack and hardware introspection plot:

```bash
ros2 launch zenbedded_sine_wave sine_wave.launch.py tier:=2
# Or shorthand:
ros2 launch zenbedded_sine_wave sine_wave_tier2.launch.py
```

This starts:
- `robot_state_publisher`, `ros2_control_node`, `joint_state_broadcaster`, and `sine_wave_tier2` controller plugin (50 Hz).
- `rqt_plot` automatically graphing the registered hardware introspection signals:
  - `/controller_manager/introspection_data/values/values[4]` (`state_interface.sine_wave/position`)
  - `/controller_manager/introspection_data/values/values[5]` (`command_interface.sine_wave/amplitude`)
  *(Note: indices 0–3 are reserved by `pal_statistics` for asynchronous topic publishing metrics).*

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `tier` | `1` | Architecture tier: `'1'` for ROS 2 CDR topics, `'2'` for `ros2_control` hardware interface |
| `use_rqt_plot` | `true` | Launch `rqt_plot` GUI (`false` for headless / terminal-only execution) |
| `introspection_topic` | `/controller_manager/introspection_data/values` | Base introspection topic for Tier 2 `rqt_plot` |

### 3. Manual Testing & CLI Inspection

Inspect published state in real-time:

```bash
ros2 topic echo /zenbedded/sine_wave/state
```

Send a manual amplitude command (Tier 1):

```bash
ros2 topic pub -1 /zenbedded/sine_wave/cmd control_msgs/msg/JointCommand "{header: {frame_id: 'sine_wave'}, joint_names: ['sine_wave'], interface_name: 'amplitude', values: [12.5]}"
```

### 4. Host Time Synchronization (Chrony Setup)

In Tier 1, the microcontroller publishes `sensor_msgs/msg/JointState` messages with timestamps embedded in `header.stamp`. For ROS 2 tools such as `rqt_plot` to graph state and command curves simultaneously on the exact same timeline, the microcontroller clock is synchronized with the host Unix epoch clock.

The firmware uses Zephyr's SNTP client on boot, dynamically extracting the host IP from the active transport locator (e.g. `192.0.2.1` over USB CDC ECM or the host Wi-Fi IP), and sets POSIX `CLOCK_REALTIME`.

To configure your host laptop to serve NTP persistently in the background:

1. Install Chrony:
   ```bash
   sudo apt install chrony
   ```

2. Add client access and standalone serving to `/etc/chrony/chrony.conf`:
   ```text
   # Allow NTP queries from USB CDC ECM subnet
   allow 192.0.2.0/24

   # Allow queries from local LAN / Wi-Fi subnet
   allow 10.0.0.0/8

   # Serve local time even when disconnected from internet upstream
   local stratum 10
   ```

3. Enable Chrony to run permanently in the background as a systemd service:
   ```bash
   sudo systemctl enable --now chrony
   sudo systemctl restart chrony
   ```

4. Verify Chrony is active:
   ```bash
   chronyc tracking
   ```
