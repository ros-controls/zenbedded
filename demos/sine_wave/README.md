# Zenbedded Sine Wave Demo

A complete end-to-end demo demonstrating both **Tier 1** (native ROS 2 CDR serialization) and **Tier 2** (high-throughput packed numeric schema for `ros2_control`) using Zenbedded on Zephyr RTOS.

The firmware generates a sinusoidal wave signal (`position = amplitude * sin(2 * pi * f * t)`) and dynamically accepts command messages to adjust the wave amplitude in real time.

```
┌─────────────────────────────────┐                       ┌──────────────────────────────────────────────┐
│ ESP32-S3 (Zephyr)               │   Zenoh (tcp/7447)    │ Host PC (ROS 2)                              │
│                                 │ <───────────────────> │                                              │
│ - sine_wave_tier1 (ROS 2 CDR)   │  Wi-Fi / USB CDC ECM  │ • rqt_plot / ros2 topic echo / PlotJuggler   │
│ - sine_wave_tier2 (No-CDR POD)  │                       │ • zenbedded_hardware_interface (ros2_control)│
└─────────────────────────────────┘                       └──────────────────────────────────────────────┘
```

---

## Architecture Variants

- **Tier 1 (`sine_wave_tier1`)**:
  - Implements standard ROS 2 messages using `JointStateCodec` (`sensor_msgs/msg/JointState`) and `JointCommandCodec` (`control_msgs/msg/JointCommand`).
  - Directly compatible with all standard ROS 2 nodes, CLI commands (`ros2 topic echo`), and visualization tools (`rqt_plot`) without requiring a custom hardware interface on the host PC.
  - Paired with the host-side `sine_wave_tier1` (`SineWaveTier1Node`) C++ commander running at 100 Hz.

- **Tier 2 (`sine_wave_tier2`)**:
  - Uses zero-copy packed numeric C structs generated from `interface_schema.yaml`.
  - Interfaces directly with `zenbedded_hardware_interface` for seamless `ros2_control` integration.
  - Paired with the host-side `sine_wave_tier2` (`SineWaveTier2Controller`) controller plugin running at 100 Hz.

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

Identify your PC's IP address on the local network (via `ip addr` on Linux or `ipconfig` on Windows).

In the firmware project's `prj.conf` (`sine_wave_tier1/prj.conf` or `sine_wave_tier2/prj.conf`), configure the locator:

```ini
CONFIG_ZENBEDDED_ZENOH_LOCATOR="192.168.1.50:7447"
```

*(Note: For USB CDC ECM connections on ESP32-S3, default static peer IP `192.0.2.1:7447` is used automatically).*

---

## Step 3: Building and Flashing (Firmware)

### For ESP32-S3 DevKitC

```bash
# Option A: Wired USB CDC ECM (Ethernet over USB)
west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -- -DSNIPPET=cdc-ecm
west flash

# Option B: Wireless Wi-Fi
west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -- -DSNIPPET=wifi
west flash

# Option C: Concurrent Dual Transport (USB CDC ECM + Wi-Fi simultaneously)
west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -S cdc-ecm -S wifi
west flash
```

*(For Tier 2, replace `sine_wave_tier1` with `sine_wave_tier2` in the build commands above).*

---

## Step 4: Configure Wi-Fi Credentials (Wi-Fi Builds Only)

Open a serial monitor (e.g., `tio /dev/ttyACM1` or `picocom -b 115200 /dev/ttyACM1`) to access the Zephyr shell:

```text
wifi_cred add -s "<MY_WIFI_SSID>" -p "<MY_WIFI_PASSWORD>"
wifi_cred set_default 0
wifi_cred auto_connect
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
- `sine_wave_tier1` node streaming oscillating amplitude commands at 100 Hz.
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
- `robot_state_publisher`, `ros2_control_node`, `joint_state_broadcaster`, and `sine_wave_tier2` controller plugin (100 Hz).
- `rqt_plot` automatically graphing the registered hardware introspection signals:
  - `/controller_manager/introspection_data/values/values[0]` (position)
  - `/controller_manager/introspection_data/values/values[1]` (amplitude)

### 3. Manual Testing & CLI Inspection

Inspect published state in real-time:

```bash
ros2 topic echo /zenbedded/sine_wave/state
```

Send a manual amplitude command (Tier 1):

```bash
ros2 topic pub -1 /zenbedded/sine_wave/cmd control_msgs/msg/JointCommand "{header: {frame_id: 'sine_wave'}, joint_names: ['sine_wave'], interface_name: 'amplitude', values: [12.5]}"
```

### 4. Visualization with PlotJuggler

> [!NOTE]
> **PlotJuggler RMW Notice**: PlotJuggler's ROS 2 plugin relies on low-level DDS introspection hooks not present in `rmw_zenoh_cpp`. If using PlotJuggler with a DDS bridge, unset `RMW_IMPLEMENTATION` for both plotjuggler and sine_wave.launch.py:
> ```bash
> unset RMW_IMPLEMENTATION
> ros2 run plotjuggler plotjuggler
> ```
