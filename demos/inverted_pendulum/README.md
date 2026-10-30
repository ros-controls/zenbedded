# Zenbedded Inverted Pendulum Demo

A rotary inverted pendulum (Furuta pendulum) demonstration showcasing both **Tier 1** (ROS 2 CDR serialization over Zenoh) and **Tier 2** (high-throughput packed numeric schema for `ros2_control`) using Zenbedded on Zephyr RTOS.

An ESP32-S3 running Zephyr reads an AS5600 magnetic encoder at 400 Hz and drives a stepper motor at 100 Hz, while non-linear energy-shaping swing-up, LQR balancing, and RViz 3D visualization run on a Host PC via [Zenoh](https://zenoh.io/).

```
┌─────────────────────────────────────────┐                       ┌──────────────────────────────────────────────┐
│ ESP32-S3 (Zephyr RTOS)                  │   Zenoh (tcp/7447)    │ Host PC (ROS 2)                              │
│                                         │ <───────────────────> │                                              │
│ • AS5600 Magnetic Encoder (400 Hz)      │  Wi-Fi / USB CDC ECM  │ • ros2_control_node (100 Hz)                 │
│ • TMC2209 / A4988 Stepper Driver (100Hz)│                       │ • InvertedPendulumTier1Hardware (Tier 1)     │
│ • Soft Hard-Stops (±135°)               │                       │ • ZenbeddedHardware (Tier 2)                 │
│ • WS2812 NeoPixel Status LED            │                       │ • LQR & Non-linear Energy Shaping Controller │
│                                         │                       │ • RViz2 3D Real-Time Visualizer              │
└─────────────────────────────────────────┘                       └──────────────────────────────────────────────┘
```

---

## Architecture Variants

The host uses a **unified `ros2_control` stack** across all options:

- **Tier 1 (`inverted_pendulum_tier1`)**:
  - Firmware implements standard ROS 2 messages using `JointStateCodec` (`sensor_msgs/msg/JointState`) and `JointCommandCodec` (`control_msgs/msg/JointCommand`).
  - Host loads `InvertedPendulumTier1Hardware`, which relays state and command messages into `ros2_control`.
  - Driven by the unified `inverted_pendulum_controller` (LQR balancing + swing-up pump).

- **Tier 2 (`inverted_pendulum_tier2`)**:
  - Firmware uses zero-copy packed binary structs generated from `interface_schema.yaml`.
  - Host loads `ZenbeddedHardware` for low-latency direct-Zenoh integration.
  - Driven by the unified `inverted_pendulum_controller`.

- **Mock Simulation (`use_mock_hardware:=true`)**:
  - Full in-process 2-DOF Lagrangian dynamics simulation without physical hardware.

In all tiers, `joint_state_broadcaster` publishes `/joint_states` and `rviz2` displays real-time 3D motion.

---

## Hardware Support

| Board | Target Qualifier | Supported Transports |
|---|---|---|
| **ESP32-S3 DevKitC** | `esp32s3_devkitc/esp32s3/procpu` | USB CDC ECM (`-S cdc-ecm`), Wi-Fi (`-S wifi`), or Dual Transport (`-S cdc-ecm -S wifi`) |

---

## Step 1: Git LFS 3D Meshes

Ensure 3D binary STL meshes are downloaded for RViz visualization:

```bash
git lfs pull
```

---

## Step 2: Start the Zenoh Router

On your Host PC, start the Zenoh router:

```bash
zenohd -l tcp/0.0.0.0:7447
```

*(Alternatively, if using `rmw_zenoh_cpp`: `ros2 run rmw_zenoh_cpp rmw_zenohd`).*

---

## Step 3: Configure Host NTP Server (Chrony)

To synchronize timestamps across host and MCU, allow local subnets in `/etc/chrony/chrony.conf`:

```text
allow 192.0.2.0/24
```

Then restart Chrony:

```bash
sudo systemctl restart chrony
```

---

## Step 4: Building and Flashing Firmware

### For Tier 1 (`inverted_pendulum_tier1`)

```bash
# Option A: Wired USB CDC ECM (Ethernet over USB)
west build -b esp32s3_devkitc/esp32s3/procpu demos/inverted_pendulum/inverted_pendulum_tier1 -p always -S cdc-ecm
west flash

# Option B: Wireless Wi-Fi
west build -b esp32s3_devkitc/esp32s3/procpu demos/inverted_pendulum/inverted_pendulum_tier1 -p always -S wifi
west flash

# Option C: Concurrent Dual Transport (USB CDC ECM prioritized + Wi-Fi fallback)
west build -b esp32s3_devkitc/esp32s3/procpu demos/inverted_pendulum/inverted_pendulum_tier1 -p always -S cdc-ecm -S wifi
west flash
```

### For Tier 2 (`inverted_pendulum_tier2`)

Replace `inverted_pendulum_tier1` with `inverted_pendulum_tier2` in any of the commands above.

---

## Step 5: Configure Wi-Fi Credentials (Wi-Fi Builds Only)

Open a serial monitor (e.g., `west espressif monitor` or `tio /dev/ttyACM1`):

```text
wifi cred add -s "<MY_WIFI_SSID>" -k 1 -p "<MY_WIFI_PASSWORD>"
```

Credentials are automatically persisted to non-volatile storage (NVS flash).

---

## Step 6: Check LED Status

| LED Indicator | Status |
|---|---|
| **Solid Blue** | Power-on initialization, awaiting network IP and encoder zero calibration |
| **Flashing Green (4 Hz)** | Network connected, encoder calibrated, 100 Hz control loop running |
| **Solid Red** | Safety limit trip / encoder disconnect |

---

## Step 7: Launch Host Controller & RViz

Build the ROS 2 workspace:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select zenbedded_inverted_pendulum --symlink-install
source install/setup.bash
```

Launch:

```bash
# Tier 1 (Topic-Based Hardware Interface + LQR Controller + RViz)
ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py tier:=1

# Tier 2 (Zenoh Binary Hardware Interface + LQR Controller + RViz)
ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py tier:=2

# In-Process Simulation (Mock Hardware + LQR Controller + RViz with Periodic Random Kicks)
ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py use_mock_hardware:=true

# In-Process Simulation (Without Kicks)
ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py use_mock_hardware:=true enable_kicking:=false

# Headless Execution (disable RViz)
ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py tier:=2 use_rviz:=false
```

---

## Hardware Wiring Guide

### TMC2209 Stepper Driver (Default)

| Signal | ESP32-S3 GPIO | Notes |
|---|---|---|
| **EN** | GPIO4 | Active Low |
| **M0** | GPIO5 | Microstepping configuration |
| **M1** | GPIO6 | Microstepping configuration |
| **STEP** | GPIO7 | Direction inverted in overlay |
| **DIR** | GPIO8 | Direction control |
| **VDD** | 3.3V | Logic power |
| **GND** | GND | Common ground with MCU and motor supply |

### AS5600 Magnetic Encoder (I2C)

| Signal | ESP32-S3 GPIO | Notes |
|---|---|---|
| **SDA** | GPIO1 | 400 kHz Fast-Mode I2C with internal pull-up |
| **SCL** | GPIO2 | 400 kHz Fast-Mode I2C with internal pull-up |
| **VCC** | 3.3V | Logic power |
| **GND** | GND | Common ground |

### Status LED (WS2812 NeoPixel)

| Signal | ESP32-S3 GPIO | Notes |
|---|---|---|
| **DIN** | GPIO48 | Driven via I2S DMA on-board LED |
