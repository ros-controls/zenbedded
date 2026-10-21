<picture>
  <source media="(prefers-color-scheme: dark)" srcset="doc/images/zenbedded_logo.png">
  <source media="(prefers-color-scheme: light)" srcset="doc/images/zenbedded_logo.png">
  <img src="doc/images/zenbedded_logo.png" width="480" alt="Zenbedded">
</picture>

<p>
  <a href="https://opensource.org/licenses/Apache-2.0"><img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg" alt="License"></a>
  <a href="https://github.com/ros-controls/zenbedded/actions/workflows/lyrical-binary-build.yml?branch=master"><img src="https://github.com/ros-controls/zenbedded/actions/workflows/lyrical-binary-build.yml/badge.svg?branch=master" alt="Lyrical Binary Build"></a>
  <a href="https://github.com/ros-controls/zenbedded/actions/workflows/zephyr-twister.yml?branch=master"><img src="https://github.com/ros-controls/zenbedded/actions/workflows/zephyr-twister.yml/badge.svg?branch=master" alt="Zephyr Twister"></a>
</p>

Zephyr-based microcontrollers as first-class `ros2_control` hardware, communicating
with the ROS 2 host directly over [Zenoh](https://zenoh.io/) — without a micro-ROS
agent or bridge process. The host side is `zenbedded_hardware_interface`; the MCU side
is `zenbedded_rcl` and `zenbedded_transport`, built from one `interface_schema.yaml`.
Tier 1 uses CDR-encoded ROS 2 messages; Tier 2 uses schema-packed payloads.

## Supported distributions

| ROS 2 Distro | Branch | Host build | Documentation |
| :---------: | :----: | :--------: | :-----------: |
| **Lyrical** | [`master`](https://github.com/ros-controls/zenbedded/tree/master) | [![Lyrical Binary Build](https://github.com/ros-controls/zenbedded/actions/workflows/lyrical-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/zenbedded/actions/workflows/lyrical-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/rolling/doc/zephyr-zenoh-integration/doc/index.html) |
| **Kilted** | [`master`](https://github.com/ros-controls/zenbedded/tree/master) | ![source build](https://img.shields.io/badge/build-source_build-blue)| [Source build](doc/supported_distributions.rst) |
| **Jazzy** | [`master`](https://github.com/ros-controls/zenbedded/tree/master) | ![source build](https://img.shields.io/badge/build-source_build-blue) | [Source build](doc/supported_distributions.rst) |
| **Humble and older** | — | ![unsupported](https://img.shields.io/badge/build-unsupported-red) | — |

Lyrical is validated end-to-end. Kilted and Jazzy use the same `master` sources; their
CI lanes are planned after the first release.

## Supported hardware

Platform | Status | Notes
:------: | :----: | :----
`esp32s3_devkitc/esp32s3/procpu` | ![passing](https://img.shields.io/badge/build-passing-brightgreen) | Reference target; built in CI and used by all demos
`native_sim/native/64` | ![passing](https://img.shields.io/badge/build-passing-brightgreen) | CI test target; no radio
Other Zephyr boards | ![not tested](https://img.shields.io/badge/build-not%20tested-yellow) | Requires a network interface supported by zenoh-pico

Zephyr v4.4.2 and the `zenbedded` zenoh-pico fork are pinned in
[`docker/image-west.yml`](docker/image-west.yml).

## Quick start

The development image provides ROS 2 Lyrical, `ros2_control`, Zephyr, and zenoh-pico.

```bash
git clone https://github.com/ros-controls/zenbedded.git
cd zenbedded
docker compose up -d
docker compose exec -it dev-env bash
```

Build the host packages:

```bash
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select zenbedded_hardware_interface
source install/setup.bash
```

Build and flash the firmware:

```bash
cd /zephyr_ws/demos/inverted_pendulum/inverted_pendulum_tier2
west build -p always -b esp32s3_devkitc/esp32s3/procpu
west flash --esp-device /dev/ttyUSB0
```

Run the host side:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py
```

Add `use_mock_hardware:=true` to the launch command to run without an MCU. See
[Getting Started](doc/getting_started.rst) for Wi-Fi setup and the full walkthrough.

## Documentation

- [control.ros.org](https://control.ros.org/rolling/doc/zephyr-zenoh-integration/doc/index.html)
- [Getting Started](doc/getting_started.rst)
- [Supported ROS 2 Distributions](doc/supported_distributions.rst)
- [Architecture](doc/architecture.rst)

Demos: [`inverted_pendulum`](demos/inverted_pendulum) and
[`sine_wave`](demos/sine_wave).

## Contributing

As an open-source project, we welcome each contributor, regardless of their background and experience. Pick a [PR](https://github.com/ros-controls/zenbedded/pulls) and review it, or [create your own](https://github.com/ros-controls/zenbedded/contribute)!
If you are new to the project, please read the [contributing guide](https://control.ros.org/rolling/doc/contributing/contributing.html) for more information on how to get started.

## License

Apache-2.0. See [`LICENSE`](LICENSE).

## Known limitations

- Packages are built from source; there is no rosdistro release yet.
- Only Lyrical is CI-tested; Kilted and Jazzy are source builds.
- A standalone Zephyr workspace is not supported yet; use the development container.
