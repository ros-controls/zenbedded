# Zenbedded: Embedded ros2_control with Zephyr & Zenoh

Better embedded integration for ROS 2 robots: bringing microcontrollers running [Zephyr RTOS](https://zephyrproject.org/) into `ros2_control` as first-class participants, with agent-less, low-latency communication over [Zenoh](https://zenoh.io/).

> **Status:** This work was originally proposed for GSoC 2026 and was not selected. We are doing it anyway.

## Motivation

Modern robots increasingly rely on microcontrollers for sensing and actuation close to the hardware, while higher-level control and planning runs on a companion computer. Today, integrating these MCUs into a ROS 2 / `ros2_control` system typically requires bridges or agents that add latency, jitter and operational complexity.

This project explores a more direct path: by combining [Zephyr RTOS](https://docs.zephyrproject.org/) on the embedded side with [Zenoh](https://zenoh.io/docs/overview/what-is-zenoh/) as the wire protocol, MCUs can publish sensor state and consume joint commands directly, without an intermediate agent, while keeping the determinism that control loops need.

## Goals

- Make Zephyr-based microcontrollers behave as proper `ros2_control` hardware, not as opaque peripherals behind a bridge.
- Reduce end-to-end latency and jitter in the embedded-to-host control path compared to existing DDS-based solutions.
- Keep the embedded footprint small and the real-time behaviour predictable.
- Provide reusable building blocks (hardware interface template, Zephyr module, benchmarks) so other projects can adopt the pattern.

## Scope

- A `ros2_control` hardware interface template that talks to embedded targets over Zenoh.
- A Zephyr module that exposes sensor data and accepts joint commands in a compatible format.
- A serialization layer mapping state and command interfaces to Zenoh key-value paths efficiently.
- Real-time refinements on the embedded side to minimize context switching and memory use in control loops.
- A benchmark suite comparing latency and jitter against existing DDS-based approaches.

An ESP32 is the reference target, but the work should generalize to other Zephyr-supported boards.

## Documentation

Documentation is available on
[control.ros.org](https://control.ros.org/rolling/doc/zephyr-zenoh-integration/doc/index.html).

## References

- [Zephyr RTOS](https://zephyrproject.org/) — [docs](https://docs.zephyrproject.org/), [GitHub](https://github.com/zephyrproject-rtos/zephyr)
- [Zenoh](https://zenoh.io/) — [docs](https://zenoh.io/docs/overview/what-is-zenoh/), [zenoh-pico (embedded)](https://github.com/eclipse-zenoh/zenoh-pico)
- [ros2_control](https://control.ros.org/)
