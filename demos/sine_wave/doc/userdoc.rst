========================
Zenbedded Sine Wave Demo
========================

A complete end-to-end demo demonstrating both **Tier 1** (native ROS 2 CDR serialization) and **Tier 2** (high-throughput packed numeric schema for ``ros2_control``) using Zenbedded on Zephyr RTOS.

The firmware generates a sinusoidal wave signal (``position = amplitude * sin(2 * pi * f * t)``) and dynamically accepts commands to adjust the wave amplitude in real time.

.. code-block:: text

    ┌─────────────────────────────────┐                       ┌──────────────────────────────────────────────┐
    │ ESP32-S3 (Zephyr)               │   Zenoh (tcp/7447)    │ Host PC (ROS 2)                              │
    │                                 │ <───────────────────> │                                              │
    │ - sine_wave_tier1 (ROS 2 CDR)   │  Wi-Fi / USB CDC ECM  │ • rqt_plot / ros2 topic echo                 │
    │ - sine_wave_tier2 (No-CDR POD)  │                       │ • zenbedded_hardware_interface (ros2_control)│
    └─────────────────────────────────┘                       └──────────────────────────────────────────────┘

Architecture Variants
---------------------

- **Tier 1 (``sine_wave_tier1``)**:
  - Implements standard ROS 2 messages using ``JointStateCodec`` (``sensor_msgs/msg/JointState``) and ``JointCommandCodec`` (``control_msgs/msg/JointCommand``).
  - Directly compatible with standard ROS 2 nodes, CLI commands (``ros2 topic echo``), and ``rqt_plot`` without requiring a custom hardware interface on the host PC.
  - Paired with the host-side ``sine_wave_tier1`` C++ commander node (100 Hz).

- **Tier 2 (``sine_wave_tier2``)**:
  - Uses zero-copy packed numeric C structs generated from ``interface_schema.yaml``.
  - Interfaces directly with ``zenbedded_hardware_interface`` for seamless ``ros2_control`` integration.
  - Paired with the host-side ``sine_wave_tier2`` (``SineWaveTier2Controller``) controller plugin (100 Hz).

Hardware Support
----------------

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Board
     - Target Qualifier
     - Supported Transports
   * - **ESP32-S3 DevKitC**
     - ``esp32s3_devkitc/esp32s3/procpu``
     - USB CDC ECM (``-S cdc-ecm``), Wi-Fi (``-S wifi``), or Dual Transport (``-S cdc-ecm -S wifi``)

Prerequisites & Zenoh Router
----------------------------

Start the Zenoh router on your Host PC:

.. code-block:: bash

    export ZENOH_CONFIG_OVERRIDE='mode="peer";listen/endpoints=["tcp/0.0.0.0:7447"]'
    export RUST_LOG=zenoh=debug
    ros2 run rmw_zenoh_cpp rmw_zenohd

Building and Flashing Firmware
------------------------------

For ESP32-S3 DevKitC:

.. code-block:: bash

    # Option A: Wired USB CDC ECM (Ethernet over USB)
    west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -- -DSNIPPET=cdc-ecm
    west flash

    # Option B: Wireless Wi-Fi
    west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -- -DSNIPPET=wifi
    west flash

    # Option C: Concurrent Dual Transport (USB CDC ECM prioritized + Wi-Fi fallback)
    west build -b esp32s3_devkitc/esp32s3/procpu demos/sine_wave/sine_wave_tier1 -p always -S cdc-ecm -S wifi
    west flash

For Tier 2, replace ``sine_wave_tier1`` with ``sine_wave_tier2`` in the build commands above.

Wi-Fi Configuration (Wi-Fi Builds Only)
---------------------------------------

Open a serial monitor (e.g., ``tio /dev/ttyACM1``) and add your Wi-Fi credentials:

.. code-block:: text

    wifi_cred add -s "<MY_WIFI_SSID>" -p "<MY_WIFI_PASSWORD>"
    wifi_cred set_default 0
    wifi_cred auto_connect

Credentials persist in NVS across power cycles.

Running the Host Controller & Visualization
-------------------------------------------

Build the ROS 2 workspace:

.. code-block:: bash

    cd ~/zephyr_ws/ros2_ws
    colcon build --packages-select zenbedded_sine_wave zenbedded_hardware_interface zenbedded_transport
    source install/setup.bash

Tier 1 Execution (ROS 2 CDR)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Launch the Tier 1 commander and live visualization:

.. code-block:: bash

    ros2 launch zenbedded_sine_wave sine_wave.launch.py tier:=1

This launches:

1. ``sine_wave_tier1`` (C++ commander publishing ``control_msgs/msg/JointCommand`` at 100 Hz).
2. ``rqt_plot`` automatically graphing:
   - ``/zenbedded/sine_wave/state/position[0]`` (firmware sine wave)
   - ``/zenbedded/sine_wave/cmd/values[0]`` (commanded amplitude)

Tier 2 Execution (ros2_control)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Launch the Tier 2 ``ros2_control`` stack and introspection plot:

.. code-block:: bash

    ros2 launch zenbedded_sine_wave sine_wave.launch.py tier:=2

This launches:

1. ``robot_state_publisher`` and ``ros2_control_node`` (with ``ZenbeddedHardware`` interface).
2. ``joint_state_broadcaster`` and ``sine_wave_tier2`` (100 Hz controller plugin).
3. ``rqt_plot`` targeting exclusively the introspection signals:
   - ``/controller_manager/introspection_data/values/values[0]`` (position)
   - ``/controller_manager/introspection_data/values/values[1]`` (amplitude)

Convenience Launch Aliases
~~~~~~~~~~~~~~~~~~~~~~~~~~

You can also use dedicated shorthand launch files:

.. code-block:: bash

    # Run Tier 1
    ros2 launch zenbedded_sine_wave sine_wave_tier1.launch.py

    # Run Tier 2
    ros2 launch zenbedded_sine_wave sine_wave_tier2.launch.py

Manual Testing & CLI Inspection
-------------------------------

Echo published state in real-time:

.. code-block:: bash

    ros2 topic echo /zenbedded/sine_wave/state

Publish a manual amplitude command (Tier 1):

.. code-block:: bash

    ros2 topic pub -1 /zenbedded/sine_wave/cmd control_msgs/msg/JointCommand "{header: {frame_id: 'sine_wave'}, joint_names: ['sine_wave'], interface_name: 'amplitude', values: [12.5]}"
