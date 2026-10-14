:github_url: https://github.com/ros-controls/zenbedded/blob/{REPOS_FILE_BRANCH}/doc/rcl_architecture.rst

.. _rcl_architecture:

Embedded ros2_control Architecture (zenbedded_rcl)
===================================================

This document specifies the architectural design for bringing the core abstractions of
``ros2_control`` into embedded firmware running on `Zephyr RTOS`_ via ``zenbedded_rcl``.
It details how the hardware abstraction layer (``HardwareComponent``), embedded controllers
(``ControllerInterface``), the orchestrator (``ControllerManager``), compile-time hardware
definitions (`Zephyr Devicetree`_), and the asynchronous multi-channel pub/sub communication
subsystem interface with one another.

Furthermore, this document explores multi-rate execution scenarios and RTOS scheduling models
where physical sensors, actuators, control loops, and network telemetry execute at distinct
update frequencies.

.. contents:: Table of Contents
   :depth: 2
   :local:


1. Motivation & Embedded Control Philosophy
--------------------------------------------

In standard desktop ROS 2 deployments, `ros2_control`_ orchestrates hardware drivers and
controllers on Linux. While powerful, traditional ROS 2 control architectures rely heavily on:

- Dynamic memory allocation (``std::vector``, heap strings, shared pointers)
- Runtime XML parsing (URDF parsing via ``urdfdom`` / ``tinyxml2``)
- Dynamic plugin loading via ``pluginlib`` (``dlopen``)
- Heavyweight DDS/XRCE middleware daemons and agents

Microcontrollers (such as the ESP32-S3, STM32, or NXP microcontrollers) operating close to physical
hardware have strict constraints:

- **Bounded RAM & Flash**: Heap allocations during execution are dangerous and prone to fragmentation.
- **Microsecond Determinism**: Low-level motor commutation, current loops, and encoder sampling require
  hard real-time scheduling that must never be delayed by network I/O or heap locks.
- **Direct Agent-less Wire Protocol**: Bypassing intermediate host bridge daemons via `Zenoh`_.

By introducing an embedded ``ros2_control``-compatible architecture directly into ``zenbedded_rcl``:

1. **Local Closed-Loop Autonomy**: High-rate feedback loops (e.g., 400 Hz velocity filtering, 1 kHz current
   control, or balance stabilization) execute locally on the microcontroller with zero network latency.
2. **Fail-Safe Supervisory Execution**: Safety limits, watchdog clamps, and emergency stops operate on-chip
   even if the wireless or serial connection to the host drops.
3. **Decoupled Telemetry**: State publications to host ROS 2 nodes are downsampled and emitted over Zenoh
   without introducing jitter into the physical actuator control loops.


2. System Architecture Overview
--------------------------------

The architecture establishes a unified pipeline from physical silicon registers to host ROS 2 nodes:

.. code-block:: text

   +----------------------------------------------------------------------------------------------------+
   |                                   ZEPHYR RTOS EMBEDDED SYSTEM                                      |
   |                                                                                                    |
   |   [ Zephyr Devicetree (DT) ] ─────────────────────────┐ (Compile-time static binding)              |
   |                                                       ▼                                            |
   |   +--------------------------------------------------------------------------------------------+   |
   |   | zenbedded_rcl ControllerManager                                                            |   |
   |   |                                                                                            |   |
   |   |   +────────────────────────────────────────────────────────────────────────────────────+   |   |
   |   |   | Interface Repository (Static Views)                                                |   |   |
   |   |   |   • State Interfaces:   [ motor_joint/pos, motor_joint/vel, pendulum/pos, ... ]    |   |   |
   |   |   |   • Command Interfaces: [ motor_joint/accel, motor_joint/vel, ... ]                |   |   |
   |   |   +────────────────────────────────────────────────────────────────────────────────────+   |   |
   |   |                                 ▲                                    │                     |   |
   |   |             read() (writes SI)  │                                    │ write() (reads CI)  |   |
   |   |                                 │                                    ▼                     |   |
   |   |   +─────────────────────────────┴──────+     +───────────────────────┴─────────────────+   |   |
   |   |   | Hardware Components (Devicetree)   |     | Embedded Controller Interfaces          |   |   |
   |   |   |   • ActuatorComponent (stepper)    |     |   • InvertedPendulumController (100Hz)  |   |   |
   |   |   |   • SensorComponent (AS5600 400Hz) |     |   • SafetyClampController (1kHz)        |   |   |
   |   |   |   • SensorComponent (IMU 200Hz)    |     |   • JointStateBroadcaster (100Hz)       |   |   |
   |   |   |   • SensorComponent (Battery 1Hz)  |     |   • CommandSubscriberController (100Hz) |   |   |
   |   |   +────────────────────────────────────+     +─────────────────────────────────────────+   |   |
   |   |                                                                      │                     |   |
   |   |   +──────────────────────────────────────────────────────────────────┴─────────────────+   |   |
   |   |   | Multi-Rate Dispatcher (Rational Decimation or Preemptive RTOS Threads)             |   |   |
   |   |   +──────────────────────────────────────────────────────────────────┬─────────────────+   |   |
   |   +──────────────────────────────────────────────────────────────────────┼─────────────────────+   |
   |                                                                          │                         |
   |                                        Lock-free Double Buffering (ABA)  │                         |
   |                                                                          ▼                         |
   |   +--------------------------------------------------------------------------------------------+   |
   |   | zenbedded_rcl Multi-Channel Pub/Sub Subsystem                                              |   |
   |   |   • Channel 1 (Pub): "rt/robot/joint_states"   (100 Hz, Tier 1 CDR or Tier 2 packed)       |   |
   |   |   • Channel 2 (Sub): "rt/robot/joint_commands" (100 Hz, Tier 1 CDR or Tier 2 packed)       |   |
   |   |   • Channel 3 (Pub): "rt/robot/imu/data"       (200 Hz, Tier 1 sensor_msgs/Imu)            |   |
   |   |   • Channel 4 (Pub): "rt/robot/diagnostics"    (1 Hz, system health & battery voltage)     |   |
   |   +----------------------------------------------------------------------┬─────────────────────+   |
   +──────────────────────────────────────────────────────────────────────────┼─────────────────────────+
                                                                              │
                                                         Zenoh Transport Mesh │ (UDP / TCP / USB-CDC)
                                                                              ▼
   +----------------------------------------------------------------------------------------------------+
   |                               HOST COMPUTER (Linux / ROS 2 Jazzy)                                  |
   |                                                                                                    |
   |   +────────────────────────────────────────────────────────────────────────────────────────────+   |
   |   | zenbedded_hardware_interface (SystemInterface) OR topic_based_hardware_interface           |   |
   |   |   • Lock-free realtime_tools::RealtimeThreadSafeBox ingest from Zenoh subscribers          |   |
   |   |   • Emits joint commands directly to Zenoh publisher                                       |   |
   |   +──────────────────────────────────────────────┬─────────────────────────────────────────────+   |
   |                                                  │                                                 |
   |   +──────────────────────────────────────────────┴─────────────────────────────────────────────+   |
   |   | Host controller_manager                                                                    |   |
   |   |   • JointTrajectoryController / DiffDriveController / Nav2 / MoveIt                        |   |
   |   +--------------------------------------------------------------------------------------------+   |
   +----------------------------------------------------------------------------------------------------+


3. Zephyr Devicetree (zephyr dt) Hardware Binding
-------------------------------------------------

In desktop ``ros2_control``, hardware joints and interfaces are declared dynamically in a URDF file.
In Zephyr, the hardware source of truth is the **Devicetree**.

Devicetree bindings allow the board author to map physical MCU peripherals (timers, PWMs, I2C devices,
GPIO step/dir controllers) directly into named joints and interfaces at compile time.

3.1 Devicetree Node Bindings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We specify four custom Devicetree bindings under ``dts/bindings/zenbedded/``:

1. ``zenbedded,controller-manager.yaml``:
   Root container defining base update frequency, thread priorities, and stack allocations.
2. ``zenbedded,joint-actuator.yaml``:
   Represents an actuator joint (motor, stepper, servo). Links to a physical Zephyr driver, declares
   joint name, exported state interfaces, and accepted command interfaces.
3. ``zenbedded,joint-sensor.yaml``:
   Represents a joint sensor (optical/magnetic encoder, potentiometer). Declares joint name,
   sampling frequency, and filtering options.
4. ``zenbedded,imu-sensor.yaml``:
   Represents a 6-DOF or 9-DOF IMU sensor. Declares frame ID, gyro/accel channels, and update rate.

3.2 Example Devicetree Overlay Snippet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Below is a representative overlay configuring a rotary inverted pendulum with an AS5600 magnetic encoder
and a TMC2209 stepper driver:

.. code-block:: dts

   / {
       ros2_control: ros2-control {
           compatible = "zenbedded,controller-manager";
           update-rate = <100>; /* Base loop frequency in Hz */

           /* Actuator: Motor Arm */
           motor_arm: actuator-0 {
               compatible = "zenbedded,joint-actuator";
               joint-name = "motor_joint";
               driver = <&a4988_ctrl>;
               state-interfaces = "position", "velocity";
               command-interfaces = "velocity", "acceleration";
               max-velocity = <7000>;      /* 7.0 rad/s * 1000 */
               max-acceleration = <30000>; /* 30.0 rad/s^2 * 1000 */
               update-rate = <100>;
           };

           /* Sensor: Fast Pendulum Encoder */
           pendulum_encoder: sensor-0 {
               compatible = "zenbedded,joint-sensor";
               joint-name = "pendulum_joint";
               sensor = <&as5600_encoder>;
               state-interfaces = "position", "velocity";
               update-rate = <400>; /* Fast sampling at 400 Hz */
               filter-alpha = <350>; /* Low-pass filter alpha = 0.35 */
           };
       };
   };

3.3 Compile-Time Static Instantiation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Rather than parsing string names or XML trees at boot, Zephyr C preprocessor macros iterate over
the Devicetree at compile time:

.. code-block:: cpp

   #define INIT_ACTUATOR_COMPONENT(node_id) \
     ActuatorComponent( \
       DT_PROP(node_id, joint_name), \
       DEVICE_DT_GET(DT_PHANDLE(node_id, driver)), \
       DT_PROP(node_id, update_rate)),

   static ActuatorComponent g_actuators[] = {
     DT_FOREACH_CHILD_STATUS_OKAY_VARGS(DT_NODELABEL(ros2_control), INIT_ACTUATOR_COMPONENT)
   };

This guarantees:

- **Zero dynamic memory allocation** during initialization.
- **Immediate validation**: Compilation fails if a referenced hardware device (e.g. ``<&a4988_ctrl>``)
  is missing or disabled in Devicetree.
- **ROM-placement** of all invariant metadata (joint strings, maximum limits, device pointers).


4. Component & Controller Abstractions
--------------------------------------

4.1 HardwareComponent Hierarchy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The embedded hardware abstraction mirrors the ``ros2_control`` lifecycle while remaining lightweight:

.. code-block:: cpp

   namespace zenbedded
   {

   enum class LifecycleState : uint8_t {
     UNCONFIGURED,
     INACTIVE,
     ACTIVE,
     FINALIZED,
     ERROR
   };

   class HardwareComponent
   {
   public:
     virtual ~HardwareComponent() = default;

     virtual int on_init() = 0;
     virtual int on_configure() = 0;
     virtual int on_activate() = 0;
     virtual int on_deactivate() = 0;

     virtual int read(uint32_t timestamp_us) = 0;
     virtual int write(uint32_t timestamp_us) = 0;

     [[nodiscard]] LifecycleState state() const { return state_; }
     [[nodiscard]] const char * name() const { return name_; }

   protected:
     const char * name_ = nullptr;
     LifecycleState state_ = LifecycleState::UNCONFIGURED;
   };

   }  // namespace zenbedded

4.2 Interface Views (StateInterfaceView & CommandInterfaceView)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In desktop ``ros2_control``, ``StateInterface`` and ``CommandInterface`` wrap a double pointer.
In ``zenbedded_rcl``, we maintain strict cache locality using static interface views:

.. code-block:: cpp

   namespace zenbedded
   {

   enum class InterfaceType : uint8_t {
     POSITION,
     VELOCITY,
     ACCELERATION,
     EFFORT,
     CUSTOM
   };

   struct StateInterfaceView
   {
     const char * joint_name;
     InterfaceType type;
     const double * value_ptr;

     double get_value() const { return *value_ptr; }
   };

   struct CommandInterfaceView
   {
     const char * joint_name;
     InterfaceType type;
     double * value_ptr;

     void set_value(double val) { *value_ptr = val; }
     double get_value() const { return *value_ptr; }
   };

   }  // namespace zenbedded

4.3 Embedded ControllerInterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

An embedded controller executes local algorithms and claims specific interfaces:

.. code-block:: cpp

   namespace zenbedded
   {

   class ControllerInterface
   {
   public:
     virtual ~ControllerInterface() = default;

     virtual int on_init() = 0;
     virtual int on_configure() = 0;
     virtual int on_activate() = 0;
     virtual int on_deactivate() = 0;

     /// Real-time control loop computation
     virtual int update(uint32_t time_us, uint32_t period_us) = 0;

     virtual uint32_t update_frequency() const = 0;
   };

   }  // namespace zenbedded

Special Infrastructure Controllers:

- **``JointStateBroadcaster``**: Claims read-only ``StateInterfaceView`` entries and copies their
  values into the ``zenbedded_rcl`` publisher double buffer.
- **``CommandSubscriberController``**: Reads from the ``zenbedded_rcl`` subscriber double buffer and
  writes target values to claimed ``CommandInterfaceView`` entries.

4.4 ControllerManager Orchestration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``ControllerManager`` coordinates:

1. **Interface Claiming & Mutual Exclusion**: Ensures that only one active controller can write to a
   given ``CommandInterfaceView`` at any time.
2. **Lifecycle Transitions**: Executes coordinated transitions across all registered components.
3. **Deterministic Execution Loop**:
   - ``read_hardware()``: Polls active components.
   - ``update_controllers()``: Executes active control algorithms.
   - ``write_hardware()``: Dispatches commands to physical actuators.
4. **Safety Watchdog**: Zeroes actuator commands or engages dynamic braking if commands stall or a
   hardware error flag is raised.


5. Multi Pub/Sub Layer Integration
----------------------------------

5.1 Decoupling Control Loops from Network Jitter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Network transmission (Wi-Fi, Ethernet, USB-CDC-ECM) is inherently stochastic. Even with Zenoh's
ultra-lean wire protocol, network retransmissions, socket buffer contention, or RF packet loss must
**never** block the real-time control loop.

To bridge the deterministic embedded control loop with the asynchronous network thread:

- Every pub/sub channel utilizes **lock-free double buffering with ABA versioning**.
- Hardware reads and controller updates write into the inactive buffer slot and atomically commit.
- Network publisher threads read the committed active slot without taking mutexes.

.. code-block:: text

   [ 100 Hz Control Loop ]                        [ Zenoh Network Task ]
              │                                              │
              ▼                                              │
     prepare_write_slot()                                    │
              │                                              │
     commit_write_slot() (atomic flip)                       │
              │                                              ▼
   [ DoubleBuffer: Slot 0 (Active) ] ──────────► read_state_from_buffer()
   [ DoubleBuffer: Slot 1 (Inactive) ]                       │
                                                             ▼
                                                    zenoh_publish_state()

5.2 Multi-Channel Topic Segregation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Instead of forcing all MCU telemetry into a single monolithic message, ``zenbedded_rcl`` provides
modular multi-channel pub/sub:

.. list-table::
   :header-rows: 1
   :widths: 20 15 15 50

   * - Channel
     - Type
     - Frequency
     - Description
   * - ``rt/robot/joint_states``
     - Publisher
     - 100 Hz
     - Primary joint feedback (position, velocity, effort). Tier 1 CDR or Tier 2 packed.
   * - ``rt/robot/joint_commands``
     - Subscriber
     - 100 Hz
     - Target commands received from host controllers.
   * - ``rt/robot/imu/data``
     - Publisher
     - 200 Hz
     - Dedicated high-rate IMU telemetry (accelerometer, gyroscope).
   * - ``rt/robot/diagnostics``
     - Publisher
     - 1 Hz
     - Low-rate health monitoring (bus voltage, motor thermals, packet drop counters).


6. Multi-Frequency Execution Scenarios
---------------------------------------

Real-world robots require multiple update frequencies across hardware devices and controllers.
Here we analyze four core scenarios and their implementation within ``zenbedded_rcl``.

6.1 Scenario 1: Fast Sensor Polling + Inner Loop vs. Downsampled Telemetry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **Hardware Context**: Rotary Inverted Pendulum (Furuta Pendulum).
- **Update Rates**:
  - AS5600 magnetic encoder sampled at **400 Hz** (2.5 ms period).
  - Velocity filtering (single-pole low pass filter, :math:`\alpha = 0.35`) computed at **400 Hz**.
  - Balancing controller (LQR / PID) executed at **100 Hz** (10 ms period).
  - Zenoh state publishing to host ROS 2 visualizer/telemetry at **50 Hz** (20 ms period).
- **Dataflow**:
  The 400 Hz encoder thread samples the angle and calculates velocity, storing it into the
  ``pendulum_joint/position`` and ``pendulum_joint/velocity`` state interfaces. The 100 Hz control loop
  reads the filtered state every 10 ms, computes motor acceleration, and applies it to the stepper.
  The 50 Hz telemetry task decodes the state and emits a Zenoh packet every 20 ms.

.. code-block:: text

   Time (ms):   0    2.5   5.0   7.5   10.0  12.5  15.0  17.5  20.0
   Encoder:     [X]───[X]───[X]───[X]───[X]───[X]───[X]───[X]───[X] (400 Hz)
   Controller:  [X]─────────────────────[X]─────────────────────[X] (100 Hz)
   Zenoh Pub:   [X]─────────────────────────────────────────────[X] (50 Hz)

6.2 Scenario 2: Heterogeneous Hardware Bus Timings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **Hardware Context**: Mobile Manipulator with diverse peripherals.
- **Update Rates**:
  - Motor commutation / PWM step generation: **1 kHz** (1 ms period).
  - 6-DOF IMU (I2C/SPI FIFO interrupt): **200 Hz** (5 ms period).
  - Smart battery monitor (SMBus/ADC): **1 Hz** (1000 ms period).
- **Challenge**:
  An I2C transaction reading battery voltage can block for up to 3 ms. If executed in the same loop
  as the 1 kHz motor driver, it would cause severe timing overruns and motor stutter.
- **Solution**:
  Per-peripheral scheduling domains where slow sensors execute asynchronously in low-priority threads,
  updating shared ``StateInterfaceView`` slots atomically.

6.3 Scenario 3: Cascaded Multi-Rate Controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **Hardware Context**: Direct-Drive BLDC Robotic Arm.
- **Update Rates**:
  - **Inner Loop**: Field-Oriented Current/Torque Controller running at **10 kHz** (Timer ISR).
  - **Mid Loop**: Joint Velocity PID Controller running at **1 kHz**.
  - **Outer Loop**: Cartesian Impedance / Trajectory Controller running at **100 Hz**.
- **Solution via Chained Interfaces**:
  The outer controller writes to a virtual command interface (target velocity). The mid-loop controller
  claims this virtual command, computes target current, and writes to the inner loop.

6.4 Scenario 4: Distributed Multi-Rate Synchronization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **Hardware Context**: Host-MCU Coordinated Trajectory Tracking.
- **Update Rates**:
  - Host ROS 2 ``JointTrajectoryController`` publishes trajectory setpoints at **50 Hz** over Zenoh.
  - Microcontroller runs a local cubic spline interpolator at **500 Hz** to generate smooth step pulses.
- **Handling Packet Jitter**:
  The MCU's ``CommandSubscriberController`` receives waypoints, places them in a ring buffer, and the
  local 500 Hz interpolator smoothly steps towards the goal. If a packet is delayed, the local controller
  holds acceleration limits rather than halting abruptly.


7. RTOS Scheduling & Concurrency Models in Zephyr
-------------------------------------------------

To support the multi-frequency scenarios above, ``zenbedded_rcl`` provides three distinct scheduling
models in Zephyr RTOS.

7.1 Model A: Single Master Tick with Integer Decimation (Rate Divisors)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In Model A, a single high-priority Zephyr thread runs at the **Greatest Common Denominator (GCD)**
frequency:

.. math::

   f_{\text{base}} = \text{GCD}(f_1, f_2, \ldots, f_N)

Each component and controller is assigned an integer rate divisor:

.. code-block:: cpp

   void master_tick_thread_fn()
   {
     uint32_t tick_count = 0;
     while (running) {
       // 1 kHz tasks (divisor = 1)
       if (tick_count % 1 == 0) {
         fast_motor_component.write();
       }
       // 200 Hz tasks (divisor = 5)
       if (tick_count % 5 == 0) {
         imu_component.read();
       }
       // 100 Hz tasks (divisor = 10)
       if (tick_count % 10 == 0) {
         pendulum_controller.update();
       }
       // 50 Hz tasks (divisor = 20)
       if (tick_count % 20 == 0) {
         state_broadcaster.publish();
       }
       tick_count++;
       k_sleep(K_MSEC(1));
     }
   }

- **Advantages**: Minimal RAM (only 1 thread stack), zero mutexes, deterministic execution order.
- **Limitations**: A slow or blocking sensor read delays the entire tick.

7.2 Model B: Multi-Domain Preemptive Threads with Lock-Free Double Buffers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In Model B, components and controllers are grouped into separate Zephyr threads prioritized by rate:

.. list-table::
   :header-rows: 1
   :widths: 25 20 20 35

   * - Domain Thread
     - Priority
     - Frequency
     - Responsibilities
   * - **Fast Domain**
     - ``K_PRIO_PREEMPT(0)``
     - 400 Hz – 1 kHz
     - High-speed encoder sampling, FOC/current commutation, step pulse generation.
   * - **Control Domain**
     - ``K_PRIO_PREEMPT(1)``
     - 100 Hz
     - Kinematics, LQR balancing, velocity PID, command processing.
   * - **Telemetry Domain**
     - ``K_PRIO_PREEMPT(3)``
     - 10 – 50 Hz
     - Zenoh publishing, JSON/CDR serialization, diagnostics.
   * - **Background Domain**
     - ``K_PRIO_PREEMPT(5)``
     - 1 Hz
     - Battery monitoring, LED strip animations, Wi-Fi keepalive.

Communication between domains is handled via lock-free double buffering, completely preventing
priority inversion.

7.3 Model C: Timer ISR / DMA + Workqueue Hybrid
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In Model C:

- Hardware timer interrupts (or DMA completion interrupts) capture raw encoder/IMU counts with
  sub-microsecond accuracy.
- Processing is offloaded to a dedicated high-priority Zephyr workqueue (``k_work_submit_to_queue``).
- Zenoh network tasks run on cooperative background work items.

7.4 Comparative Trade-off Analysis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Metric
     - Model A (Decimation)
     - Model B (Multi-Domain)
     - Model C (ISR/Workqueue)
   * - **Jitter on Fast Loop**
     - Moderate (I/O dependent)
     - **Extremely Low (< 5 µs)**
     - **Sub-microsecond**
   * - **Memory Footprint**
     - **Lowest (~1 KB stack)**
     - Medium (~4-6 KB stacks)
     - Medium (~3 KB stacks)
   * - **Concurrency Safety**
     - Trivial (Single thread)
     - Requires Double Buffer
     - Requires Atomic / Spinlock
   * - **Recommended For**
     - Simple boards, single-bus
     - Multi-sensor robots
     - High-speed motor drives


8. Reference Implementation: Inverted Pendulum
----------------------------------------------

Applying this architecture to the `Inverted Pendulum Demo`_:

1. **Devicetree Overlay**:
   Defines ``motor_arm`` (stepper actuator @ 100 Hz) and ``pendulum_encoder`` (AS5600 sensor @ 400 Hz).
2. **Execution Mapping (Model B)**:

   - **400 Hz Thread**: Samples AS5600 via I2C, updates ``pendulum_joint/position`` and
     ``pendulum_joint/velocity`` state interface views.
   - **100 Hz Thread**: Evaluates ``InvertedPendulumController`` LQR equation, checks travel limits,
     and commands ``stepper_ctrl_move_to()``.
   - **100 Hz State Broadcaster**: Encodes state into Tier 2 packed struct and publishes to
     ``rt/robot/state``.
   - **Asynchronous Zenoh Subscriber**: Ingests command acceleration from host over ``rt/robot/cmd``.
3. **Host Coupling**:
   The host runs ``zenbedded_hardware_interface`` inside ``ros2_control``, seamlessly visualizing and
   controlling the pendulum in RViz2 and Raylib.


9. Summary & Next Steps
-----------------------

This architecture bridges the modular, declarative paradigm of ``ros2_control`` with the resource-constrained,
deterministic execution environment of `Zephyr RTOS`_.

- Implementation tasks are tracked in the master project plan:
  - ``HardwareComponent`` base class and ``ComponentManager`` lifecycle implementation in ``zenbedded_rcl``.
  - Static Devicetree binding YAML definitions under ``zenbedded_rcl/dts/bindings/``.
  - Multi-topic dynamic client registration in ``zenbedded_transport``.


.. _Zephyr RTOS: https://zephyrproject.org/
.. _Zephyr Devicetree: https://docs.zephyrproject.org/latest/build/dts/index.html
.. _ros2_control: https://control.ros.org/
.. _Zenoh: https://zenoh.io/
.. _Inverted Pendulum Demo: https://github.com/ros-controls/zenbedded/tree/main/demos/inverted_pendulum
