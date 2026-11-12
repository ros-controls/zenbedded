:github_url: https://github.com/ros-controls/zenbedded/blob/{REPOS_FILE_BRANCH}/doc/rcl_architecture.rst

.. _rcl_architecture:

Embedded ros2_control Architecture (zenbedded_rcl)
===================================================

.. note::

   **Status: proposal and delivery plan.** Nothing in sections 4–8 is implemented yet.
   Section 11 lays out the phased plan to get there.
   ``zenbedded_rcl/include/zenbedded_rcl/hardware_component.hpp`` is an empty header.
   The "Today" column in :ref:`rcl_arch_status` lists what does exist.

This document proposes how to bring the core ``ros2_control`` abstractions onto
`Zephyr RTOS`_ microcontrollers in ``zenbedded_rcl``. These are hardware components,
controllers, a controller manager and the lifecycle that ties them together. The goal is
that a developer who knows ``ros2_control`` can read MCU code and recognise every concept.
Where the MCU version has to differ (no heap, no ``pluginlib``, no URDF parsing at boot,
several rate domains), the reason is stated in the relevant section.

.. contents:: Table of Contents
   :depth: 2
   :local:


1. Motivation
-------------

Host-side ``ros2_control`` relies on dynamic allocation, runtime URDF parsing, ``pluginlib``
(``dlopen``) and a single ``update()`` loop. None of those fit an MCU:

- **Bounded RAM and flash.** Allocating after boot risks heap fragmentation.
- **Determinism.** Encoder sampling, step generation and balance loops must not wait on
  network I/O or heap locks.
- **Agent-less transport.** The MCU talks to the host directly over `Zenoh`_, with no
  XRCE agent in between.

An embedded ``ros2_control`` layer buys three things:

1. **Local closed loops.** Fast loops (400 Hz filtering, 1 kHz velocity PID, balancing)
   run on the MCU with no network latency.
2. **Local safety.** Command watchdogs, travel limits and safe-stop keep working when the
   Wi-Fi or USB link drops.
3. **Decoupled telemetry.** State goes to the host at its own, lower rate without
   adding jitter to the actuator loop.


.. _rcl_arch_status:

2. Status and Scope
-------------------

**Baseline.** This document assumes PR #106 (multi-publisher / multi-subscriber API) is
merged before any of this work starts. "Today" therefore means ``master`` plus PR #106.

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Concept
     - Today (``master`` + PR #106)
     - Proposed here
   * - Hardware access
     - Hand-written in each demo ``main.cpp`` (``stepper_ctrl_*``, AS5600 thread)
     - ``HardwareComponent`` instances generated from Devicetree
   * - State/command exchange with host
     - ``ZenbeddedNode`` with ``ZenbeddedPublisher<Codec>`` /
       ``ZenbeddedSubscriber<Codec>``, each with its own ``DoubleByteBuffer``. The
       demos still use the older ``ZenbeddedClient``
     - Unchanged API. Broadcaster/forwarder controllers are built on the publisher and
       subscriber
   * - Cross-thread buffers
     - ``DoubleBuffer`` / ``DoubleByteBuffer`` guarded by ``k_spinlock``
       (lock-free version replaced in ``27e22ea``)
     - Same primitive, used only at rate-domain boundaries
   * - Command watchdog
     - Per demo (``kCommandTimeoutMs`` in the inverted pendulum)
     - Owned by ``ControllerManager``
   * - Control law (inverted pendulum)
     - Runs **on the host** (``inverted_pendulum_controller``). MCU integrates the
       commanded acceleration
     - Can optionally move onto the MCU as an embedded controller
   * - Subscriber callbacks / periodic publish
     - Subscriber callbacks run in the zenoh-pico read task. Periodic publish uses
       ``k_work_delayable`` on the system workqueue (``ZenbeddedClient``: its own
       ``k_thread``)
     - Dispatched by ``ZenbeddedExecutor`` (issue #120, section 8)

**Out of scope:**

- Current/FOC loops above ~2 kHz. These stay inside the motor driver (timer ISR or DMA)
  and appear to the framework as an ``effort`` or ``velocity`` command interface.
- Runtime plugin loading.
- URDF on the MCU.
- ROS 2 actions on the MCU.


3. System Overview
------------------

.. code-block:: text

   ┌───────────────────────── Zephyr MCU (zenbedded_rcl) ─────────────────────────┐
   │                                                                              │
   │  Devicetree ─(compile time)─► ResourceManager                                │
   │                                ├─ HardwareComponents (actuators, sensors)    │
   │                                └─ Interfaces (state / command / reference)   │
   │                                     ▲ read()             │ write() + limits  │
   │                                     │                    ▼                   │
   │  ControllerManager ─ owns ─► RateDomains (thread + period each)              │
   │                                └─ per tick: read → update → write            │
   │                                                                              │
   │  Controllers: joint_state_broadcaster, forward_command_controller, pid, ...  │
   │       │ (broadcasters / forwarders only)                                     │
   │       ▼                                                                      │
   │  DoubleBuffer<T> ◄─► ZenbeddedExecutor / pub-sub ◄─► zenbedded_transport     │
   └───────────────────────────────────────┬──────────────────────────────────────┘
                                           │ Zenoh over UDP / TCP / USB CDC-ECM
   ┌──────────────────────── Host (ROS 2 Jazzy) ──────────────────────────────────┐
   │  zenbedded_hardware_interface ◄─► controller_manager ◄─► host controllers    │
   └──────────────────────────────────────────────────────────────────────────────┘

The MCU-side names deliberately match the host-side ones. They are ``ResourceManager``,
``ControllerManager``, state/command/reference interfaces, ``on_configure`` and so on. The
earlier name ``ComponentManager`` is dropped: its job is ``ResourceManager``.


4. Devicetree: Hardware and Controller Declaration
--------------------------------------------------

On the host, joints and interfaces come from the ``<ros2_control>`` tag in the URDF, and
controllers come from ``controllers.yaml``. On the MCU, Devicetree takes both roles. It
declares rate domains, hardware components and **controller instances** under one
``ros2-control`` node. Controllers are software nodes, the same way Zephyr declares
``gpio-keys`` or input processors.

Kconfig only decides which controller *types* are compiled in, and it does that
automatically (4.5). A controller type is never compiled unless an enabled node uses
it.

4.1 Bindings
^^^^^^^^^^^^

All bindings live in ``zenbedded_rcl/dts/bindings/``. ``zephyr/module.yml`` gains
``settings: dts_root: .`` so the build finds them.

**Framework and hardware**

``zenbedded,controller-manager``
   Root node. Its children are rate domains, hardware components and controllers.
``zenbedded,rate-domain``
   ``update-rate`` (Hz), ``priority``, ``stack-size``. Each domain becomes one thread.
``zenbedded,joint-actuator``
   ``joint-name``, ``driver`` (phandle), ``state-interfaces``, ``command-interfaces``,
   ``domain`` (phandle), limits.
``zenbedded,joint-sensor``
   ``joint-name``, ``sensor`` (phandle), ``state-interfaces``, ``domain``, filter options,
   ``acquisition`` (``periodic``, ``notify``, ``offload`` or ``dma``; default
   ``periodic``), ``stale-timeout-ms``.
``zenbedded,imu-sensor``
   ``frame-id``, ``sensor`` (phandle), ``domain``, ``acquisition``, ``stale-timeout-ms``.

**Controllers**

Every controller binding includes ``zenbedded,controller-common.yaml``:

- ``domain`` (phandle, required): the rate domain the controller runs in.
- ``update-rate`` (Hz, optional): must divide the domain rate. It defaults to the
  domain rate.
- ``inactive-at-boot`` (bool): when set, autostart configures the controller but does
  not activate it. Use this for alternative controllers that claim the same
  interfaces.
- ``chained-to`` (phandle, optional): the downstream chainable controller whose
  reference interfaces this controller writes (6.3).

The P0/P1 types each add their own properties:

``zenbedded,joint-state-broadcaster``
   ``interfaces`` (string array of ``joint/iface``), ``channel`` (string).
``zenbedded,forward-command-controller``
   ``joints`` (string array), ``interface-name`` (string), ``channel`` (string),
   ``command-timeout-ms``.
``zenbedded,pid-controller``
   ``joints``, ``command-interface``, ``state-interface``, ``kp-micro``, ``ki-micro``,
   ``kd-micro``, ``i-clamp-micro``, ``feedforward`` (bool).
``zenbedded,imu-sensor-broadcaster``
   ``sensor`` (phandle to a ``zenbedded,imu-sensor``), ``channel``.

**Application controllers**

Controllers specific to one application, such as the pendulum's LQR, use exactly the
same mechanism. The application ships its own binding in its ``dts/bindings/`` (for
example ``app,inverted-pendulum-controller``) and defines the type with the same macro
(5.5). There is no separate path for application controllers.

Devicetree has no floating point, so every physical quantity is an integer in
**micro-units** (µrad, µrad/s, µrad/s², µN·m). Property names carry the unit, e.g.
``max-velocity-urad-s``, so nobody has to guess the scale factor.

A sensor's ``acquisition`` mode selects how its samples reach the framework (7.1, 8.4),
and ``stale-timeout-ms`` is the age after which consumers must treat its value as
invalid rather than merely old. Both are optional; the defaults are ``periodic`` and no
timeout.

4.2 Example (inverted pendulum, TMC2209 + AS5600)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: dts

   / {
       ros2_control: ros2-control {
           compatible = "zenbedded,controller-manager";

           fast: domain-fast {
               compatible = "zenbedded,rate-domain";
               update-rate = <400>;
               priority = <2>;
           };
           control: domain-control {
               compatible = "zenbedded,rate-domain";
               update-rate = <100>;
               priority = <4>;
           };

           motor_arm: actuator-0 {
               compatible = "zenbedded,joint-actuator";
               joint-name = "motor_joint";
               driver = <&tmc2209_ctrl>;
               domain = <&control>;
               state-interfaces = "position", "velocity";
               command-interfaces = "acceleration";
               max-velocity-urad-s = <7000000>;      /* 7.0 rad/s   */
               max-acceleration-urad-s2 = <30000000>; /* 30.0 rad/s² */
               position-limits-urad = <(-2356194) 2356194>; /* ±135° */
           };

           pendulum_encoder: sensor-0 {
               compatible = "zenbedded,joint-sensor";
               joint-name = "pendulum_joint";
               sensor = <&as5600_encoder>;
               domain = <&fast>;
               state-interfaces = "position", "velocity";
               velocity-filter-alpha-permille = <350>;
               acquisition = "periodic";
               stale-timeout-ms = <20>;
           };

           jsb: controller-0 {
               compatible = "zenbedded,joint-state-broadcaster";
               domain = <&control>;
               update-rate = <50>;
               interfaces = "motor_joint/position", "motor_joint/velocity",
                            "pendulum_joint/position", "pendulum_joint/velocity";
               channel = "zenbedded/inverted_pendulum/state";
           };

           fwd: controller-1 {
               compatible = "zenbedded,forward-command-controller";
               domain = <&control>;
               joints = "motor_joint";
               interface-name = "acceleration";
               channel = "zenbedded/inverted_pendulum/cmd";
               command-timeout-ms = <200>;
           };
       };
   };

4.3 Static instantiation
^^^^^^^^^^^^^^^^^^^^^^^^

Each component type is instantiated **per compatible**, not per child. Iterating
``DT_FOREACH_CHILD`` with a single type's macro would also try to build sensor nodes
as actuators.

.. code-block:: cpp

   #define DT_DRV_COMPAT zenbedded_joint_actuator

   #define ZB_ACTUATOR_DEFINE(inst)                                          \
     static StepperActuator zb_actuator_##inst{                             \
       DT_INST_PROP(inst, joint_name),                                       \
       DEVICE_DT_GET(DT_INST_PHANDLE(inst, driver)),                         \
       ZB_LIMITS_FROM_DT(inst)};                                             \
     ZB_COMPONENT_REGISTER(zb_actuator_##inst, DT_INST_PHANDLE(inst, domain));

   DT_INST_FOREACH_STATUS_OKAY(ZB_ACTUATOR_DEFINE)

``ZB_COMPONENT_REGISTER`` places a descriptor in a Zephyr iterable section
(``STRUCT_SECTION_ITERABLE``). The resource manager then finds every component with
``STRUCT_SECTION_FOREACH`` and needs no hand-maintained array. This gives:

- no allocation;
- a build failure if a referenced device is missing or disabled;
- joint names, limits and device pointers placed in flash.

Controllers are instantiated the same way, with one ``DT_DRV_COMPAT`` per controller
type (5.5).

4.4 Build-time checks
^^^^^^^^^^^^^^^^^^^^^

Because hardware and controllers are both in Devicetree, most configuration errors
become compile errors instead of boot-time failures. ``BUILD_ASSERT`` / ``static_assert``
checks in the generated tables cover:

- Each controller's ``update-rate`` divides its domain's rate.
- Every ``joint/iface`` named by a controller is exported by some component. String
  literals from ``DT_PROP`` are compared as ``constexpr std::string_view``.
- No two controllers that are active at boot (no ``inactive-at-boot``) claim the same
  command interface.
- No command interface is written from two domains (5.4).
- A ``chained-to`` target is a chainable type, and the chain has no cycles.

Conflicts involving ``inactive-at-boot`` controllers are checked at activation time,
the same way ``ros2_control`` does. Activation then returns ``-EBUSY``.

4.5 Kconfig coupling
^^^^^^^^^^^^^^^^^^^^

Zephyr generates a ``DT_HAS_<COMPAT>_ENABLED`` Kconfig symbol for every compatible
that has an enabled node. Each controller type's symbol defaults to that symbol, so
using a controller in the overlay is all it takes to compile it:

.. code-block:: kconfig

   config ZENBEDDED_CONTROLLER_PID
       bool "pid_controller"
       default y
       depends on DT_HAS_ZENBEDDED_PID_CONTROLLER_ENABLED

Kconfig keeps only global settings:

- ``CONFIG_ZENBEDDED_RCL_DOUBLE``
- executor thread options
- maximum interfaces
- ``CONFIG_ZENBEDDED_CM_AUTOSTART``


5. Abstractions
---------------

5.1 Dispatch without ``virtual``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``zenbedded_rcl`` has a zero-vtable rule (see ``ZenbeddedSubscriberBase``). Components and
controllers follow the same pattern Zephyr device drivers use. Each concrete type
provides one ``constexpr`` ops table in flash. Each instance stores a pointer to its
table plus its own state. Every operation returns ``0`` or a negative ``errno``.

.. code-block:: cpp

   namespace zenbedded
   {

   using time_point = k_ticks_t;   // 64-bit, monotonic; no 71-minute wrap like uint32_t µs
   using duration = k_ticks_t;

   struct ComponentOps
   {
     int (*on_configure)(HardwareComponent &);
     int (*on_activate)(HardwareComponent &);
     int (*on_deactivate)(HardwareComponent &);
     int (*on_cleanup)(HardwareComponent &);
     int (*on_error)(HardwareComponent &);          // must leave actuators in a safe state
     int (*read)(HardwareComponent &, time_point, duration);
     int (*write)(HardwareComponent &, time_point, duration);
   };

   class HardwareComponent
   {
   public:
     [[nodiscard]] LifecycleState state() const { return state_; }
     [[nodiscard]] const char * name() const { return name_; }
     // Non-virtual forwarding helpers, e.g. int read(time_point t, duration d)
     //   { return ops_->read(*this, t, d); }

   protected:
     constexpr HardwareComponent(const ComponentOps * ops, const char * name)
     : ops_(ops), name_(name) {}

   private:
     const ComponentOps * ops_;
     const char * name_;
     LifecycleState state_ = LifecycleState::UNCONFIGURED;
   };

   }  // namespace zenbedded

A ``CRTP`` helper (``ComponentBase<Derived>``) builds the ops table from member functions.
Concrete components then look like ordinary classes. Controllers use the same scheme
with a ``ControllerOps`` table whose ``update(time_point, duration)`` replaces
``read``/``write``.

5.2 Lifecycle
^^^^^^^^^^^^^

States and transitions mirror ``rclcpp_lifecycle`` as used by ``ros2_control``:

.. code-block:: text

                 on_configure            on_activate
   UNCONFIGURED ─────────────► INACTIVE ────────────► ACTIVE
        ▲          on_cleanup     │  ▲   on_deactivate   │
        └─────────────────────────┘  └───────────────────┘
   any state ── error return ──► on_error ──► UNCONFIGURED (success) | FINALIZED (failure)

Rules:

- **Ordering.** On activation, hardware goes first, then controllers. On deactivation and
  error, controllers stop first, then hardware. A controller may activate only once every
  interface it claims belongs to an ACTIVE component.
- **Who drives transitions.** At boot, ``ControllerManager`` runs
  configure→activate for everything automatically (``CONFIG_ZENBEDDED_CM_AUTOSTART``).
  At runtime, the host's ``zenbedded_hardware_interface::on_activate()`` /
  ``on_deactivate()`` can request the same transitions over a Zenoh control channel.
  This keeps the host lifecycle and the MCU lifecycle consistent.
- **Real-time context.** Only ``read``/``write``/``update`` run in rate-domain threads.
  Lifecycle callbacks run in the manager thread and may block, for example to probe an
  I2C device.

5.3 Interfaces
^^^^^^^^^^^^^^

ros2_control has three interface kinds, and so does the MCU:

- **State interfaces**: written by a component in ``read()``, read by controllers.
- **Command interfaces**: written by exactly one active controller, consumed by a
  component in ``write()``.
- **Reference interfaces**: exported by a *chainable* controller as its input, so an
  upstream controller can write to them. This is how cascades are built (6.3).

Interfaces are identified by ``joint_name/interface_name``, e.g. ``motor_joint/velocity``.
These strings match the host's names exactly, so ``controllers.yaml`` can be read across.
Each name is resolved once to an index during ``on_configure``. The real-time path never
compares strings.

.. code-block:: cpp

   using real_t = ZB_REAL_T;   // float by default; double with CONFIG_ZENBEDDED_RCL_DOUBLE

   class StateInterfaceHandle
   {
   public:
     [[nodiscard]] real_t get() const { return *value_; }
   private:
     const real_t * value_;
   };

   class CommandInterfaceHandle
   {
   public:
     void set(real_t v) { *value_ = v; }
     [[nodiscard]] real_t get() const { return *value_; }
   private:
     real_t * value_;
   };

``float`` is the default because many targets (ESP32-S3 LX7, Cortex-M4F) only have a
single-precision FPU. On those chips, ``double`` arithmetic is emulated in software and
costs roughly 10–50× more in a 1 kHz loop. Values become ``double`` only in the codec,
where ``sensor_msgs/JointState`` requires it.

5.4 Data ownership across rate domains
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Handles are plain pointers. They are only safe when reader and writer run in the
**same** thread. A 32-bit core cannot load a ``double`` atomically, and neither can a
``float`` struct, so "atomic slot updates" across threads are not possible.

The rule:

1. Every interface value has exactly one owning domain, the domain of the component or
   controller that writes it.
2. When a value is read by another domain, the manager creates a *mirror* in the reader
   domain. At the start of each reader tick, the manager copies the writer's latest
   snapshot into the mirror through a ``DoubleBuffer<Snapshot>``. Only these copies take
   the spinlock, never user code.
3. The build fails, via a ``static_assert`` in the generated tables, if two domains
   claim the same command interface.
4. A value that arrives from a notification-driven sensor (8.4) carries its acquisition
   timestamp and a validity flag. A consumer may use it only while
   ``now - acquired_at`` is within the component's ``stale-timeout-ms``; an expired value
   is a fault to report, not a slightly old reading. Acquisition time, not publish time,
   is what the host needs for fusion in Tier 1.

These rules make the existing spinlocked ``DoubleBuffer`` sufficient. No second
concurrency primitive is needed.

5.5 Controllers
^^^^^^^^^^^^^^^

A controller *type* is written once, like a Zephyr driver. Its *instances* come from
Devicetree (4.1):

.. code-block:: cpp

   #define DT_DRV_COMPAT zenbedded_pid_controller

   #define ZB_PID_PARAMS(inst)                                       \
     PidController::Params{                                          \
       .joints = ZB_DT_STRING_ARRAY(inst, joints),                   \
       .command_interface = DT_INST_PROP(inst, command_interface),   \
       .state_interface = DT_INST_PROP(inst, state_interface),       \
       .kp = ZB_MICRO(DT_INST_PROP(inst, kp_micro)),                 \
       .ki = ZB_MICRO(DT_INST_PROP_OR(inst, ki_micro, 0)),           \
       .kd = ZB_MICRO(DT_INST_PROP_OR(inst, kd_micro, 0)),           \
     }

   DT_INST_FOREACH_STATUS_OKAY_VARGS(ZB_CONTROLLER_DT_INST_DEFINE, PidController, ZB_PID_PARAMS)

``ZB_CONTROLLER_DT_INST_DEFINE(inst, Type, params_macro)`` has four jobs:

- create a static ``Type`` instance holding its ``constexpr`` ``Params``;
- place a descriptor in the controllers' iterable section, carrying the domain,
  divisor, ``inactive-at-boot`` and ``chained-to``;
- emit the build-time checks in 4.4;
- use the node label as the controller's name (``jsb``, ``fwd``), as in
  ``controllers.yaml``.

``ZB_MICRO`` converts micro-units to ``real_t`` at compile time.

A controller declares the interfaces it needs through
``command_interface_configuration()`` and ``state_interface_configuration()``. These
return names built from its ``Params``, matching the host API.

**Parameters later.** Devicetree integers suit limits and rates but are clumsy for
gains. A later phase (11, P7) can generate the ``Params`` struct from a
``controllers.yaml``, preferably the same file the host ``controller_manager`` loads.
That would allow real-number values and identical parameter names on host and MCU. The
Devicetree node would then only bind the instance to its domain and interfaces. All of
this is compile-time; runtime-settable parameters remain open question 3.

5.6 ControllerManager and ResourceManager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``ResourceManager``

- owns component descriptors and interface storage;
- resolves names to indices;
- tracks which controller has claimed each command interface (one claimant per
  interface);
- applies joint limits in the write path, before ``write()``, so limits hold whichever
  controller is active. This mirrors where newer ros2_control enforces them.

``ControllerManager``

- owns the rate domains and runs lifecycle transitions (5.2);
- on each domain tick, runs ``read`` for that domain's components, then mirror copies,
  then ``update`` for its controllers (in chain order), then ``write``;
- runs the **command watchdog**: if no command arrives within ``command-timeout-ms``,
  the target command interfaces drop to a safe value (zero velocity or acceleration) and
  a diagnostic flag is set. This replaces the demo-local ``kCommandTimeoutMs`` logic;
- counts overruns per domain (a tick taking longer than its period) and reports them on
  the diagnostics channel.

5.7 Controller porting plan
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Upstream names are kept wherever behaviour matches. This lets a host
``controllers.yaml`` entry move to the MCU with minimal edits.

.. list-table::
   :header-rows: 1
   :widths: 28 12 60

   * - Upstream controller
     - Priority
     - MCU port
   * - ``joint_state_broadcaster``
     - P0
     - Writes claimed state interfaces into a ``ZenbeddedPublisher<JointStateCodec>``
       (or a Tier 2 ``RawCodec``) at its own rate. Replaces the hand-written state
       publishing in each demo.
   * - ``forward_command_controller``
     - P0
     - Reads ``ZenbeddedSubscriber<JointCommandCodec>::take()`` each tick and writes
       the values to claimed command interfaces. Uses ``accepted_count()`` for the
       watchdog. This is the earlier "CommandSubscriberController", renamed for parity.
   * - ``pid_controller``
     - P1
     - Chainable. Exports ``<joint>/<iface>`` reference interfaces. Feed-forward and
       anti-windup follow ``control_toolbox::Pid``.
   * - ``imu_sensor_broadcaster``
     - P1
     - Publishes ``sensor_msgs/Imu`` on its own channel.
   * - ``joint_trajectory_controller``
     - P2
     - Subset only: a ring buffer of waypoints plus cubic or quintic interpolation (6.4).
       No action server; the host keeps the action and streams waypoints.
   * - ``diff_drive_controller``
     - P2
     - Kinematics and odometry on the MCU for mobile bases. Odometry published as
       ``nav_msgs/Odometry``.
   * - Not ported
     - —
     - ``admittance_controller``, ``gripper_action_controller``, MoveIt-side controllers.
       They need actions, heavy maths or the full robot model.

Safety clamping is **not** a controller. It is the limits stage of the write path
(5.6). Every command passes through it, so no controller can bypass it.


6. Multi-Rate Scenarios
-----------------------

6.1 Fast sensing, slower control, downsampled telemetry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Inverted pendulum:

- AS5600 sampled and velocity-filtered at 400 Hz in the ``fast`` domain.
- Control at 100 Hz in the ``control`` domain.
- State published at 50 Hz by ``joint_state_broadcaster`` running in ``control`` with a
  divisor of 2.

``pendulum_joint/*`` is owned by ``fast`` and mirrored into ``control`` once every 10 ms.

6.2 Heterogeneous bus timings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A 1 kHz step or PWM actuator, a 200 Hz I2C IMU and a 1 Hz battery ADC. If they shared a
thread, a 3 ms I2C transaction would make the 1 kHz actuator overrun its period. The fix
is to give each one its own domain. This is exactly the problem domains exist to solve.

6.3 Cascaded controllers
^^^^^^^^^^^^^^^^^^^^^^^^

A 100 Hz trajectory controller writes the reference interface of a 1 kHz velocity
``pid_controller``. That controller writes the actuator's ``effort`` command. The FOC
current loop stays inside the driver (see :ref:`rcl_arch_status`, out of scope). Because
the cascade crosses a domain boundary, the reference value is mirrored as described
in 5.4.

6.4 Host trajectory, MCU interpolation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The host streams waypoints at 50 Hz. The MCU ``joint_trajectory_controller`` subset
interpolates at 500 Hz. If a waypoint arrives late, the controller decelerates within
the acceleration limits. If it is later than ``command-timeout-ms``, the watchdog
(5.6) takes over.


7. Scheduling
-------------

7.1 One model: rate domains
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The earlier draft offered three alternative models. They collapse into one:

- **A rate domain** is one Zephyr thread with a fixed period. Inside a domain, members
  run at integer divisors of the domain rate.
- **Single-thread setup** (the old "Model A") is the special case of one domain. Its
  tick rate must be the *least common multiple* of the member rates. Equivalently, the
  tick period is the greatest common divisor of the periods. For 1000, 200, 100 and 50 Hz
  that is a 1 kHz tick with divisors 1, 5, 10 and 20. The GCD of the *frequencies*
  (50 Hz) would be wrong.
- **Multi-domain setup** (the old "Model B") is the default for anything with more than
  one bus.
- **ISR and DMA capture** (the old "Model C") is a driver implementation detail inside a
  ``HardwareComponent``, not a framework mode. Acquisition is either *rate-driven* (the
  owning domain reads the sink on its tick) or *notification-driven* (a data-ready or
  FIFO-watermark handler wakes the owning domain early). In both cases the handler does
  no bus I/O and no logging: ISRs only raise, threads read (8.4).

7.2 Tick timing
^^^^^^^^^^^^^^^

Domain threads sleep to an **absolute** deadline so that errors do not accumulate. A
relative ``k_sleep(K_MSEC(period))`` after the work drifts by the work's execution time
on every tick.

.. code-block:: cpp

   int64_t next = k_uptime_ticks();
   for (;;) {
     next += domain.period_ticks;
     run_tick(domain);
     if (k_uptime_ticks() > next) { ++domain.overruns; }
     k_sleep(K_TIMEOUT_ABS_TICKS(next));   // requires CONFIG_TIMEOUT_64BIT
   }

(A periodic ``k_timer`` plus ``k_timer_status_sync()`` would also work. Either way,
overruns are counted.)

A finer tick improves ``k_sleep`` granularity, but every tick is a wake-up, so raising
``CONFIG_SYS_CLOCK_TICKS_PER_SEC`` must be weighed against idle power on battery-powered
nodes (``CONFIG_TICKLESS_KERNEL`` keeps idle ticks suppressed). The domain period and the
tick rate are otherwise independent: an absolute-deadline sleep lands on the intended
boundary as long as the tick is finer than the target jitter. The value belongs in P4
(11.3), chosen from :doc:`benchmarking` measurements.

7.3 Priorities
^^^^^^^^^^^^^^

Faster domains get higher priority (rate-monotonic). The Zenoh RX/lease threads and the
``ZenbeddedExecutor`` sit *below* every control domain but above logging and background
work. A network stall must never preempt control, and control must not starve the Zenoh
lease long enough for the session to drop. Exact priority numbers go in Kconfig, not in
this document.

.. note::

   Jitter targets (for example < 50 µs on a 1 kHz domain on ESP32-S3) are to be measured
   with the harness in :doc:`benchmarking` before being quoted as properties of the
   design.


8. Network Integration and ZenbeddedExecutor
--------------------------------------------

8.1 Channels
^^^^^^^^^^^^

- Broadcaster and forwarder controllers are the only controllers that touch the network
  layer. All other controllers see plain interfaces.
- Channels follow the project's ``zenbedded/<robot>/<channel>`` convention, e.g.
  ``zenbedded/inverted_pendulum/state`` and ``zenbedded/inverted_pendulum/cmd``.
  Further channels (``imu``, ``diagnostics``, ``lifecycle``) follow the same pattern.
  The Tier 1 or Tier 2 encoding is chosen per channel by its codec.

8.2 Why an executor (issue #120)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With PR #106 merged:

- ``internal_sub_handler`` in ``zenoh_transport.c`` copies each sample into a static
  ``rx_buf``. ``ZenbeddedSubscriberBase::on_transport_cb`` then writes it into the
  subscriber's ``DoubleByteBuffer``, increments ``accepted_count`` and calls the user
  callback. All of this happens **inside the zenoh-pico read task**, so a slow user
  callback stalls packet reception and the lease task.
- ``ZenbeddedPublisher::start_periodic_publish`` runs on the system workqueue. It calls
  ``k_work_schedule(period)`` again *after* publishing, so each period is stretched by
  the publish time and the rate drifts.
- ``ZenbeddedClient``, still used by the demos, has its own publish thread with
  ``period_ms = 1000 / freq``. That runs fast for rates that do not divide 1000 (400 Hz
  becomes 500 Hz). This is latent: every current demo uses 100 Hz.
- User callbacks (read task), periodic publishers (system workqueue) and the
  application's own loop are three contexts touching the same state.

The executor fixes all four, and it also gives the ``ControllerManager`` the
"manager thread" that section 5.2 needs. PR #106 already has most of the pieces: the
per-subscriber buffer, ``take()``, ``accepted_count()`` and the static-thunk dispatch.
The executor mainly *moves where the callback runs*.

8.3 Rate domains vs. the executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Two schedulers must not compete for the same work. The split is:

.. list-table::
   :header-rows: 1
   :widths: 22 39 39

   * -
     - Rate domain (section 7)
     - ``ZenbeddedExecutor``
   * - Driven by
     - Time (fixed period, absolute deadline)
     - Events (sample arrived, timer due, cancel)
   * - Runs
     - ``read`` / ``update`` / ``write``, mirrors, watchdog
     - Subscription callbacks, telemetry timers, lifecycle and parameter requests
   * - May block
     - Never
     - Briefly (e.g. an I2C probe during ``on_configure``)
   * - Priority
     - Highest band
     - Below the control domains and zenoh-pico tasks

Real-time code never runs inside the executor. Controllers read their inputs from
buffers that executor callbacks (or the transport directly) fill. The command watchdog
stays in the control domain, so a stalled executor still triggers a safe stop.

An application that does not use ``ControllerManager`` (such as the current demos) can
use the executor on its own, exactly as issue #120 sketches.

8.4 Design
^^^^^^^^^^

**Executables.** ``ZenbeddedExecutor<N>`` holds up to ``N`` executables (``N <= 31``)
in a fixed array. Subscriptions, timers and guard conditions (manual triggers) are all
executables. Dispatch uses a static thunk, the same pattern as
``ZenbeddedSubscriberBase``, with no ``virtual``.

**Wake-up.** One ``k_event`` per executor, with one bit per executable and bit 31
meaning "cancel":

- Subscriptions are PR #106's ``ZenbeddedSubscriber<Codec>``. ``executor.add(sub)``
  hands the subscriber its executor and bit index. From then on,
  ``on_transport_cb`` writes the buffer and calls ``k_event_post(&ev, BIT(idx))``
  instead of the user callback. It does no decoding and runs no user code. A subscriber
  that is not added to an executor keeps #106's inline behaviour, so existing code does
  not change.
- On dispatch, the executor calls ``take()`` and passes the decoded ``Value`` to the
  same ``CallbackFn`` signature #106 already uses.
- The executor waits in ``k_event_wait(&ev, mask, false, timeout)``, where ``timeout`` is
  the absolute deadline of the earliest timer. A single wait therefore covers both
  messages and timers, with no polling.
- After waking, it clears the bits it is about to handle and dispatches **in
  registration order**. Order is fixed at registration, so it is deterministic.

**Subscription buffering (depth).** Depth is chosen per subscription:

- **Depth 1** (default) is #106's existing ``DoubleByteBuffer``, which keeps only the
  latest value. If
  two samples arrive before dispatch, the callback sees only the newer one. This is the
  right behaviour for state and commands, and matches a ``KEEP_LAST(1)`` QoS.
- **Depth N** is a new template parameter, ``ZenbeddedSubscriber<Codec, Depth>``,
  backed by a ``k_msgq`` of ``N`` fixed-size slots, so nothing is coalesced. This
  is for streams where every sample matters, such as trajectory waypoints (6.4). When
  the queue is full, the oldest sample is dropped and a counter is incremented.

**Timers.**

- ``ZenbeddedTimer`` keeps an absolute next-deadline in ticks and advances it by whole
  periods, so it never drifts.
- If dispatch was late by more than one period, the callback runs **once**. The missed
  periods are skipped and counted, which is what ``rclcpp`` does.
- ``start_periodic_publish`` stays as API, but is reimplemented as a timer on the
  default executor. This removes the workqueue drift. ``ZenbeddedClient``'s publish
  thread becomes a timer too.

**Sources (notification-driven and slow sensors).** A sensor whose driver announces
data-ready or crosses a FIFO watermark registers an executor *source*: one fixed
callback and one event bit. The driver's handler, including an ISR, does nothing but
``k_event_post(&ev, BIT(idx))``; the executor reads the sample on its own thread. Two
sinks cover the acquisition policies:

- ``LatestValue<T>`` keeps the newest sample with an acquisition timestamp and a
  validity flag. A notification then only shortens latency; it never queues. This is the
  sink a rate-driven domain polls (7.1).
- ``SampleQueue<T, N>`` is a bounded SPSC ring for bursts where every sample matters, for
  example an IMU FIFO. When full it drops the oldest sample and counts the drop.

The number of events dispatched per pass is capped, so an event storm degrades by
dropping samples rather than by delaying the executor. A sensor that can block for longer
than a domain period does not get a source: it gets its own low-priority domain (6.2),
which owns the bus and writes a ``LatestValue<T>``. Consumers compare that slot's
acquisition timestamp against ``stale-timeout-ms`` (4.1).

Subscription *depth*, described above, is inbound buffering. These sinks are the
acquisition side, where the producer is a driver rather than the Zenoh read task.

**Spinning.**

- ``spin()`` runs forever on the executor thread.
- ``spin_some(timeout, max_events)`` handles at most ``max_events`` events and then
  returns. This is for simple applications that drive the executor from their own loop.
  The cap bounds how long one call can take, which matters if that loop is also the
  control loop.
- ``cancel()`` posts bit 31.

**Autonomous mode.**

- ``CONFIG_ZENBEDDED_EXECUTOR_THREAD`` provides a default executor whose thread is
  defined with ``K_THREAD_DEFINE(..., K_FOREVER)``. #106's ``ZenbeddedNode::init()``
  starts it once ``zenbedded_transport_init`` succeeds.
- Issue #120 suggests starting it from ``SYS_INIT``. That runs before ``main()``, before
  the network is up, and before C++ static objects are constructed, so the thread
  would have to wait for all three anyway. Starting it on demand is simpler.

**Registration.**

- ``add_*`` is allowed before the first spin. After that it returns ``-EBUSY``, so the
  executable table never changes while dispatch is running.
- Removal happens only through ``cancel()`` followed by reconstruction. This matches
  the lifecycle's cleanup step.

.. code-block:: cpp

   ZenbeddedNode node;
   ZenbeddedExecutor<8> executor;

   ZenbeddedSubscriber<JointCommandCodec> cmd_sub;           // depth 1 (default)
   node.init();
   node.create_subscriber(cmd_sub, "zenbedded/inverted_pendulum/cmd", on_cmd);

   ZenbeddedTimer telemetry{K_MSEC(20), on_telemetry};       // replaces start_periodic_publish

   executor.add(cmd_sub);
   executor.add(telemetry);
   executor.spin();   // or: start the default executor thread

8.5 Priorities
^^^^^^^^^^^^^^

From highest to lowest: control rate domains, then zenoh-pico read and lease tasks,
then the executor, then logging and background work.

- The read task sits above the executor so that RX copies finish quickly. Its work per
  sample is bounded by the RX buffer size.
- It sits below the control domains, so network bursts cannot delay control.
- ``zenbedded_transport_init`` currently passes default ``zp_task_*_options``. These
  must be set explicitly, because the actual default priority depends on the zenoh-pico
  Zephyr port.

8.6 Later optimisation
^^^^^^^^^^^^^^^^^^^^^^

Each RX sample is copied twice: first into the transport's static ``rx_buf``, then into
the subscription buffer. A receive-into callback (``zenbedded_recv_into_cb_t``) could
let ``z_bytes_reader_read`` write straight into the subscription's write slot. This
changes the C transport API, so it is deferred until benchmarks show the extra copy
matters.

8.7 Tests (``native_sim``)
^^^^^^^^^^^^^^^^^^^^^^^^^^

- Callbacks run on the executor thread: assert on ``k_current_get()``.
- The RX handler returns within a bounded time with a deliberately slow user callback
  registered.
- Depth 1 coalesces and depth N preserves order, including the drop-oldest counter.
- A timer shows no cumulative drift over 10 000 periods, and a forced late dispatch
  counts the skipped periods.
- ``cancel()`` unblocks ``spin()`` promptly, and ``add`` after spin returns ``-EBUSY``.
- A source posted from a simulated ISR context is dispatched on the executor thread, and
  a slow handler cannot block a second source.
- A ``LatestValue`` sink behind a slow domain reports stale once its timeout passes, and
  a full ``SampleQueue`` drops the oldest sample and increments its counter.
- With more events pending than the per-pass cap, the surplus is deferred to the next
  pass rather than dropped.


9. Reference Implementation: Inverted Pendulum
----------------------------------------------

**Today** (``demos/inverted_pendulum``):

- ``main.cpp`` runs a 400 Hz AS5600 sampling thread and a 100 Hz loop.
- The loop reads the commanded *acceleration* from the host, integrates it into stepper
  velocity, enforces the ±135° travel limit and a command timeout, and publishes state.
- The LQR and energy-shaping controller runs on the host as
  ``inverted_pendulum_controller``, loaded by ``controller_manager`` next to
  ``joint_state_broadcaster``.

**After migration:**

1. The overlay in 4.2, now including the ``jsb`` and ``fwd`` controller nodes, replaces
   the hand-written device lookups.
2. ``StepperActuator`` (accepts acceleration commands, integrates to velocity, uses
   ``stepper_ctrl_move_to``) and ``As5600JointSensor`` (filtered velocity) become
   reusable components.
3. ``joint_state_broadcaster`` and ``forward_command_controller`` replace the
   hand-written publish and subscribe code.
4. The watchdog and travel limit move into ``ControllerManager`` / ``ResourceManager``.
5. The host side is unchanged: the same ``zenbedded_hardware_interface``, the same
   topics and the same controllers.
6. Optional later step: port the LQR as an MCU controller and switch the host to a
   ``forward_command_controller``. This lets the two placements be compared directly.


10. Open Questions
------------------

1. **Ops tables vs. virtual.** An ops-table pointer costs the same as a vptr. The gain
   is ``constexpr`` placement and no RTTI, not size. Confirm the zero-vtable rule should
   cover framework objects too, not just pub/sub.
2. **Lifecycle control channel.** Should it be a Zenoh queryable (request/reply) or a
   plain topic with an acknowledgement in diagnostics?
3. **Parameters.** Should they stay compile-time only, or have a small runtime-settable
   subset (PID gains) over Zenoh?
4. **Float default.** Confirm ``float`` is acceptable for the pendulum's LQR if it moves
   onto the MCU.
5. **Executor capacity.** Is 31 executables per executor (one ``k_event`` word) enough,
   or is a second word or a second executor needed for large robots?
6. **Source registration.** Does a notification-driven sensor register its event bit in
   code, or does the Devicetree binding declare the source (4.1) so the bit is
   generated? The latter keeps Devicetree as the single declaration site at the cost of
   one more binding property.
7. **Tick rate.** Which ``CONFIG_SYS_CLOCK_TICKS_PER_SEC`` and ``CONFIG_TICKLESS_KERNEL``
   setting do the target boards settle on, given the idle-power tradeoff in 7.2?


11. Delivery Plan
-----------------

The design is delivered in eight phases, starting from ``master`` with PR #106 merged. Each phase is one PR, or a short stack of PRs
following the project's ``gh stack`` workflow. Each phase leaves ``master`` working and
ends with something testable. The phases are ordered so that the inverted pendulum moves
onto the framework as early as possible, and every later phase is proven on real
hardware.

11.1 Dependencies
^^^^^^^^^^^^^^^^^

.. code-block:: text

   (master + PR #106)
          │
          ▼
   P0 Executor ──► P1 Core types ──► P2 Managers + JSB/FCC ──► P3 Devicetree
                                            │                        │
                                            │                        ▼
                                            │                  P4 Rate domains
                                            │                        │
                                            ▼                        ▼
                                  P6 Lifecycle channel      P5 PID / IMU / chaining
                                                                     │
                                                                     ▼
                                                            P7 JTC / diff-drive / params

P6 needs only the managers (P2) and the executor (P0), so it can run in parallel with
P3–P5.

11.2 Milestones
^^^^^^^^^^^^^^^

- **M1 (end of P2)**: the pendulum balances through ``ControllerManager``, with
  controllers registered in code. The host side is unchanged.
- **M2 (end of P3)**: the pendulum is fully declared in Devicetree (4.2). ``main.cpp``
  contains no device lookups and no publish, subscribe or watchdog code.
- **M3 (end of P4)**: multi-rate execution, with jitter measured and published in
  :doc:`benchmarking`.
- **M4 (end of P5)**: a chained cascade (6.3) runs on hardware.

11.3 Phases
^^^^^^^^^^^

**P0: ZenbeddedExecutor (issue #120).**

- *Depends on*: nothing beyond the baseline (``master`` with PR #106 merged).
- *Work*:

  - ``zenbedded_rcl/include/zenbedded_rcl/executor.hpp`` and ``timer.hpp``: the section
    8.4 design.
  - Wake-up hook in ``zenoh_transport.c``.
  - Kconfig for the default executor thread and for zenoh-pico task priorities (8.5).
  - Attach ``ZenbeddedSubscriber`` to an executor (event post instead of an inline
    callback).
  - Add the ``Depth`` template parameter.
  - ``add_source`` and the ``LatestValue<T>`` / ``SampleQueue<T, N>`` sinks (8.4).
  - Reimplement ``start_periodic_publish`` and ``ZenbeddedClient``'s publish thread as
    timers.

- *Done when*:

  - The tests in 8.7 pass in Twister on ``native_sim``.
  - The ``sine_wave`` and pendulum demos run unchanged from the user's point of view.
  - A 400 Hz publish timer measures 400 Hz, not 500 Hz.

- *Decisions*: open question 5 (executor capacity).

**P1: Core types.**

- *Depends on*: P0. Lifecycle transitions run on the executor thread.
- *Work*:

  - ``lifecycle.hpp``.
  - ``interfaces.hpp``: handles and ``real_t``.
  - ``hardware_component.hpp``: replace the empty header.
  - ``controller_interface.hpp``.
  - CRTP helpers that build the ops tables.
  - New Ztest suite ``tests/rcl_core``.

- *Done when*:

  - Every lifecycle transition, including error paths and ``on_error``, is covered by a
    test.
  - The test binary contains no ``_ZTV*`` (vtable) symbols, checked with ``nm`` in CI.

- *Decisions*: open questions 1 (vtable rule) and 4 (``float``).

**P2: ResourceManager, ControllerManager, P0 controllers.**

- *Depends on*: P1.
- *Work*:

  - ``ResourceManager``: interface table, claiming, the limits stage of the write path.
  - ``ControllerManager``: single domain, autostart, command watchdog.
  - ``joint_state_broadcaster`` and ``forward_command_controller`` on top of the
    pub/sub API.
  - A low-level ``ZB_CONTROLLER_DEFINE`` registration macro. Tests keep using it after
    P3.
  - Port the pendulum tier 1 demo, with ``StepperActuator`` and ``As5600JointSensor``
    written in the demo for now.

- *Done when*:

  - The pendulum balances on hardware with the host LQR.
  - ``test_inverted_pendulum_tier1_launch.py`` passes unchanged.
  - Killing the host stops the stepper within ``command-timeout-ms``.
  - The demo's hand-written publish, subscribe and watchdog code is deleted.

**P3: Devicetree front end.**

- *Depends on*: P2.
- *Work*:

  - Bindings (4.1) and ``dts_root`` in ``zephyr/module.yml``.
  - ``DT_DRV_COMPAT`` instantiation for components and controllers (4.3, 5.5).
  - Build-time checks (4.4) and Kconfig coupling (4.5).
  - Move the generic stepper actuator and sensor-API joint sensor into
    ``zenbedded_rcl``.
  - Port the pendulum tier 2 demo as well.

- *Done when*:

  - Both pendulum tiers build from the overlay in 4.2 alone.
  - A pytest runs ``west build`` on a set of deliberately invalid overlays and expects
    each to fail with the matching assertion. Examples: a rate that does not divide the
    domain rate, an unknown interface, two boot-active writers.
  - A controller type absent from the overlay adds no code. Compare ``size`` output
    with and without it.

**P4: Multiple rate domains.**

- *Depends on*: P3.
- *Work*:

  - Domain threads with absolute deadlines (7.2).
  - Mirrors through ``DoubleBuffer`` (5.4).
  - Per-domain overrun counters on a ``diagnostics`` channel.
  - OFFLOAD domains for buses that can block (6.2), writing ``LatestValue`` sinks and
    reporting staleness on the diagnostics channel.
  - Benchmark harness.

- *Done when*:

  - The pendulum encoder runs in the ``fast`` domain through the framework, and the
    demo's own encoder thread is deleted.
  - Per-domain jitter histograms on ESP32-S3 are recorded in :doc:`benchmarking`.
  - Only measured jitter numbers replace the targets in 7.3.

**P5: Chaining and P1 controllers.**

- *Depends on*: P4, for cross-domain cascades.
- *Work*:

  - Reference interfaces and ``chained-to``.
  - ``pid_controller`` and ``imu_sensor_broadcaster``.
  - Depth-N subscriptions (8.4).

- *Done when*:

  - A ``native_sim`` cascade test (trajectory → PID → mock actuator) tracks a step
    within tolerance.
  - One demo uses ``pid_controller`` on hardware.

**P6: Lifecycle control channel.**

- *Depends on*: P0 and P2.
- *Work*:

  - On the host, ``zenbedded_hardware_interface::on_activate`` / ``on_deactivate``
    send requests.
  - On the MCU, an executor subscription or queryable drives ``ControllerManager``
    transitions.

- *Done when*:

  - ``ros2 control set_hardware_component_state <hw> inactive`` stops the MCU's
    actuators, and ``active`` resumes them.
  - A test asserts the round-trip.

- *Decisions*: open question 2.

**P7: P2 controllers and generated parameters.**

- *Depends on*: P5.
- *Work*:

  - ``joint_trajectory_controller`` subset and ``diff_drive_controller``.
  - A ``controllers.yaml`` → ``Params`` header generator (5.5), reusing the
    ``zenbedded_schema`` code generation.
  - Optional: port the pendulum LQR as an application controller and compare host vs.
    MCU placement (section 9, step 6).

- *Decisions*: open question 3.

11.4 Every phase also includes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- A Twister scenario for the new code on ``native_sim``. Phases that touch a demo also
  get an ESP32-S3 build-only scenario.
- Updates to this document: move the finished item from "Proposed" to "Today" in
  :ref:`rcl_arch_status`, and update ``zenbedded_rcl/doc/userdoc.rst``.
- The pre-commit hooks used across the repo (clang-format, cpplint, cppcheck,
  codespell, license headers).
- A GitHub issue per phase, linked from #120 and from this section, so progress is
  visible outside the document.


.. _Zephyr RTOS: https://zephyrproject.org/
.. _Zenoh: https://zenoh.io/
