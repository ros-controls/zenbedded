// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOCK_PENDULUM_HARDWARE__MOCK_PENDULUM_HARDWARE_HPP_
#define MOCK_PENDULUM_HARDWARE__MOCK_PENDULUM_HARDWARE_HPP_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#ifdef MOCK_PENDULUM_WITH_RAYLIB
#include "mock_pendulum_hardware/pendulum_visualizer.hpp"
#endif

namespace mock_pendulum_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Mock hardware for a rotary (Furuta) inverted pendulum.
///
/// Angle conventions (chosen to match the controller, NOT the URDF joint zero):
///   q1 (motor_joint)    : arm angle, 0 = wherever the operator centred the arm,
///                         mechanically limited to +-135 deg by the lid bosses.
///   q2 (pendulum_joint) : 0 = UPRIGHT, +-pi = hanging. Always reported wrapped
///                         into [-pi, pi].
///
/// The URDF's `arm_to_pendulum` zero is the hanging pose; this interface offsets
/// that by pi so the controller sees the usual "error = q2" upright convention.
class MockPendulumHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- parameter helpers -------------------------------------------------
  double param(const std::string & name, double fallback) const;
  bool param_bool(const std::string & name, bool fallback) const;

  // ---- plant -------------------------------------------------------------
  /// Angular acceleration of the pendulum about its own hinge, given the
  /// *realised* arm acceleration (post-limit), in rad/s^2.
  double pendulum_accel(double q2, double q2_dot, double q1_dot, double q1_ddot) const;

  /// Advance the true state by one fixed sub-step of `h` seconds of sim time.
  void integrate(double h);

  /// Apply the sensor model (quantisation, noise, finite differencing) and
  /// publish into hw_states_.
  void publish_states(double dt_sim);

  static double wrap_pi(double a);

  // ---- physical parameters (post randomisation) --------------------------
  double J1_{6.10e-5};         ///< arm inertia about the motor axis  [kg m^2]
  double J2_pivot_{4.447e-5};  ///< pendulum inertia about its hinge  [kg m^2]
  double J2_com_{8.06e-6};     ///< pendulum inertia about its COM    [kg m^2]
  double m2_{0.014};           ///< pendulum mass                     [kg]
  double l2_{0.051};           ///< hinge -> pendulum COM             [m]
  double L1_{0.062};           ///< motor axis -> hinge (arm reach)   [m]
  double b1_{1.0e-4};          ///< arm viscous friction              [N m s/rad]
  double b2_{8.0e-6};          ///< pendulum viscous friction         [N m s/rad]
  double c2_{2.0e-6};          ///< pendulum Coulomb friction         [N m]
  double g_{9.80665};

  // nominal (pre-randomisation) copies, so re-activation re-rolls from nominal
  double J1_nom_{0.0}, J2_com_nom_{0.0}, m2_nom_{0.0}, l2_nom_{0.0};
  double b1_nom_{0.0}, b2_nom_{0.0}, c2_nom_{0.0};

  // ---- actuator limits ---------------------------------------------------
  double motor_limit_{2.35619449};  ///< +-135 deg hard stop           [rad]
  double motor_max_vel_{5.0};       ///< step-rate ceiling             [rad/s]
  double motor_max_accel_{200.0};   ///< commanded accel clamp         [rad/s^2]
  double motor_max_torque_{0.4};    ///< holding torque, step loss     [N m]
  bool model_step_loss_{true};
  double stop_restitution_{0.0};  ///< 0 = dead stop, 0.1 = slight bounce

  // ---- sensor model ------------------------------------------------------
  double motor_steps_per_rev_{6400.0};
  double encoder_counts_per_rev_{4096.0};
  double encoder_noise_{0.0004};  ///< 1-sigma, rad
  double encoder_offset_{0.0};    ///< upright is never exactly the zero count
  bool pendulum_vel_from_encoder_{true};
  double vel_filter_alpha_{0.5};
  double motor_vel_noise_{0.002};
  double report_offset_{0.0};  ///< added to the reported pendulum angle

  // ---- process noise -----------------------------------------------------
  double pendulum_accel_noise_{0.03};  ///< 1-sigma, rad/s^2 (draughts, imbalance)
  double motor_accel_noise_{0.02};     ///< 1-sigma, rad/s^2 (micro-step jitter)
  double param_randomization_{0.02};   ///< 1-sigma fractional plant scatter
  double initial_q2_{3.14159265358979};
  double initial_q2_jitter_{0.01};

  // ---- integration / timing ---------------------------------------------
  double sim_speed_{1.0};  ///< sim seconds per wall second
  double max_substep_{5.0e-4};
  double max_cycle_dt_{0.1};
  int max_substeps_{4000};

  // ---- true state --------------------------------------------------------
  double q1_{0.0}, q1_dot_{0.0};
  double q2_{0.0}, q2_dot_{0.0};
  double q2_ddot_prev_{0.0};
  double sim_time_{0.0};
  bool at_limit_{false};
  bool step_loss_{false};

  // ---- measured state ----------------------------------------------------
  double q2_meas_prev_{0.0};
  double q2_dot_filt_{0.0};
  bool have_prev_meas_{false};

  std::vector<double> hw_states_;
  std::vector<double> hw_commands_;

  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

#ifdef MOCK_PENDULUM_WITH_RAYLIB
  std::unique_ptr<PendulumVisualizer> viz_;
#endif
  bool visualize_{false};
};

}  // namespace mock_pendulum_hardware

#endif  // MOCK_PENDULUM_HARDWARE__MOCK_PENDULUM_HARDWARE_HPP_
