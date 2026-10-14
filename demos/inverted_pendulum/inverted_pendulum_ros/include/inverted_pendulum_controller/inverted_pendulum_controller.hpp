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

#ifndef INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_
#define INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace inverted_pendulum_controller
{

/**
 * @brief Hybrid energy-shaping swing-up and LQR balance controller for a Furuta rotary inverted
 * pendulum.
 *
 * This controller orchestrates two operating regimes:
 * 1. Non-linear Energy Shaping (Swing-up): Bang-bang energy pump driving the pendulum
 *    mechanical energy toward upright potential energy, combined with bounded arm-centering PD.
 * 2. Full-State LQR (Balancing): Regulates the arm position, arm velocity, pendulum angle,
 *    and pendulum velocity around the unstable upright equilibrium (q2 = 0).
 *
 * Transitions between swing-up and balance are protected by candidate angular windows,
 * actuator command feasibility gating, soft-start ramp blending, and fall hysteresis.
 */
class InvertedPendulumController : public controller_interface::ControllerInterface
{
public:
  InvertedPendulumController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Returns true if currently in LQR balancing regime.
  bool is_balancing() const { return balancing_; }

private:
  // Hardware interface names
  std::string motor_joint_state_name_;
  std::string motor_joint_vel_name_;
  std::string pendulum_joint_state_name_;
  std::string pendulum_joint_vel_name_;
  std::string motor_joint_command_name_;

  // Physical plant constants
  double m2_{0.014};      ///< Pendulum mass [kg]
  double l2_{0.051};      ///< Pendulum hinge to COM distance [m]
  double L1_{0.062};      ///< Motor axis to pendulum hinge reach [m]
  double J2_{4.447e-5};   ///< Pendulum moment of inertia about pivot [kg*m^2]
  double g_{9.80665};     ///< Gravitational acceleration [m/s^2]
  double E_target_{0.0};  ///< Upright potential energy target (m2 * g * l2) [J]

  // Full-state LQR gains: u = -K * [q1, q1_dot, q2, q2_dot]
  double k_q1_{-5.000};
  double k_q1_vel_{-4.352};
  double k_q2_{418.238};
  double k_q2_vel_{33.281};

  // Energy-shaping swing-up parameters
  double k_e_{80.0};               ///< Energy gain multiplier
  double k_arm_{31.0};             ///< Arm P-recentering gain during swing-up [rad/s^2 per rad]
  double k_arm_vel_{6.0};          ///< Arm D-recentering gain during swing-up [rad/s^2 per rad/s]
  double swing_accel_max_{100.0};  ///< Maximum swing-up pump acceleration [rad/s^2]
  double arm_zone_{2.0};           ///< Arm angle limit beyond which pump is suppressed [rad]
  double swing_start_vel_{
    0.01};  ///< Angular velocity threshold for dead-start directional kick [rad/s]

  // Mode transition & gating parameters
  double catch_angle_{0.20};  ///< Angular candidate window for upright catch [rad] (~11.5 deg)
  double fall_angle_{
    0.60};  ///< Angular boundary beyond which balance falls back to swing-up [rad] (~34.4 deg)
  double catch_cmd_fraction_{
    0.60};                       ///< Maximum fraction of motor_accel_max allowable to permit catch
  int catch_ramp_steps_{10};     ///< Control cycles to ramp in LQR command smoothly upon catch
  int catch_ramp_remaining_{0};  ///< Counter of remaining ramp cycles

  // Actuator safety limit
  double motor_accel_max_{150.0};  ///< Hard clamp on commanded motor acceleration [rad/s^2]

  // Operational state
  bool balancing_{false};
};

}  // namespace inverted_pendulum_controller

#endif  // INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_
