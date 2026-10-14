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

#include "inverted_pendulum_controller/inverted_pendulum_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

// ============================================================================
// Control Architecture
// ============================================================================
//
// The Furuta (rotary inverted) pendulum controller operates in two regimes:
//
// 1. SWING-UP (Non-linear Energy Shaping):
//    Drives the pendulum's total mechanical energy toward the upright potential energy
//    E_target = m2 * g * l2.
//
//    Total energy about the hinge:
//      E = 0.5 * J2 * q2_dot^2 + m2 * g * l2 * cos(q2)
//
//    Time derivative of energy:
//      dE/dt = q2_dot * m2 * L1 * l2 * cos(q2) * u_pump
//
//    To pump energy toward E_target:
//      pump_dir = -(E - E_target) * q2_dot * cos(q2)
//      u_pump   = swing_accel_max * sign(pump_dir)
//
//    Bang-bang pumping is mathematically required: a proportional energy pump
//    vanishes at rest (q2_dot ~ 0), failing to overcome bearing Coulomb friction.
//    A dead-start kick seeds the initial half-swing.
//
//    Simultaneously, bounded arm-centering PD regulates motor travel:
//      u_arm = -k_arm * q1 - k_arm_vel * q1_dot
//    When |q1| >= arm_zone, u_pump is zeroed so arm-centering has unconditional authority.
//
// 2. BALANCING (Full-State Linear Quadratic Regulator):
//    Active when |q2| <= catch_angle and the required command does not saturate.
//      x = [q1, q1_dot, q2, q2_dot]^T
//      u = -K * x = -(k_q1 * q1 + k_q1_vel * q1_dot + k_q2 * q2 + k_q2_vel * q2_dot)
//
//    Catch Gating & Soft Ramping:
//    - Command gate: catches are rejected if |u_lqr_candidate| exceeds
//      catch_cmd_fraction * motor_accel_max, preventing overshoot and chattering.
//    - Linear ramp: smoothly blends the LQR command over catch_ramp_steps cycles.
//    - Hysteresis: balance exits back to swing-up if |q2| > fall_angle (fall_angle > catch_angle).
//
// ============================================================================

namespace inverted_pendulum_controller
{

InvertedPendulumController::InvertedPendulumController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn InvertedPendulumController::on_init()
{
  try
  {
    // Interface names
    auto_declare<std::string>("motor_joint_state_name", "motor_joint/position");
    auto_declare<std::string>("motor_joint_vel_name", "motor_joint/velocity");
    auto_declare<std::string>("pendulum_joint_state_name", "pendulum_joint/position");
    auto_declare<std::string>("pendulum_joint_vel_name", "pendulum_joint/velocity");
    auto_declare<std::string>("motor_joint_command_name", "motor_joint/acceleration");

    // Physical plant constants (must match hardware / URDF)
    auto_declare<double>("m2", 0.014);     // Pendulum mass [kg]
    auto_declare<double>("l2", 0.051);     // Pendulum hinge to COM [m]
    auto_declare<double>("L1", 0.062);     // Motor arm radius to hinge [m]
    auto_declare<double>("J2", 4.447e-5);  // Pendulum inertia about hinge [kg*m^2]
    auto_declare<double>("g", 9.80665);    // Gravity [m/s^2]

    // Full-state LQR gains (u = -K * [q1, q1d, q2, q2d])
    auto_declare<double>("k_q1", -5.000);
    auto_declare<double>("k_q1_vel", -4.352);
    auto_declare<double>("k_q2", 418.238);
    auto_declare<double>("k_q2_vel", 33.281);

    // Energy-shaping swing-up parameters
    auto_declare<double>("k_e", 80.0);
    auto_declare<double>("k_arm", 31.0);
    auto_declare<double>("k_arm_vel", 6.0);
    auto_declare<double>("swing_accel_max", 100.0);
    auto_declare<double>("arm_zone", 2.0);
    auto_declare<double>("swing_start_vel", 0.01);

    // Mode transition parameters
    auto_declare<double>("catch_angle", 0.20);
    auto_declare<double>("fall_angle", 0.60);
    auto_declare<double>("catch_cmd_fraction", 0.60);
    auto_declare<int64_t>("catch_ramp_steps", 10);

    // Actuator safety limits
    auto_declare<double>("motor_accel_max", 150.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
InvertedPendulumController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back(motor_joint_command_name_);
  return cfg;
}

controller_interface::InterfaceConfiguration
InvertedPendulumController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back(motor_joint_state_name_);
  cfg.names.push_back(motor_joint_vel_name_);
  cfg.names.push_back(pendulum_joint_state_name_);
  cfg.names.push_back(pendulum_joint_vel_name_);
  return cfg;
}

controller_interface::CallbackReturn InvertedPendulumController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto & n = *get_node();

  motor_joint_state_name_ = n.get_parameter("motor_joint_state_name").as_string();
  motor_joint_vel_name_ = n.get_parameter("motor_joint_vel_name").as_string();
  pendulum_joint_state_name_ = n.get_parameter("pendulum_joint_state_name").as_string();
  pendulum_joint_vel_name_ = n.get_parameter("pendulum_joint_vel_name").as_string();
  motor_joint_command_name_ = n.get_parameter("motor_joint_command_name").as_string();

  m2_ = std::fabs(n.get_parameter("m2").as_double());
  l2_ = std::fabs(n.get_parameter("l2").as_double());
  L1_ = std::fabs(n.get_parameter("L1").as_double());
  J2_ = std::fabs(n.get_parameter("J2").as_double());
  g_ = std::fabs(n.get_parameter("g").as_double());

  k_q1_ = n.get_parameter("k_q1").as_double();
  k_q1_vel_ = n.get_parameter("k_q1_vel").as_double();
  k_q2_ = n.get_parameter("k_q2").as_double();
  k_q2_vel_ = n.get_parameter("k_q2_vel").as_double();

  k_e_ = n.get_parameter("k_e").as_double();
  k_arm_ = std::fabs(n.get_parameter("k_arm").as_double());
  k_arm_vel_ = std::fabs(n.get_parameter("k_arm_vel").as_double());
  swing_accel_max_ = std::fabs(n.get_parameter("swing_accel_max").as_double());
  arm_zone_ = std::fabs(n.get_parameter("arm_zone").as_double());
  swing_start_vel_ = std::fabs(n.get_parameter("swing_start_vel").as_double());

  catch_angle_ = std::fabs(n.get_parameter("catch_angle").as_double());
  fall_angle_ = std::fabs(n.get_parameter("fall_angle").as_double());
  catch_cmd_fraction_ = std::clamp(n.get_parameter("catch_cmd_fraction").as_double(), 0.05, 1.0);

  // Robustly handle either integer or double declaration for ramp steps
  const auto & ramp_param = n.get_parameter("catch_ramp_steps");
  if (ramp_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    catch_ramp_steps_ = std::max(1, static_cast<int>(ramp_param.as_int()));
  }
  else
  {
    catch_ramp_steps_ = std::max(1, static_cast<int>(ramp_param.as_double()));
  }

  motor_accel_max_ = std::fabs(n.get_parameter("motor_accel_max").as_double());

  if (fall_angle_ <= catch_angle_)
  {
    RCLCPP_WARN(
      n.get_logger(),
      "fall_angle (%.3f rad) <= catch_angle (%.3f rad); automatically adjusting fall_angle = 2 * "
      "catch_angle (%.3f rad)",
      fall_angle_, catch_angle_, 2.0 * catch_angle_);
    fall_angle_ = 2.0 * catch_angle_;
  }

  // Upright potential energy target
  E_target_ = m2_ * g_ * l2_;

  // Pre-flight check on catch authority
  const double edge_cmd = std::fabs(k_q2_) * catch_angle_;
  if (edge_cmd > catch_cmd_fraction_ * motor_accel_max_)
  {
    RCLCPP_WARN(
      n.get_logger(),
      "catch_angle (%.3f rad) demands %.1f rad/s^2 from k_q2 alone (%.0f%% of motor_accel_max). "
      "Command gate (catch_cmd_fraction=%.2f) may reject catches at window boundary; consider "
      "lowering catch_angle.",
      catch_angle_, edge_cmd, 100.0 * edge_cmd / motor_accel_max_, catch_cmd_fraction_);
  }

  RCLCPP_INFO(
    n.get_logger(),
    "Furuta LQR & Energy-shaping controller configured:\n"
    "  Plant:     m2=%.4f kg  l2=%.4f m  L1=%.4f m  J2=%.3e kg*m^2  g=%.4f m/s^2\n"
    "  LQR K:     [k_q1=%.3f, k_q1_vel=%.3f, k_q2=%.3f, k_q2_vel=%.3f]\n"
    "  Swing-up:  pump_sat=%.1f rad/s^2  k_arm=%.1f  k_arm_vel=%.1f  arm_zone=%.2f rad\n"
    "  Catch:     |q2| < %.3f rad (%.1f deg), cmd < %.0f%% of accel_max, fall_angle=%.3f rad (%.1f "
    "deg)\n"
    "  Ramp:      %d cycles to blend LQR command after catch",
    m2_, l2_, L1_, J2_, g_, k_q1_, k_q1_vel_, k_q2_, k_q2_vel_, swing_accel_max_, k_arm_,
    k_arm_vel_, arm_zone_, catch_angle_, catch_angle_ * 180.0 / M_PI, 100.0 * catch_cmd_fraction_,
    fall_angle_, fall_angle_ * 180.0 / M_PI, catch_ramp_steps_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InvertedPendulumController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  balancing_ = false;
  catch_ramp_remaining_ = 0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InvertedPendulumController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!command_interfaces_.empty())
  {
    if (!command_interfaces_[0].set_value(0.0))
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to zero command interface on deactivation");
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type InvertedPendulumController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // State interfaces ordered: [0] q1, [1] q1d, [2] q2, [3] q2d
  const double q1 = state_interfaces_[0].get_optional().value_or(0.0);
  const double q1d = state_interfaces_[1].get_optional().value_or(0.0);
  const double q2_raw = state_interfaces_[2].get_optional().value_or(0.0);
  const double q2d = state_interfaces_[3].get_optional().value_or(0.0);

  // Normalize pendulum angle into [-pi, pi] so upright is always at 0 rad regardless
  // of continuous rotation, multiple revolutions, or encoder wrapping offsets.
  const double q2 = std::remainder(q2_raw, 2.0 * M_PI);
  const double abs_q2 = std::fabs(q2);

  // Evaluate candidate LQR command to check actuator feasibility
  const double u_lqr_candidate = -(k_q1_ * q1 + k_q1_vel_ * q1d + k_q2_ * q2 + k_q2_vel_ * q2d);
  const bool command_is_safe = std::fabs(u_lqr_candidate) < catch_cmd_fraction_ * motor_accel_max_;

  // ---- State Machine: Mode Transitions ------------------------------------
  if (!balancing_ && abs_q2 < catch_angle_ && command_is_safe)
  {
    balancing_ = true;
    catch_ramp_remaining_ = catch_ramp_steps_;
    RCLCPP_INFO(
      get_node()->get_logger(), "CATCH: entering LQR balance at q2=%.4f rad, q2d=%.4f rad/s", q2,
      q2d);
  }
  else if (balancing_ && abs_q2 > fall_angle_)
  {
    balancing_ = false;
    RCLCPP_WARN(
      get_node()->get_logger(),
      "FALL: pendulum angle |q2|=%.4f rad exceeded fall boundary; resuming swing-up", abs_q2);
  }

  // ---- Compute Control Command --------------------------------------------
  double u = 0.0;

  if (balancing_)
  {
    // Full-state LQR control: u = -K * x
    u = u_lqr_candidate;

    // Smooth ramp engagement to prevent actuator acceleration step discontinuities
    if (catch_ramp_remaining_ > 0)
    {
      const double frac =
        1.0 - static_cast<double>(catch_ramp_remaining_) / static_cast<double>(catch_ramp_steps_);
      u *= frac;
      --catch_ramp_remaining_;
    }
  }
  else
  {
    // Non-linear energy shaping swing-up:
    // Mechanical energy of the pendulum about its hinge: E = 0.5 * J2 * q2d^2 + m2 * g * l2 *
    // cos(q2)
    const double E = 0.5 * J2_ * q2d * q2d + m2_ * g_ * l2_ * std::cos(q2);
    const double E_err = E - E_target_;

    // Pumping direction: sign of power transfer term dE/dt = q2d * m2 * L1 * l2 * cos(q2) * u
    const double pump_dir = -E_err * q2d * std::cos(q2);

    double u_pump = 0.0;
    if (std::fabs(q1) < arm_zone_)
    {
      if (std::fabs(q2d) > swing_start_vel_)
      {
        u_pump = std::copysign(swing_accel_max_, pump_dir);
      }
      else
      {
        // Dead start: seed initial half-swing with fixed directional kick
        u_pump = swing_accel_max_;
      }
    }

    // Arm-centering PD (always active, has full authority beyond arm_zone)
    const double u_arm = -k_arm_ * q1 - k_arm_vel_ * q1d;

    u = u_pump + u_arm;
  }

  // Actuator hardware saturation clamp
  u = std::clamp(u, -motor_accel_max_, motor_accel_max_);

  if (!command_interfaces_[0].set_value(u))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to set acceleration command interface value!");
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 500,
    "[%s] q1=%+.3f q1d=%+.4f | q2=%+.4f q2d=%+.4f | u=%+8.3f", balancing_ ? "BALANCE" : "SWING-UP",
    q1, q1d, q2, q2d, u);

  return controller_interface::return_type::OK;
}

}  // namespace inverted_pendulum_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  inverted_pendulum_controller::InvertedPendulumController,
  controller_interface::ControllerInterface)
