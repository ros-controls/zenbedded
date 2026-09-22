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

// ============================================================================
//
// Control architecture
// ============================================================================
//
// Two regimes, selected by how far the pendulum is from upright:
//
//   SWING-UP  (|q2| > catch_angle_)
//     Energy-shaping pump: drive the pendulum's mechanical energy toward the
//     upright potential energy.
//
//     dE/dt  =  q2_dot * m2 * L1 * l2 * cos(q2) * q1_ddot
//
//     The SIGN of  -(E - E_target) * q2_dot * cos(q2)  gives the direction
//     that adds energy. The magnitude is a fixed saturation:
//
//       u_pump = swing_accel_max * sign( -(E - E_target) * q2_dot * cos(q2) )
//
//     Bang-bang, not proportional: a proportional pump vanishes at rest and
//     cannot break the hinge's Coulomb friction, so the pendulum never starts.
//     The arm is simultaneously re-centred with a PD term so the pump never
//     exhausts the available travel:
//
//       u_arm  = -k_arm * q1 - k_arm_vel * q1_dot
//
//     The pump term is zeroed when |q1| > arm_zone_  so the arm-centering
//     has unconditional authority near the stops.
//
//   BALANCE  (|q2| ≤ catch_angle_)
//     Full-state LQR feedback computed from the linearisation about q2 = 0:
//
//       state x = [q1, q1_dot, q2, q2_dot]
//       u = -K * x
//
//     LQR derivation (plant constants from model/inverted_pendulum.urdf):
//       m2=0.014 kg, l2=0.051 m, L1=0.062 m, J2_hinge=4.447e-5 kg·m²
//       Q = diag(5, 0.5, 80, 3),  R = 0.2
//       K = [-5.000, -4.352, +418.238, +33.281]
//
//     Eigenvalues of (A - BK):
//       -14.492, -10.947, -1.759 ± 1.367i  (all stable)
//
//     Note the negative sign on k_q1: the arm has to move away from centre
//     before it can come back, and negating this gain is the most common
//     reason a hand-tuned balancer walks into a hard stop.
//
//   TRANSITION
//     Hysteresis: enter balance at catch_angle_, exit (back to swing-up) at
//     fall_angle_ > catch_angle_.  This prevents chattering near the boundary.
//
// All gains are ROS parameters so they can be tuned without recompiling.
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

    // Plant constants (must match the hardware / URDF values)
    auto_declare<double>("m2", 0.014);     // pendulum mass            [kg]
    auto_declare<double>("l2", 0.051);     // hinge to COM             [m]
    auto_declare<double>("L1", 0.062);     // arm reach                [m]
    auto_declare<double>("J2", 4.447e-5);  // pendulum hinge inertia   [kg m²]
    auto_declare<double>("g", 9.80665);

    // LQR gains for x = [q1, q1d, q2, q2d]
    // Defaults: Q=diag(5,0.5,80,3), R=0.2 on the URDF plant.
    auto_declare<double>("k_q1", -5.000);
    auto_declare<double>("k_q1_vel", -4.352);
    auto_declare<double>("k_q2", 418.238);
    auto_declare<double>("k_q2_vel", 33.281);

    // Swing-up
    auto_declare<double>("k_e", 80.0);       // (direction only; magnitude is swing_accel_max)
    auto_declare<double>("k_arm", 30.0);     // arm-recentering P gain
    auto_declare<double>("k_arm_vel", 6.0);  // arm-recentering D gain
    auto_declare<double>("swing_accel_max", 60.0);  // pump saturation       [rad/s²]
    auto_declare<double>("arm_zone", 2.0);          // pump disabled beyond  [rad]
    auto_declare<double>("swing_start_vel", 0.01);  // dead-start threshold  [rad/s]

    // Mode transitions
    auto_declare<double>("catch_angle", 0.35);         // candidate window       [rad]
    auto_declare<double>("fall_angle", 0.70);          // exit LQR               [rad]
    auto_declare<double>("catch_cmd_fraction", 0.55);  // max |u_lqr|/accel_max to allow catch
    auto_declare<double>("catch_ramp_steps", 10.0);    // cycles to blend in LQR command

    // Safety
    auto_declare<double>("motor_accel_max", 150.0);  // hard clamp on output  [rad/s²]
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------

controller_interface::CallbackReturn InvertedPendulumController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto & n = *get_node();

  motor_joint_state_name_ = n.get_parameter("motor_joint_state_name").as_string();
  motor_joint_vel_name_ = n.get_parameter("motor_joint_vel_name").as_string();
  pendulum_joint_state_name_ = n.get_parameter("pendulum_joint_state_name").as_string();
  pendulum_joint_vel_name_ = n.get_parameter("pendulum_joint_vel_name").as_string();
  motor_joint_command_name_ = n.get_parameter("motor_joint_command_name").as_string();

  m2_ = n.get_parameter("m2").as_double();
  l2_ = n.get_parameter("l2").as_double();
  L1_ = n.get_parameter("L1").as_double();
  J2_ = n.get_parameter("J2").as_double();
  g_ = n.get_parameter("g").as_double();

  k_q1_ = n.get_parameter("k_q1").as_double();
  k_q1_vel_ = n.get_parameter("k_q1_vel").as_double();
  k_q2_ = n.get_parameter("k_q2").as_double();
  k_q2_vel_ = n.get_parameter("k_q2_vel").as_double();

  k_e_ = n.get_parameter("k_e").as_double();
  k_arm_ = n.get_parameter("k_arm").as_double();
  k_arm_vel_ = n.get_parameter("k_arm_vel").as_double();
  swing_accel_max_ = std::fabs(n.get_parameter("swing_accel_max").as_double());
  arm_zone_ = std::fabs(n.get_parameter("arm_zone").as_double());

  catch_angle_ = std::fabs(n.get_parameter("catch_angle").as_double());
  fall_angle_ = std::fabs(n.get_parameter("fall_angle").as_double());
  catch_cmd_fraction_ = std::clamp(n.get_parameter("catch_cmd_fraction").as_double(), 0.05, 1.0);
  catch_ramp_steps_ =
    std::max(1, static_cast<int>(n.get_parameter("catch_ramp_steps").as_double()));
  if (fall_angle_ <= catch_angle_)
  {
    RCLCPP_WARN(
      n.get_logger(),
      "fall_angle (%.3f) <= catch_angle (%.3f); setting fall_angle = 2 * catch_angle", fall_angle_,
      catch_angle_);
    fall_angle_ = 2.0 * catch_angle_;
  }

  motor_accel_max_ = std::fabs(n.get_parameter("motor_accel_max").as_double());

  E_target_ = m2_ * g_ * l2_;  // PE at upright; KE=0 -> E_target = m2*g*l2

  const double edge_cmd = std::fabs(k_q2_) * catch_angle_;
  if (edge_cmd > catch_cmd_fraction_ * motor_accel_max_)
  {
    RCLCPP_WARN(
      n.get_logger(),
      "catch_angle (%.3f rad) alone demands %.1f rad/s^2 from k_q2 -- already %.0f%% of "
      "motor_accel_max with zero velocity margin. The command gate (catch_cmd_fraction=%.2f) "
      "will reject most catches at this window edge; consider a smaller catch_angle.",
      catch_angle_, edge_cmd, 100.0 * edge_cmd / motor_accel_max_, catch_cmd_fraction_);
  }

  RCLCPP_INFO(
    n.get_logger(),
    "Furuta LQR controller configured:\n"
    "  Plant:    m2=%.4f kg  l2=%.4f m  L1=%.4f m  J2=%.3e\n"
    "  LQR K:   [%.3f, %.3f, %.3f, %.3f]\n"
    "  Swing-up: sat=%.1f rad/s²  arm_zone=%.2f rad\n"
    "  Catch:    |q2| < %.3f rad (%.1f°), command < %.0f%% of accel_max, fall back at %.3f rad "
    "(%.1f°)\n"
    "  Ramp:     %d cycles to blend in LQR command after catch",
    m2_, l2_, L1_, J2_, k_q1_, k_q1_vel_, k_q2_, k_q2_vel_, swing_accel_max_, arm_zone_,
    catch_angle_, catch_angle_ * 180.0 / M_PI, 100.0 * catch_cmd_fraction_, fall_angle_,
    fall_angle_ * 180.0 / M_PI, catch_ramp_steps_);

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
  command_interfaces_[0].set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------

controller_interface::return_type InvertedPendulumController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // State interfaces are guaranteed to be in the order declared by
  // state_interface_configuration(): q1, q1d, q2, q2d.
  const double q1 = state_interfaces_[0].get_optional().value_or(0.0);
  const double q1d = state_interfaces_[1].get_optional().value_or(0.0);
  const double q2 = state_interfaces_[2].get_optional().value_or(0.0);
  const double q2d = state_interfaces_[3].get_optional().value_or(0.0);

  // ---- Mode transitions ---------------------------------------------------
  //
  // Catching on POSITION ALONE is not safe. At the edge of a 0.35 rad catch
  // window, k_q2 * catch_angle alone already demands ~146 rad/s^2 out of a
  // 150 rad/s^2 ceiling -- 98% of authority spent before velocity damping or
  // arm recentering get a say. Any residual swing velocity, sensor noise, or
  // actuation delay (a real serial round-trip that this control law has no
  // way to see) pushes the command over the clamp. A clipped LQR command
  // cannot arrest the swing, so it "catches", instantly saturates, and gets
  // thrown back out -- which is exactly a catch/fall/re-swing oscillation
  // that never appears in a low-noise, zero-delay simulation.
  //
  // Fix: gate the catch on the command the LQR would ACTUALLY issue right
  // now, not on position alone. This is a joint condition on position and
  // velocity together (whichever combination is currently dangerous), and it
  // adapts automatically to noise levels a fixed velocity threshold would
  // have to be re-tuned for by hand.
  const double abs_q2 = std::fabs(q2);
  const double u_lqr_candidate = -(k_q1_ * q1 + k_q1_vel_ * q1d + k_q2_ * q2 + k_q2_vel_ * q2d);
  const bool command_is_safe = std::fabs(u_lqr_candidate) < catch_cmd_fraction_ * motor_accel_max_;

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
    RCLCPP_WARN(get_node()->get_logger(), "FELL: returning to swing-up at q2=%.4f rad", q2);
  }

  // ---- Control law -------------------------------------------------------
  double u = 0.0;

  if (balancing_)
  {
    // Full-state LQR: u = -K * [q1, q1d, q2, q2d]
    u = -(k_q1_ * q1 + k_q1_vel_ * q1d + k_q2_ * q2 + k_q2_vel_ * q2d);

    // Soft-engage: blend in linearly over catch_ramp_steps_ cycles instead of
    // slamming from the swing-up command to the full LQR command in one
    // cycle. A real stepper does not track a step change in acceleration
    // instantly; ramping keeps the actuator inside its achievable response
    // during exactly the moment precision matters most.
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
    // Energy of the pendulum about its hinge
    // E = 0.5 * J2 * q2d² + m2*g*l2*cos(q2)
    // E_target = m2*g*l2  (upright, zero KE)
    // dE/dt = q2d * m2*L1*l2*cos(q2) * u   (dominant coupling term)
    // To drive E → E_target: u = -k_e * (E - E_target) * q2d * cos(q2)
    const double E = 0.5 * J2_ * q2d * q2d + m2_ * g_ * l2_ * std::cos(q2);
    const double E_err = E - E_target_;

    // Pump DIRECTION from energy shaping; pump MAGNITUDE is a fixed
    // saturation, not proportional to the energy error.
    //
    // This is a bang-bang law and it has to be. A proportional pump
    //   u = -k_e * E_err * q2_dot * cos(q2)
    // collapses to ~0 when the pendulum is at rest (q2_dot ~ 0), producing a
    // hinge torque roughly 8x SMALLER than the bearing's Coulomb friction.
    // The pendulum then never breaks stiction and simply hangs there. The
    // saturated form delivers ~1300x the friction torque and always starts.
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
        // Dead start: q2_dot is ~0, so pump_dir carries no usable sign.
        // Kick in a fixed direction to seed the first half-swing.
        u_pump = swing_accel_max_;
      }
    }

    // Arm-centering PD: always active
    const double u_arm = -k_arm_ * q1 - k_arm_vel_ * q1d;

    u = u_pump + u_arm;
  }

  // Global safety clamp (hardware also enforces its own limit)
  u = std::clamp(u, -motor_accel_max_, motor_accel_max_);

  if (!command_interfaces_[0].set_value(u))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to set command interface value");
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 250,
    "[%s] q1=%+.3f q1d=%+.4f | q2=%+.4f q2d=%+.4f | u=%+8.3f", balancing_ ? "BALANCE" : "SWING-UP",
    q1, q1d, q2, q2d, u);

  return controller_interface::return_type::OK;
}

}  // namespace inverted_pendulum_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  inverted_pendulum_controller::InvertedPendulumController,
  controller_interface::ControllerInterface)
