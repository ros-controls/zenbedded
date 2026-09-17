// Copyright 2026 Open Source Robotics Foundation, Inc.
// Licensed under the Apache License, Version 2.0 (see LICENSE).

#ifndef INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_
#define INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace inverted_pendulum_controller
{

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

private:
  // Interface name strings
  std::string motor_joint_state_name_;
  std::string motor_joint_vel_name_;
  std::string pendulum_joint_state_name_;
  std::string pendulum_joint_vel_name_;
  std::string motor_joint_command_name_;

  // Plant constants
  double m2_{0.014}, l2_{0.051}, L1_{0.062}, J2_{4.447e-5}, g_{9.80665};
  double E_target_{0.0};  // computed from m2, g, l2 in on_configure

  // LQR gains  (u = -K * [q1, q1d, q2, q2d])
  double k_q1_{-5.000};
  double k_q1_vel_{-4.352};
  double k_q2_{418.238};
  double k_q2_vel_{33.281};

  // Swing-up
  double k_e_{80.0};
  double k_arm_{30.0};
  double k_arm_vel_{6.0};
  double swing_accel_max_{60.0};
  double arm_zone_{2.0};
  double swing_start_vel_{0.01};

  // Mode switching
  double catch_angle_{0.35};
  double fall_angle_{0.70};
  double motor_accel_max_{150.0};

  // Runtime state
  bool balancing_{false};
};

}  // namespace inverted_pendulum_controller

#endif  // INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_
