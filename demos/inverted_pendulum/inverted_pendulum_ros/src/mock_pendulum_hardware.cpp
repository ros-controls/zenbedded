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

#include "mock_pendulum_hardware/mock_pendulum_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "rclcpp/logging.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kCoulombEps = 1.0e-3;  // rad/s, smoothing width of tanh friction

rclcpp::Logger logger() { return rclcpp::get_logger("MockPendulumHardware"); }
}  // namespace

namespace mock_pendulum_hardware
{

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

double MockPendulumHardware::param(const std::string & name, double fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end())
  {
    return fallback;
  }
  try
  {
    return std::stod(it->second);
  }
  catch (const std::exception &)
  {
    RCLCPP_WARN(
      logger(), "Parameter '%s' is not a number ('%s'); using %g", name.c_str(), it->second.c_str(),
      fallback);
    return fallback;
  }
}

bool MockPendulumHardware::param_bool(const std::string & name, bool fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end())
  {
    return fallback;
  }
  std::string v = it->second;
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return std::tolower(c); });
  return v == "true" || v == "1" || v == "yes" || v == "on";
}

CallbackReturn MockPendulumHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Defaults below are the CAD values from model/inverted_pendulum.urdf:
  //   arm     m=0.026 kg, COM r=0.041 m, Izz,com=1.73e-5  -> J1 = 6.10e-5
  //   pendulum m=0.014 kg, COM d=0.051 m, Ixx,com=8.06e-6 -> J2_pivot = 4.447e-5
  //   arm_to_pendulum origin x=0.062                      -> L1 = 0.062
  // Every parameter is optional: a missing one falls back to the CAD value
  // rather than killing the hardware component.
  J1_nom_ = param("J1", 6.10e-5);
  m2_nom_ = param("m2", 0.014);
  l2_nom_ = param("l2", 0.051);
  L1_ = param("L1", 0.062);
  b1_nom_ = param("b1", 1.0e-4);
  b2_nom_ = param("b2", 8.0e-6);
  c2_nom_ = param("c2", 2.0e-6);
  g_ = param("g", 9.80665);

  // `J2` is the pendulum inertia about the HINGE (the sysid number, 4.45e-5),
  // not about the COM. The parallel-axis split is kept so that randomising
  // m2/l2 moves the hinge inertia consistently.
  const double J2_pivot_nom = param("J2", 4.447e-5);
  J2_com_nom_ = J2_pivot_nom - m2_nom_ * l2_nom_ * l2_nom_;
  if (J2_com_nom_ <= 0.0)
  {
    RCLCPP_WARN(
      logger(),
      "J2 (%g) is smaller than m2*l2^2 (%g): treating J2 as the COM inertia instead of the "
      "hinge inertia.",
      J2_pivot_nom, m2_nom_ * l2_nom_ * l2_nom_);
    J2_com_nom_ = J2_pivot_nom;
  }

  motor_limit_ = std::fabs(param("motor_position_limit", 2.35619449));  // +-135 deg
  motor_max_vel_ = std::fabs(param("motor_max_velocity", 5.0));
  motor_max_accel_ = std::fabs(param("motor_max_acceleration", 200.0));
  motor_max_torque_ = std::fabs(param("motor_max_torque", 0.4));
  model_step_loss_ = param_bool("model_step_loss", true);
  stop_restitution_ = std::clamp(param("hard_stop_restitution", 0.0), 0.0, 0.9);

  motor_steps_per_rev_ = param("motor_steps_per_rev", 6400.0);
  encoder_counts_per_rev_ = param("encoder_counts_per_rev", 4096.0);
  encoder_noise_ = std::fabs(param("encoder_noise", 0.0004));
  encoder_offset_ = param("encoder_offset", 0.0);
  pendulum_vel_from_encoder_ = param_bool("pendulum_velocity_from_encoder", true);
  // Added to the REPORTED pendulum angle only (the plant is untouched). Leave
  // at 0 for the controller convention (0 = upright); set to pi if you are
  // feeding joint_state_broadcaster -> robot_state_publisher and want RViz to
  // agree with the URDF, whose arm_to_pendulum zero is the hanging pose.
  report_offset_ = param("pendulum_report_offset", 0.0);
  vel_filter_alpha_ = std::clamp(param("velocity_filter_alpha", 0.5), 0.01, 1.0);
  motor_vel_noise_ = std::fabs(param("motor_velocity_noise", 0.002));

  pendulum_accel_noise_ = std::fabs(param("pendulum_accel_noise", 0.03));
  motor_accel_noise_ = std::fabs(param("motor_accel_noise", 0.02));
  param_randomization_ = std::fabs(param("param_randomization", 0.02));
  initial_q2_ = param("initial_q2", kPi);
  initial_q2_jitter_ = std::fabs(param("initial_q2_jitter", 0.01));

  sim_speed_ = param("sim_speed_factor", 1.0);
  if (!(sim_speed_ > 0.0))
  {
    RCLCPP_WARN(logger(), "sim_speed_factor must be > 0; forcing 1.0");
    sim_speed_ = 1.0;
  }
  max_substep_ = std::max(param("max_substep", 5.0e-4), 1.0e-6);
  max_substeps_ = static_cast<int>(std::max(param("max_substeps", 4000.0), 1.0));
  max_cycle_dt_ = std::max(param("max_cycle_dt", 0.1), 1.0e-4);

  visualize_ = param_bool("visualize", false);

  const double seed = param("seed", 0.0);
  if (seed > 0.0)
  {
    rng_.seed(static_cast<std::mt19937::result_type>(seed));
  }
  else
  {
    std::random_device rd;
    rng_.seed(rd());
  }

  hw_states_.assign(4, 0.0);
  hw_commands_.assign(1, 0.0);

  RCLCPP_INFO(
    logger(),
    "Furuta mock: L1=%.4f m, l2=%.4f m, m2=%.4f kg, J2_hinge=%.3e, stops=+-%.1f deg, "
    "vmax=%.2f rad/s, sim_speed=%.2fx, viz=%s",
    L1_, l2_nom_, m2_nom_, J2_com_nom_ + m2_nom_ * l2_nom_ * l2_nom_, motor_limit_ * 180.0 / kPi,
    motor_max_vel_, sim_speed_, visualize_ ? "on" : "off");

#ifndef MOCK_PENDULUM_WITH_RAYLIB
  if (visualize_)
  {
    RCLCPP_WARN(
      logger(),
      "visualize:=true but the package was built without raylib. Rebuild with "
      "-DWITH_RAYLIB=ON to get the window.");
    visualize_ = false;
  }
#endif

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface> MockPendulumHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back("motor_joint", "position", &hw_states_[0]);
  interfaces.emplace_back("motor_joint", "velocity", &hw_states_[1]);
  interfaces.emplace_back("pendulum_joint", "position", &hw_states_[2]);
  interfaces.emplace_back("pendulum_joint", "velocity", &hw_states_[3]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MockPendulumHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back("motor_joint", "acceleration", &hw_commands_[0]);
  return interfaces;
}

CallbackReturn MockPendulumHardware::on_activate(const rclcpp_lifecycle::State & /*prev*/)
{
  // Roll a fresh plant. A real rig is never exactly its CAD model: print
  // tolerances, the omitted magnet, bearing preload and how much the operator
  // tightened the grub screw all move these by a percent or two.
  auto jitter = [&](double nominal)
  {
    return param_randomization_ > 0.0 ? nominal * (1.0 + param_randomization_ * normal_(rng_))
                                      : nominal;
  };

  J1_ = jitter(J1_nom_);
  J2_com_ = jitter(J2_com_nom_);
  m2_ = jitter(m2_nom_);
  l2_ = jitter(l2_nom_);
  b1_ = jitter(b1_nom_);
  b2_ = jitter(b2_nom_);
  c2_ = jitter(c2_nom_);
  J2_pivot_ = J2_com_ + m2_ * l2_ * l2_;

  // The encoder's electrical zero never lands exactly on true upright, so the
  // controller's "upright" is a hair off the true inverted equilibrium.
  encoder_offset_ = param("encoder_offset", 0.0) + 0.1 * param_randomization_ * normal_(rng_);

  // The stepper is homed by the operator centring the arm, so its zero is
  // exact by definition; the pendulum starts hanging, give or take a nudge.
  q1_ = 0.0;
  q1_dot_ = 0.0;
  q2_ = wrap_pi(initial_q2_ + initial_q2_jitter_ * normal_(rng_));
  q2_dot_ = initial_q2_jitter_ * 0.5 * normal_(rng_);
  q2_ddot_prev_ = 0.0;
  sim_time_ = 0.0;
  at_limit_ = false;
  step_loss_ = false;

  have_prev_meas_ = false;
  q2_dot_filt_ = 0.0;
  hw_commands_[0] = 0.0;
  publish_states(1.0);

#ifdef MOCK_PENDULUM_WITH_RAYLIB
  if (visualize_)
  {
    PendulumVisualizer::Config cfg;
    cfg.arm_length = L1_;
    cfg.rod_length = param("viz_rod_length", 2.0 * l2_nom_);
    cfg.base_height = param("viz_base_height", 0.070);
    cfg.arm_height = param("viz_arm_height", 0.089);
    cfg.motor_limit = motor_limit_;
    cfg.width = static_cast<int>(param("viz_width", 960.0));
    cfg.height = static_cast<int>(param("viz_height", 720.0));
    viz_ = std::make_unique<PendulumVisualizer>(cfg);
    viz_->start();
  }
#endif

  RCLCPP_INFO(logger(), "Activated: pendulum starts at %.3f rad (0 = upright)", q2_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MockPendulumHardware::on_deactivate(const rclcpp_lifecycle::State & /*prev*/)
{
#ifdef MOCK_PENDULUM_WITH_RAYLIB
  if (viz_)
  {
    viz_->stop();
    viz_.reset();
  }
#endif
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Dynamics
// ---------------------------------------------------------------------------

double MockPendulumHardware::wrap_pi(double a) { return std::remainder(a, kTwoPi); }

// Torque balance about the hinge, with q2 measured from UPRIGHT and positive
// about the arm's own +x (radial) axis, exactly as arm_to_pendulum defines it.
//
//   J2_hinge * q2'' =  m2*g*l2*sin(q2)              gravity (destabilising at 0)
//                    + m2*L1*l2*cos(q2)*q1''        tangential drag of the pivot
//                    + m2*l2^2*sin(q2)*cos(q2)*q1'^2 centrifugal stiffening
//                    - b2*q2' - c2*tanh(q2'/eps)    bearing friction
//
// Sign check: at q2 = 0 a positive arm acceleration accelerates the pivot along
// the arm's +y while the bob's inertia holds it back, so q2 grows — i.e. you
// catch a pendulum leaning at q2 > 0 with q1'' < 0, moving the base under the
// fall. That is the standard Furuta convention.
double MockPendulumHardware::pendulum_accel(
  double q2, double q2_dot, double q1_dot, double q1_ddot) const
{
  const double s = std::sin(q2);
  const double c = std::cos(q2);

  const double tau_gravity = m2_ * g_ * l2_ * s;
  const double tau_coupling = m2_ * L1_ * l2_ * c * q1_ddot;
  const double tau_centrifugal = m2_ * l2_ * l2_ * s * c * q1_dot * q1_dot;
  const double tau_viscous = -b2_ * q2_dot;
  const double tau_coulomb = -c2_ * std::tanh(q2_dot / kCoulombEps);

  return (tau_gravity + tau_coupling + tau_centrifugal + tau_viscous + tau_coulomb) / J2_pivot_;
}

void MockPendulumHardware::integrate(double h)
{
  // ---- motor: an open-loop stepper tracking an acceleration command --------
  double a_cmd = hw_commands_[0];
  if (!std::isfinite(a_cmd))
  {
    a_cmd = 0.0;
  }
  a_cmd = std::clamp(a_cmd, -motor_max_accel_, motor_max_accel_);

  // Micro-stepping timing jitter: the commanded profile is never realised to
  // the last step.
  a_cmd += motor_accel_noise_ * normal_(rng_);

  // Torque feasibility. Reflected inertia at the motor axis includes the
  // pendulum swinging out of plane, plus the reaction of its own acceleration.
  step_loss_ = false;
  if (model_step_loss_ && motor_max_torque_ > 0.0)
  {
    const double s2 = std::sin(q2_);
    const double J0 = J1_ + m2_ * L1_ * L1_ + m2_ * l2_ * l2_ * s2 * s2;
    const double tau_react = m2_ * L1_ * l2_ * std::cos(q2_) * q2_ddot_prev_ + b1_ * q1_dot_;
    const double tau_req = J0 * a_cmd + tau_react;
    if (std::fabs(tau_req) > motor_max_torque_)
    {
      const double tau_avail = std::copysign(motor_max_torque_, tau_req);
      a_cmd = (tau_avail - tau_react) / J0;
      step_loss_ = true;
    }
  }

  const double v_old = q1_dot_;
  double v_new = v_old + a_cmd * h;

  // Step-rate ceiling: past 5 rad/s the firmware simply stops accelerating.
  v_new = std::clamp(v_new, -motor_max_vel_, motor_max_vel_);

  double p_new = q1_ + v_new * h;

  // Mechanical hard stops (the lid bosses). A stepper that runs into one
  // stalls: it loses the steps and stops dead rather than bouncing.
  at_limit_ = false;
  if (p_new > motor_limit_)
  {
    p_new = motor_limit_;
    v_new = -stop_restitution_ * v_new;
    at_limit_ = true;
  }
  else if (p_new < -motor_limit_)
  {
    p_new = -motor_limit_;
    v_new = -stop_restitution_ * v_new;
    at_limit_ = true;
  }

  // What the arm ACTUALLY did is what the pendulum feels — this is what makes
  // the stops and the speed cap visible in the pendulum's motion instead of
  // being a silent saturation.
  const double a_real = (v_new - v_old) / h;

  // ---- pendulum ----------------------------------------------------------
  double a2 = pendulum_accel(q2_, q2_dot_, v_old, a_real);
  a2 += pendulum_accel_noise_ * normal_(rng_);

  // Semi-implicit (symplectic) Euler: stable on oscillators at this step size.
  q2_dot_ += a2 * h;
  q2_ += q2_dot_ * h;
  q2_ = wrap_pi(q2_);
  q2_ddot_prev_ = a2;

  q1_dot_ = v_new;
  q1_ = p_new;
  sim_time_ += h;
}

// ---------------------------------------------------------------------------
// Sensor model
// ---------------------------------------------------------------------------

void MockPendulumHardware::publish_states(double dt_sim)
{
  // Stepper position is an integer step count, by construction.
  const double step = kTwoPi / motor_steps_per_rev_;
  const double q1_meas = std::round(q1_ / step) * step;

  // Magnetic encoder: additive noise, then quantisation to counts.
  const double count = kTwoPi / encoder_counts_per_rev_;
  double q2_meas = q2_ + encoder_offset_ + encoder_noise_ * normal_(rng_);
  q2_meas = wrap_pi(std::round(q2_meas / count) * count);

  double q2_dot_meas;
  if (pendulum_vel_from_encoder_ && have_prev_meas_ && dt_sim > 0.0)
  {
    // Finite difference of the quantised angle, wrapped so the -pi/+pi seam is
    // not a 2*pi velocity spike, then low-passed. This reproduces the real
    // quantisation-driven velocity noise a controller has to live with.
    const double raw = wrap_pi(q2_meas - q2_meas_prev_) / dt_sim;
    q2_dot_filt_ += vel_filter_alpha_ * (raw - q2_dot_filt_);
    q2_dot_meas = q2_dot_filt_;
  }
  else
  {
    q2_dot_meas = q2_dot_;
    q2_dot_filt_ = q2_dot_;
  }
  q2_meas_prev_ = q2_meas;
  have_prev_meas_ = true;

  hw_states_[0] = q1_meas;
  hw_states_[1] = q1_dot_ + motor_vel_noise_ * normal_(rng_);
  hw_states_[2] = report_offset_ == 0.0 ? q2_meas : wrap_pi(q2_meas + report_offset_);
  hw_states_[3] = q2_dot_meas;
}

// ---------------------------------------------------------------------------
// read / write
// ---------------------------------------------------------------------------

hardware_interface::return_type MockPendulumHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt_wall = period.seconds();
  if (!(dt_wall > 0.0))
  {
    return hardware_interface::return_type::OK;
  }
  // Survive a debugger pause or a scheduling hiccup without exploding.
  dt_wall = std::min(dt_wall, max_cycle_dt_);

  // Sim speed factor: how much SIM time one WALL second buys. The dynamics,
  // the sub-step size and therefore the trajectory are untouched; only the
  // sim-time-per-control-cycle changes. Note the corollary: at 5x, a 200 Hz
  // controller is effectively a 40 Hz controller as far as the plant is
  // concerned.
  const double dt_sim = dt_wall * sim_speed_;

  int n = static_cast<int>(std::ceil(dt_sim / max_substep_));
  n = std::clamp(n, 1, max_substeps_);
  const double h = dt_sim / static_cast<double>(n);

  for (int i = 0; i < n; ++i)
  {
    integrate(h);
  }

  publish_states(dt_sim);

  if (at_limit_)
  {
    RCLCPP_WARN_THROTTLE(
      logger(), clock_, 1000, "Arm is against the %+.0f deg hard stop",
      q1_ > 0.0 ? motor_limit_ * 180.0 / kPi : -motor_limit_ * 180.0 / kPi);
  }
  if (step_loss_)
  {
    RCLCPP_WARN_THROTTLE(
      logger(), clock_, 1000,
      "Commanded acceleration exceeds the %.2f N m holding torque: losing steps",
      motor_max_torque_);
  }

#ifdef MOCK_PENDULUM_WITH_RAYLIB
  if (viz_)
  {
    viz_->update(q1_, q2_, sim_time_, at_limit_ || step_loss_, sim_speed_);
  }
#endif

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockPendulumHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw_commands_[0] is the motor acceleration written by the controller during
  // update(); it is consumed by the integrator in read() on the next cycle,
  // which is exactly the one-cycle actuation delay the real rig has.
  return hardware_interface::return_type::OK;
}

}  // namespace mock_pendulum_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mock_pendulum_hardware::MockPendulumHardware, hardware_interface::SystemInterface)
