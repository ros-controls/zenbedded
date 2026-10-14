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

#ifndef MOCK_PENDULUM_HARDWARE__PENDULUM_VISUALIZER_HPP_
#define MOCK_PENDULUM_HARDWARE__PENDULUM_VISUALIZER_HPP_

#include <atomic>
#include <thread>

namespace mock_pendulum_hardware
{

/// Optional raylib window showing the rig as high-fidelity 3D primitives
/// (compact enclosure with ventilation grill, rounded blue arm, red lollipop
/// pendulum, and cover travel limit markings).
///
/// The window lives on its own thread and communicates via lock-free atomics
/// with read(), ensuring rendering never blocks the real-time control cycle.
class PendulumVisualizer
{
public:
  struct Config
  {
    double arm_length = 0.065;        ///< motor axis -> hinge [m]
    double rod_length = 0.052;        ///< hinge -> bob centre [m] (clears box bottom)
    double base_height = 0.058;       ///< enclosure height [m]
    double arm_height = 0.072;        ///< hinge height above table [m]
    double motor_limit = 2.35619449;  ///< +-135 deg [rad]
    int width = 960;
    int height = 720;
  };

  explicit PendulumVisualizer(const Config & cfg);
  ~PendulumVisualizer();

  PendulumVisualizer(const PendulumVisualizer &) = delete;
  PendulumVisualizer & operator=(const PendulumVisualizer &) = delete;

  void start();
  void stop();

  /// Called from read(). Non-blocking, wait-free.
  void update(double q1, double q2, double sim_time, bool fault, double sim_speed);

  /// Called from read() to check if the user clicked the Distort button.
  bool consume_disturbance();

private:
  void run();

  Config cfg_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> q1_{0.0};
  std::atomic<double> q2_{0.1};
  std::atomic<double> sim_time_{0.0};
  std::atomic<double> sim_speed_{1.0};
  std::atomic<bool> fault_{false};
  std::atomic<bool> disturb_trigger_{false};
};

}  // namespace mock_pendulum_hardware

#endif  // MOCK_PENDULUM_HARDWARE__PENDULUM_VISUALIZER_HPP_
