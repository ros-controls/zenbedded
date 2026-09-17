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

/// Optional raylib window showing the rig as primitives (box base, cylinder
/// arm, rod + bob pendulum).
///
/// The window lives on its own thread and only ever reads lock-free atomics
/// written by read(), so a slow or stalled renderer can never stretch a
/// control cycle. The real-time loop is never blocked by drawing.
///
/// Platform note: GLFW requires the window to be created on the main thread on
/// macOS. On Linux and Windows a dedicated thread is fine, which is what this
/// does.
class PendulumVisualizer
{
public:
  struct Config
  {
    double arm_length = 0.062;   ///< motor axis -> hinge          [m]
    double rod_length = 0.102;   ///< hinge -> bob centre          [m]
    double base_height = 0.070;  ///< enclosure height             [m]
    double arm_height = 0.089;   ///< hinge height above the table [m]
    double motor_limit = 2.35619449;
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

private:
  void run();

  Config cfg_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> q1_{0.0};
  std::atomic<double> q2_{3.14159265358979};
  std::atomic<double> sim_time_{0.0};
  std::atomic<double> sim_speed_{1.0};
  std::atomic<bool> fault_{false};
};

}  // namespace mock_pendulum_hardware

#endif  // MOCK_PENDULUM_HARDWARE__PENDULUM_VISUALIZER_HPP_
