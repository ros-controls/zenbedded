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

#include "mock_pendulum_hardware/pendulum_visualizer.hpp"

#include <raylib.h>
#include <cmath>
#include <cstdio>

namespace mock_pendulum_hardware
{
namespace
{

/// raylib is y-up, so (x, y, z)_world -> (x, z, -y)_raylib (right-handed).
inline Vector3 conv(double x, double y, double z)
{
  return Vector3{static_cast<float>(x), static_cast<float>(z), static_cast<float>(-y)};
}
}  // namespace

PendulumVisualizer::PendulumVisualizer(const Config & cfg) : cfg_(cfg) {}

PendulumVisualizer::~PendulumVisualizer() { stop(); }

void PendulumVisualizer::start()
{
  if (running_.exchange(true))
  {
    return;
  }
  thread_ = std::thread(&PendulumVisualizer::run, this);
}

void PendulumVisualizer::stop()
{
  running_.store(false);
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void PendulumVisualizer::update(double q1, double q2, double sim_time, bool fault, double sim_speed)
{
  q1_.store(q1, std::memory_order_relaxed);
  q2_.store(q2, std::memory_order_relaxed);
  sim_time_.store(sim_time, std::memory_order_relaxed);
  fault_.store(fault, std::memory_order_relaxed);
  sim_speed_.store(sim_speed, std::memory_order_relaxed);
}

void PendulumVisualizer::run()
{
  SetTraceLogLevel(LOG_WARNING);
  InitWindow(cfg_.width, cfg_.height, "Furuta pendulum - mock hardware");
  SetTargetFPS(60);

  Camera3D cam{};
  cam.position = Vector3{0.30f, 0.20f, 0.30f};
  cam.target = Vector3{0.0f, 0.09f, 0.0f};
  cam.up = Vector3{0.0f, 1.0f, 0.0f};
  cam.fovy = 45.0f;
  cam.projection = CAMERA_PERSPECTIVE;

  const Color kBg{18, 20, 26, 255};
  const Color kEnclosure{64, 64, 72, 255};
  const Color kArm{102, 153, 204, 255};
  const Color kRod{204, 102, 102, 255};

  while (running_.load(std::memory_order_relaxed) && !WindowShouldClose())
  {
    UpdateCamera(&cam, CAMERA_ORBITAL);

    const double q1 = q1_.load(std::memory_order_relaxed);
    const double q2 = q2_.load(std::memory_order_relaxed);
    const bool fault = fault_.load(std::memory_order_relaxed);

    // Hinge position: the arm swings about +z at radius L1.
    const double hx = cfg_.arm_length * std::cos(q1);
    const double hy = cfg_.arm_length * std::sin(q1);
    const double hz = cfg_.arm_height;

    // Pendulum unit vector. In the arm frame it is (0, -sin q2, cos q2) —
    // q2 = 0 points along +z (upright) and rotates about the arm's own +x —
    // then rotated into the world by q1 about +z.
    const double ux = std::sin(q2) * std::sin(q1);
    const double uy = -std::sin(q2) * std::cos(q1);
    const double uz = std::cos(q2);

    const double tx = hx + cfg_.rod_length * ux;
    const double ty = hy + cfg_.rod_length * uy;
    const double tz = hz + cfg_.rod_length * uz;

    BeginDrawing();
    ClearBackground(kBg);
    BeginMode3D(cam);

    DrawGrid(12, 0.05f);

    // Enclosure and lid.
    DrawCubeV(
      conv(0.0, 0.0, cfg_.base_height * 0.5),
      Vector3{0.10f, static_cast<float>(cfg_.base_height), 0.10f}, kEnclosure);
    DrawCubeWiresV(
      conv(0.0, 0.0, cfg_.base_height * 0.5),
      Vector3{0.10f, static_cast<float>(cfg_.base_height), 0.10f}, Color{100, 100, 112, 255});
    DrawCubeV(
      conv(0.0, 0.0, cfg_.base_height + 0.0025), Vector3{0.104f, 0.005f, 0.104f}, kEnclosure);

    // Hard-stop markers at +-135 deg.
    // for (int s = -1; s <= 1; s += 2)
    // {
    //   const double a = s * cfg_.motor_limit;
    //   DrawCylinderEx(
    //     conv(0.045 * std::cos(a), 0.045 * std::sin(a), cfg_.base_height + 0.005),
    //     conv(0.045 * std::cos(a), 0.045 * std::sin(a), cfg_.base_height + 0.020), 0.004f, 0.004f,
    //     8, Color{160, 120, 60, 255});
    // }

    // Motor shaft and arm.
    DrawCylinderEx(
      conv(0.0, 0.0, cfg_.base_height), conv(0.0, 0.0, hz), 0.005f, 0.005f, 12,
      Color{140, 140, 150, 255});
    DrawCylinderEx(conv(0.0, 0.0, hz), conv(hx, hy, hz), 0.009f, 0.009f, 16, kArm);
    DrawSphere(conv(hx, hy, hz), 0.007f, Color{80, 120, 170, 255});

    // Pendulum rod and bob.
    DrawCylinderEx(conv(hx, hy, hz), conv(tx, ty, tz), 0.003f, 0.003f, 12, kRod);
    DrawSphere(conv(tx, ty, tz), 0.009f, Color{160, 70, 70, 255});

    // Upright reference.
    DrawCylinderEx(
      conv(hx, hy, hz), conv(hx, hy, hz + cfg_.rod_length), 0.0006f, 0.0006f, 6,
      Color{90, 200, 140, 120});

    EndMode3D();

    char buf[192];
    std::snprintf(
      buf, sizeof(buf), "arm %+7.2f deg    pendulum %+7.2f deg    t %7.2f s    %.2fx",
      q1 * 180.0 / M_PI, q2 * 180.0 / M_PI, sim_time_.load(std::memory_order_relaxed),
      sim_speed_.load(std::memory_order_relaxed));
    DrawText(buf, 14, 12, 18, RAYWHITE);
    DrawText("0 deg = upright   |   +-180 deg = hanging", 14, 36, 14, Color{150, 150, 160, 255});
    if (fault)
    {
      DrawText("HARD STOP / STEP LOSS", 14, cfg_.height - 30, 20, Color{230, 120, 60, 255});
    }
    EndDrawing();
  }

  CloseWindow();
  running_.store(false);
}

}  // namespace mock_pendulum_hardware
