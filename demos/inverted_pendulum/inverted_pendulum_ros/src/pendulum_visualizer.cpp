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
#include <rlgl.h>

#include <cmath>
#include <cstdio>

namespace mock_pendulum_hardware
{
namespace
{

inline float dist3(Vector3 a, Vector3 b)
{
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
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

bool PendulumVisualizer::consume_disturbance()
{
  return disturb_trigger_.exchange(false, std::memory_order_relaxed);
}

void PendulumVisualizer::run()
{
  SetTraceLogLevel(LOG_WARNING);
  SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
  InitWindow(cfg_.width, cfg_.height, "Furuta Pendulum - Hardware Simulation");
  SetTargetFPS(60);

  Camera3D cam{};
  cam.position = Vector3{0.22f, 0.17f, 0.22f};
  cam.target = Vector3{0.0f, 0.065f, 0.0f};
  cam.up = Vector3{0.0f, 1.0f, 0.0f};
  cam.fovy = 40.0f;
  cam.projection = CAMERA_PERSPECTIVE;

  // Enforce locked distance to completely disable zoom
  const float locked_distance = dist3(cam.position, cam.target);

  // CAD Aesthetic Studio Palette matching reference render
  const Color kBg{20, 24, 30, 255};              // #14181e Deep studio backdrop
  const Color kEnclosure{34, 40, 52, 255};       // Dark matte charcoal box
  const Color kEnclosureBevel{50, 60, 75, 255};  // Subtle enclosure bevel
  const Color kLid{38, 44, 56, 255};             // Top cover lid
  const Color kGrillSlot{18, 22, 28, 255};       // Recessed side ventilation slit
  const Color kGrillEdge{48, 56, 70, 255};       // Slit highlight edge
  const Color kMotorCollar{26, 30, 38, 255};     // Motor bearing hub
  const Color kCoverMarking{90, 105, 130, 220};  // Cover limit tick lines
  const Color kCoverArc{60, 72, 92, 160};        // Travel range arc
  const Color kArm{46, 126, 218, 255};           // Vibrant electric/cobalt blue
  const Color kArmHighlight{66, 146, 238, 255};  // Arm chamfer highlight
  const Color kJointBore{24, 28, 38, 255};       // Bearing recess in blue arm
  const Color kPendulum{216, 68, 52, 255};       // Coral / tomato red paddle
  const Color kPendulumEdge{236, 88, 72, 255};   // Paddle edge highlight

  // Interactive Distort Button UI coordinates
  const Rectangle time_card_rec{18.0f, 18.0f, 146.0f, 32.0f};
  const Rectangle distort_btn_rec{18.0f, 56.0f, 146.0f, 32.0f};

  while (running_.load(std::memory_order_relaxed) && !WindowShouldClose())
  {
    // Update camera orbital rotation, then lock distance to disable zoom
    UpdateCamera(&cam, CAMERA_ORBITAL);
    Vector3 to_cam{
      cam.position.x - cam.target.x, cam.position.y - cam.target.y, cam.position.z - cam.target.z};
    float current_len = std::sqrt(to_cam.x * to_cam.x + to_cam.y * to_cam.y + to_cam.z * to_cam.z);
    if (current_len > 1e-4f)
    {
      float scale = locked_distance / current_len;
      cam.position = Vector3{
        cam.target.x + to_cam.x * scale, cam.target.y + to_cam.y * scale,
        cam.target.z + to_cam.z * scale};
    }

    // Handle interactive disturbance button / Space key
    const Vector2 mouse_pos = GetMousePosition();
    const bool is_hover_btn = CheckCollisionPointRec(mouse_pos, distort_btn_rec);
    const bool is_pressed_btn =
      (is_hover_btn && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) || IsKeyPressed(KEY_SPACE);
    if (is_pressed_btn)
    {
      disturb_trigger_.store(true, std::memory_order_relaxed);
    }

    const double q1 = q1_.load(std::memory_order_relaxed);
    const double q2 = q2_.load(std::memory_order_relaxed);
    const double sim_time = sim_time_.load(std::memory_order_relaxed);

    BeginDrawing();
    ClearBackground(kBg);

    BeginMode3D(cam);

    // Subtle soft floor shadow beneath the box
    DrawCircle3D(
      Vector3{0.0f, 0.0005f, 0.0f}, 0.075f, Vector3{1.0f, 0.0f, 0.0f}, 90.0f,
      Color{12, 15, 20, 160});

    // -------------------------------------------------------------------------
    // 1. Base Enclosure Box & Ventilation Grill
    // -------------------------------------------------------------------------
    const float box_w = 0.076f;
    const float box_d = 0.076f;
    const float box_h = static_cast<float>(cfg_.base_height);

    // Main box body
    DrawCubeV(Vector3{0.0f, box_h * 0.5f, 0.0f}, Vector3{box_w, box_h, box_d}, kEnclosure);
    DrawCubeWiresV(
      Vector3{0.0f, box_h * 0.5f, 0.0f}, Vector3{box_w, box_h, box_d}, kEnclosureBevel);

    // Top cover lid
    DrawCubeV(
      Vector3{0.0f, box_h + 0.001f, 0.0f}, Vector3{box_w + 0.002f, 0.002f, box_d + 0.002f}, kLid);
    DrawCubeWiresV(
      Vector3{0.0f, box_h + 0.001f, 0.0f}, Vector3{box_w + 0.002f, 0.002f, box_d + 0.002f},
      kEnclosureBevel);

    // Vertical ventilation slots on the front face (Z = box_d / 2)
    const float face_z = box_d * 0.5f + 0.0003f;
    for (float sx = -0.024f; sx <= 0.024f; sx += 0.0035f)
    {
      DrawLine3D(Vector3{sx, 0.008f, face_z}, Vector3{sx, 0.030f, face_z}, kGrillSlot);
      DrawLine3D(
        Vector3{sx + 0.0006f, 0.008f, face_z}, Vector3{sx + 0.0006f, 0.030f, face_z}, kGrillEdge);
    }

    // Top motor collar
    DrawCylinderEx(
      Vector3{0.0f, box_h + 0.001f, 0.0f}, Vector3{0.0f, box_h + 0.006f, 0.0f}, 0.011f, 0.011f, 20,
      kMotorCollar);

    // -------------------------------------------------------------------------
    // 2. Cover Travel Limit Markings (Replaces physical stoppers)
    // -------------------------------------------------------------------------
    const float cover_y = box_h + 0.0022f;
    const float limit_rad = static_cast<float>(cfg_.motor_limit);

    // Radial tick lines indicating +-135 deg travel boundary
    for (int sign = -1; sign <= 1; sign += 2)
    {
      const float angle = sign * limit_rad;
      const Vector3 p_in{0.016f * std::cos(angle), cover_y, -0.016f * std::sin(angle)};
      const Vector3 p_out{0.033f * std::cos(angle), cover_y, -0.033f * std::sin(angle)};
      DrawLine3D(p_in, p_out, kCoverMarking);
    }

    // Arc on cover indicating permissible travel envelope
    constexpr int kArcSegs = 32;
    for (int i = 0; i < kArcSegs; ++i)
    {
      const float a0 = -limit_rad + (2.0f * limit_rad * i) / kArcSegs;
      const float a1 = -limit_rad + (2.0f * limit_rad * (i + 1)) / kArcSegs;
      const Vector3 p0{0.025f * std::cos(a0), cover_y, -0.025f * std::sin(a0)};
      const Vector3 p1{0.025f * std::cos(a1), cover_y, -0.025f * std::sin(a1)};
      DrawLine3D(p0, p1, kCoverArc);
    }

    // -------------------------------------------------------------------------
    // 3. Rotating Blue Arm & Non-Overlapping Precision Joint
    // -------------------------------------------------------------------------
    rlPushMatrix();
    rlTranslatef(0.0f, static_cast<float>(cfg_.arm_height), 0.0f);
    rlRotatef(static_cast<float>(q1 * 180.0 / M_PI), 0.0f, 1.0f, 0.0f);

    const float arm_len = static_cast<float>(cfg_.arm_length);
    const float arm_w = 0.020f;
    const float arm_h = 0.018f;

    // Blue rectangular arm body (ends flush at X = arm_len)
    DrawCubeV(Vector3{arm_len * 0.5f, 0.0f, 0.0f}, Vector3{arm_len, arm_h, arm_w}, kArm);
    DrawCubeWiresV(
      Vector3{arm_len * 0.5f, 0.0f, 0.0f}, Vector3{arm_len, arm_h, arm_w}, kArmHighlight);

    // Rounded proximal end (centered at motor axis X = 0)
    DrawCylinderEx(
      Vector3{0.0f, -arm_h * 0.5f, 0.0f}, Vector3{0.0f, arm_h * 0.5f, 0.0f}, arm_w * 0.5f,
      arm_w * 0.5f, 16, kArm);

    // Flat distal end face at X = arm_len with circular bearing recess
    DrawCylinderEx(
      Vector3{arm_len - 0.001f, 0.0f, 0.0f}, Vector3{arm_len + 0.0003f, 0.0f, 0.0f}, 0.0065f,
      0.0065f, 16, kJointBore);

    // Red cylindrical connector pin extending out from arm bore into pendulum hub
    const float pin_len = 0.005f;
    DrawCylinderEx(
      Vector3{arm_len, 0.0f, 0.0f}, Vector3{arm_len + pin_len, 0.0f, 0.0f}, 0.0042f, 0.0042f, 16,
      kPendulum);

    // -------------------------------------------------------------------------
    // 4. Red Lollipop Pendulum (Proportional, clears bottom of box, zero overlap)
    // -------------------------------------------------------------------------
    const float hub_thickness = 0.006f;
    rlTranslatef(arm_len + pin_len + hub_thickness * 0.5f, 0.0f, 0.0f);
    rlRotatef(static_cast<float>(q2 * 180.0 / M_PI), 1.0f, 0.0f, 0.0f);

    // Base hinge hub disc (normal along X)
    DrawCylinderEx(
      Vector3{-hub_thickness * 0.5f, 0.0f, 0.0f}, Vector3{hub_thickness * 0.5f, 0.0f, 0.0f}, 0.008f,
      0.008f, 20, kPendulum);
    DrawCylinderWiresEx(
      Vector3{-hub_thickness * 0.5f, 0.0f, 0.0f}, Vector3{hub_thickness * 0.5f, 0.0f, 0.0f}, 0.008f,
      0.008f, 20, kPendulumEdge);

    const float rod_len = static_cast<float>(cfg_.rod_length);  // 0.052m
    const float paddle_radius = 0.013f;
    const float stem_len = rod_len - paddle_radius * 0.7f;

    // Stem connecting hub to top paddle
    DrawCubeV(Vector3{0.0f, stem_len * 0.5f, 0.0f}, Vector3{0.0045f, stem_len, 0.006f}, kPendulum);
    DrawCubeWiresV(
      Vector3{0.0f, stem_len * 0.5f, 0.0f}, Vector3{0.0045f, stem_len, 0.006f}, kPendulumEdge);

    // Top circular paddle disc (lollipop bob)
    DrawCylinderEx(
      Vector3{-0.0025f, rod_len, 0.0f}, Vector3{0.0025f, rod_len, 0.0f}, paddle_radius,
      paddle_radius, 28, kPendulum);
    DrawCylinderWiresEx(
      Vector3{-0.0025f, rod_len, 0.0f}, Vector3{0.0025f, rod_len, 0.0f}, paddle_radius,
      paddle_radius, 28, kPendulumEdge);

    rlPopMatrix();

    EndMode3D();

    // -------------------------------------------------------------------------
    // 5. Clean Sim Time Card & Distort Button (No status indicator, no flicker)
    // -------------------------------------------------------------------------
    // Sim Time Display Card
    DrawRectangleRounded(time_card_rec, 0.35f, 6, Color{24, 28, 36, 210});
    DrawRectangleRoundedLinesEx(time_card_rec, 0.35f, 6, 1.2f, Color{50, 60, 75, 255});
    DrawText("TIME", 32, 27, 11, Color{150, 165, 185, 255});

    char time_str[24];
    std::snprintf(time_str, sizeof(time_str), "%5.1f s", sim_time);
    DrawText(time_str, 74, 26, 14, Color{225, 235, 248, 255});

    // Distort Pendulum Button
    const Color btn_bg =
      is_hover_btn
        ? (IsMouseButtonDown(MOUSE_BUTTON_LEFT) ? Color{70, 95, 135, 240} : Color{45, 58, 78, 230})
        : Color{32, 40, 54, 210};
    const Color btn_border = is_hover_btn ? Color{80, 135, 210, 255} : Color{50, 65, 85, 255};
    const Color btn_text = is_hover_btn ? Color{245, 250, 255, 255} : Color{190, 205, 225, 255};

    DrawRectangleRounded(distort_btn_rec, 0.35f, 6, btn_bg);
    DrawRectangleRoundedLinesEx(distort_btn_rec, 0.35f, 6, 1.2f, btn_border);
    DrawText("DISTORT", 46, 65, 12, btn_text);

    // Subtle footer instruction
    DrawText(
      "Orbit: Drag  |  Distort: Space or Click Button", 18, cfg_.height - 22, 11,
      Color{75, 85, 105, 180});

    EndDrawing();
  }

  CloseWindow();
  running_.store(false);
}

}  // namespace mock_pendulum_hardware
