# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import scipy.linalg

# Define physical system parameters extracted from the URDF model
M = 0.026  # Arm mass (kg)
m = 0.014  # Pendulum mass (kg)
r = 0.062  # Arm radius/reach to pendulum pivot (m)
d = 0.051  # Pendulum length to COM (m)
g = 9.81

# Simplified linearized matrices for acceleration input
# A matrix [p, p_dot, theta, theta_dot]
A = np.array([[0, 1, 0, 0], [0, 0, -m * g / M, 0], [0, 0, 0, 1], [0, 0, (M + m) * g / (M * d), 0]])

# B matrix (acceleration input)
B = np.array([[0], [1.0 / M], [0], [-1.0 / (M * d)]])

# Define Cost Matrices Q and R (Your tuning knobs)
# Q penalizes state deviation: [cart_pos, cart_vel, pendulum_pos, pendulum_vel]
Q = np.diag([10.0, 1.0, 100.0, 10.0])

# R penalizes control effort (acceleration magnitude)
R = np.array([[0.1]])

# Solve LQR Algebraic Riccati Equation
P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ (B.T @ P)

print("Calculated LQR Gains (K):")
print(f"k_motor_pos: {K[0, 0]:.4f}")
print(f"k_motor_vel: {K[0, 1]:.4f}")
print(f"kp         : {K[0, 2]:.4f}")
print(f"kd         : {K[0, 3]:.4f}")
