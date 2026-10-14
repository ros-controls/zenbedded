#!/usr/bin/env python3
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

"""
Linear Quadratic Regulator (LQR) Solver for Furuta Rotary Inverted Pendulum.

This utility linearizes the coupled 2-DOF Furuta pendulum equations of motion
about the upright equilibrium (q2 = 0) with motor acceleration command input (u = q1_ddot),
solves the continuous Algebraic Riccati Equation (ARE), computes the state-feedback
gain matrix K, and evaluates closed-loop stability.

State vector:
    x = [q1, q1_dot, q2, q2_dot]^T
where:
    q1     : motor arm angle [rad]
    q1_dot : motor arm angular velocity [rad/s]
    q2     : pendulum angle relative to upright [rad] (q2 = 0 is upright)
    q2_dot : pendulum angular velocity [rad/s]

Control law:
    u = -K * x = -(k_q1 * q1 + k_q1_vel * q1_dot + k_q2 * q2 + k_q2_vel * q2_dot)
"""

import sys


def solve_furuta_lqr():
    try:
        import numpy as np
        import scipy.linalg
    except ImportError as e:
        print(f"Error: Missing required scientific Python package: {e}")
        print("Please install numpy and scipy: pip install numpy scipy")
        sys.exit(1)

    # Physical parameters matching CAD model & URDF
    m2 = 0.014  # Pendulum mass [kg]
    l2 = 0.051  # Pendulum hinge to COM distance [m]
    L1 = 0.062  # Motor arm reach to hinge [m]
    J2 = 4.447e-5  # Pendulum moment of inertia about pivot [kg*m^2]
    g = 9.80665  # Gravitational acceleration [m/s^2]
    b2 = 1.0e-4  # Hinge viscous damping [N*m*s/rad]

    # Linearized continuous-time state-space matrices around q2 = 0:
    #   q1_ddot = u
    #   J2 * q2_ddot = m2*g*l2 * q2 - b2 * q2_dot - m2*L1*l2 * u
    # State: x = [q1, q1_dot, q2, q2_dot]^T
    A = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, (m2 * g * l2) / J2, -b2 / J2],
        ]
    )

    B = np.array(
        [
            [0.0],
            [1.0],
            [0.0],
            [-(m2 * L1 * l2) / J2],
        ]
    )

    # LQR weighting matrices
    # Q penalizes state deviations: [q1, q1_dot, q2, q2_dot]
    Q = np.diag([5.0, 0.5, 80.0, 3.0])

    # R penalizes control effort (commanded motor acceleration)
    R = np.array([[0.2]])

    # Solve Continuous Algebraic Riccati Equation: A^T P + P A - P B R^-1 B^T P + Q = 0
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)

    # Closed-loop system matrix and eigenvalues
    A_cl = A - B @ K
    eigvals = np.linalg.eigvals(A_cl)

    # Map gains to controller coordinate convention:
    # In InvertedPendulumController:
    # u = -(k_q1 * q1 + k_q1_vel * q1d + k_q2 * q2 + k_q2_vel * q2d)
    # Because B[3] is negative, K[0, 2] and K[0, 3] from Riccati have negative signs;
    # in the controller sign convention:
    k_q1 = K[0, 0]
    k_q1_vel = K[0, 1]
    k_q2 = -K[0, 2]
    k_q2_vel = -K[0, 3]

    print("=================================================================")
    print(" Furuta Pendulum LQR Gain Computation")
    print("=================================================================")
    print(f"Plant: m2={m2} kg, l2={l2} m, L1={L1} m, J2={J2:.3e} kg*m^2, g={g}")
    q_diag = [float(x) for x in np.diag(Q)]
    print(f"Cost:  Q=diag({q_diag}), R={float(R[0, 0])}")
    print("-----------------------------------------------------------------")
    print("Computed LQR Feedback Gains:")
    print(f"  k_q1     : {k_q1:10.4f}")
    print(f"  k_q1_vel : {k_q1_vel:10.4f}")
    print(f"  k_q2     : {k_q2:10.4f}")
    print(f"  k_q2_vel : {k_q2_vel:10.4f}")
    print("-----------------------------------------------------------------")
    print("Closed-loop poles (all Re(lambda) < 0 ensures asymptotic stability):")
    for pole in eigvals:
        print(f"  {pole.real:+.4f} {pole.imag:+.4f}j")
    print("-----------------------------------------------------------------")
    print("YAML parameters snippet for controllers.yaml:")
    print(f"    k_q1:     {k_q1:.3f}")
    print(f"    k_q1_vel: {k_q1_vel:.3f}")
    print(f"    k_q2:     {k_q2:.3f}")
    print(f"    k_q2_vel: {k_q2_vel:.3f}")
    print("=================================================================")


if __name__ == "__main__":
    solve_furuta_lqr()
