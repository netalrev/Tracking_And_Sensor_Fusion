# Phase 2: Mathematical Engine (Extended Kalman Filter) Architecture

## 1. Phase Objective
The objective of this phase was to implement the Extended Kalman Filter (EKF) as a pure, isolated mathematical engine. The EKF is designed to fuse asynchronous, heterogeneous sensor data (Radar in spherical coordinates with radial velocity, EO in angles-only) into a unified 3D Cartesian state estimation, utilizing a Constant Velocity (CV) kinematic model.

## 2. Core Components
* **`EKF.h` / `EKF.cpp`**: A highly encapsulated class managing the 6D State Vector ($X$) and the 6x6 Covariance Matrix ($P$). It exposes public methods for prediction and sensor-specific updates, while keeping Jacobian derivations strictly private.

## 3. Architectural Decisions & Deep Understanding (Key Interview Points)

### A. Extreme Performance Optimization (Eigen Fixed-Size Matrices)
Real-time tracking systems require deterministic, low-latency execution.
* **Implementation:** The internal state and covariance matrices are defined using Eigen's fixed-size templates (`Eigen::Matrix<double, 6, 6>`). When matrix dimensions are known at compile-time and are sufficiently small, Eigen bypasses dynamic heap allocations entirely (allocating on the stack) and heavily utilizes CPU vectorization (SIMD) and loop unrolling. Conversely, the public API accepts dynamic `Eigen::VectorXd` to allow flexible interfacing without exposing the rigid internal memory layout.

### B. The Radial Velocity Initialization Trap
Initializing a 3D Cartesian track from a single Radar measurement presents a mathematical challenge: the Radar only measures Radial Velocity ($v_r$), leaving the tangential velocity completely unobservable.
* **Implementation:** During `init()`, the radial velocity is projected onto the Cartesian axes as an initial "best guess." Crucially, the velocity components of the Covariance Matrix ($P$) are deliberately inflated to a massive variance ($1000.0$). This instructs the Kalman Gain to inherently distrust this initial velocity guess and quickly converge to the true 3D velocity vector upon receiving subsequent positional updates.

### C. Dynamic Process Noise ($Q$) and the Time Delta ($\Delta t$)
In an asynchronous system, the time gap between measurements ($\Delta t$) fluctuates constantly.
* **Implementation:** The system utilizes the **Discrete White Noise Acceleration Model**. This correctly models how an unknown target acceleration affects position and velocity over time. The covariance of position grows with $\Delta t^4$, position-velocity covariance with $\Delta t^3$, and velocity covariance with $\Delta t^2$. This ensures that if a track coasts for a long duration, its positional uncertainty inflates exponentially, perfectly reflecting the physical reality of the uncertainty.

### D. Mathematical Guardrails: Normalization & Singularities
* **Angle Normalization:** The innovation step ($y = Z - h(X)$) explicitly normalizes angular differences to strictly remain within $[-\pi, \pi]$. This prevents catastrophic track divergence (e.g., treating the difference between $179^\circ$ and $-179^\circ$ as $358^\circ$ instead of $2^\circ$).
* **Singularity Protection:** When a target flies directly over the sensor ($X=0, Y=0$), the Jacobian formulas encounter a "Divide by Zero" condition (azimuth derivation depends on $X^2 + Y^2$). Guard clauses (`if d2 < 0.0001`) immediately abort the update step to prevent `NaN` values from permanently corrupting the covariance matrix.