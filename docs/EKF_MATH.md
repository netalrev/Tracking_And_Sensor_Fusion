# Mathematical Foundations: Multi-Sensor EKF Tracking

This document outlines the core mathematical models used in the Extended Kalman Filter (EKF) implementation for our tracking and sensor fusion system.

## 1. Coordinate System Conversions

The state vector is maintained in a 3D Cartesian coordinate system, while radar measurements are received in a Spherical coordinate system.

### 1.1 Cartesian to Spherical (Radar Measurements)
Given the Cartesian state $(X, Y, Z, V_x, V_y, V_z)$:

$$ R = \sqrt{X^2 + Y^2 + Z^2} $$
$$ \text{Azimuth (az)} = \text{atan2}(Y, X) $$
$$ \text{Elevation (el)} = \arcsin\left(\frac{Z}{R}\right) $$
$$ V_r = \frac{X \cdot V_x + Y \cdot V_y + Z \cdot V_z}{R} $$

### 1.2 Spherical to Cartesian
$$ X = R \cdot \cos(el) \cdot \cos(az) $$
$$ Y = R \cdot \cos(el) \cdot \sin(az) $$
$$ Z = R \cdot \sin(el) $$

---

## 2. Process Noise & Acceleration Kinematics

The system utilizes a Constant Velocity (CV) model. To account for target maneuvers, random acceleration $a$ is modeled as process noise over a time step $\Delta t$:

$$ V = a \cdot \Delta t $$
$$ X = \frac{1}{2} a \cdot (\Delta t)^2 $$

The standard deviation and variance for the velocity process noise are defined as:
$$ \sigma_v = \sigma_a \cdot \Delta t $$
$$ \text{Var}_v = \sigma_a^2 \cdot (\Delta t)^2 $$

---

## 3. Measurement Model - $h(x)$ and Jacobian $H$

### 3.1 Non-Linear Measurement Function $h(x)$
For a Radar sensor providing 4 measurements (Range, Azimuth, Elevation, Radial Velocity):

$$ h(x) = \begin{bmatrix} R \\ az \\ el \\ V_r \end{bmatrix} $$

### 3.2 Measurement Jacobian $H$ (Radar, $4 \times 6$)
The Jacobian matrix $H = \frac{\partial h}{\partial x}$ is computed to linearize the measurement model around the predicted state:

$$ H = \begin{bmatrix}
\frac{X}{R} & \frac{Y}{R} & \frac{Z}{R} & 0 & 0 & 0 \\
\\
-\frac{Y}{R^2} & \frac{X}{R^2} & 0 & 0 & 0 & 0 \\
\\
-\frac{X \cdot Z}{R^2 \sqrt{X^2+Y^2}} & -\frac{Y \cdot Z}{R^2 \sqrt{X^2+Y^2}} & \frac{\sqrt{X^2+Y^2}}{R^2} & 0 & 0 & 0 \\
\\
\frac{V_x \cdot R - X \cdot V_r}{R^2} & \frac{V_y \cdot R - Y \cdot V_r}{R^2} & \frac{V_z \cdot R - Z \cdot V_r}{R^2} & \frac{X}{R} & \frac{Y}{R} & \frac{Z}{R}
\end{bmatrix} $$

---

## 4. State Transition Model - $f(x, \Delta t)$ and Jacobian $F$

### 4.1 State Transition Function $f(x, \Delta t)$
The function propagates the 6-DOF state forward in time assuming constant velocity:

$$ f(x, \Delta t) = \begin{bmatrix} X + V_x \cdot \Delta t \\ Y + V_y \cdot \Delta t \\ Z + V_z \cdot \Delta t \\ V_x \\ V_y \\ V_z \end{bmatrix} $$

### 4.2 State Transition Jacobian $F$ ($6 \times 6$)
The linear transformation matrix $F$ used in the prediction step:

$$ F = \begin{bmatrix}
1 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 1 & 0 & 0 & \Delta t & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix} $$