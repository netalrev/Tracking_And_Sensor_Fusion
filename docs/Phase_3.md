# Phase 3: Entity Layer & Data Association Architecture

## 1. Phase Objective
The objective of this phase was to construct the logical wrappers and association mechanisms required to elevate pure EKF mathematics into a multi-target tracking system. This involves managing the lifecycle of distinct targets (Tracks) and statistically associating incoming asynchronous measurements to these tracks using the Mahalanobis distance.

## 2. Core Components
* **`Track.h` / `Track.cpp`**: The Entity Layer. A class representing a single real-world target. It encapsulates an `EKF` instance and manages the M/N survival logic (Hits vs. Misses) through a finite state machine (`TENTATIVE`, `CONFIRMED`, `DEAD`).
* **`DataAssociation.h` / `DataAssociation.cpp`**: A stateless logic engine implementing the Global Nearest Neighbor (GNN) assignment strategy coupled with a Chi-Square statistical gate.

## 3. Architectural Decisions & Deep Understanding (Key Interview Points)

### A. Composition vs. Inheritance (Decoupling Mathematics from Entities)
* **Implementation:** The `Track` class *contains* an `EKF` object rather than inheriting from it. 
* **Reasoning:** In modern C++ software architecture, prefer composition over inheritance for "Has-A" relationships. A track is not a mathematical filter; it is an entity that *uses* a filter. This strict decoupling allows the system to swap out the EKF for an IMM (Interacting Multiple Model) or Particle Filter in the future without altering the `Track` API or the Tracker Manager logic.

### B. M/N Logic for Clutter Rejection & Track Survivability
Radar systems naturally produce Poisson-distributed clutter (false alarms).
* **Implementation:** New tracks are initialized in a `TENTATIVE` state. They are only promoted to `CONFIRMED` after a consecutive "hit streak" (e.g., 3 updates). Conversely, if a track misses an update (sensor occlusion or drop), it does not die immediately; it begins "coasting." The missed counter increments, and the track is only marked `DEAD` after crossing a critical threshold (e.g., 5 misses). This hysteresis prevents the system from generating "Ghost Tracks" and ensures resilient tracking through transient sensor failures.

### C. Statistical Gating via Mahalanobis Distance
Using Euclidean distance to associate measurements fails spectacularly when dealing with highly maneuvering targets or targets with large uncertainties.
* **Implementation:** We utilize the **Mahalanobis Distance** ($D^2 = y^T S^{-1} y$). This distance metric scales the positional error by the innovation covariance ($S$), thereby accounting for both the kinematics of the target (where it's heading) and the filter's confidence. 
* **Gating:** A strict statistical gate based on the Chi-Square ($\chi^2$) distribution (99% confidence interval) is applied. The threshold dynamically adapts to the sensor's degrees of freedom ($13.277$ for 4D Radar, $9.210$ for 2D EO). Any measurement falling outside this gate is aggressively rejected as Clutter or a new potential target.

### D. Zero-dt Fusion Handling
* **Implementation:** The `Track::predict()` method explicitly guards against $\Delta t \le 0.0$.
* **Reasoning:** In asynchronous Sensor Fusion, a Radar and an EO measurement may arrive with the exact same millisecond timestamp. Predicting with $\Delta t = 0$ neutralizes the process noise ($Q$), potentially corrupting the state covariance matrix. By bypassing the predict step and proceeding directly to the update step, the architecture naturally supports synchronous, multi-sensor batch updating at identical timestamps.