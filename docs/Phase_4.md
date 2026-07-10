# Phase 4: Tracker Orchestration & I/O Management Architecture

## 1. Phase Objective
The final phase of the C++ architecture unifies the Event Queue, the Mathematical EKF Engine, and the Data Association logic into a cohesive, automated execution loop. The objective was to build an orchestrator (`TrackerManager`) that processes the chronological queue in real-time, manages track lifecycles, and safely exports the fused situational awareness picture.

## 2. Core Components
* **`TrackerManager.h` / `TrackerManager.cpp`**: The central orchestrator. It holds ownership of all active targets (`std::vector<std::unique_ptr<Track>>`), processes the priority queue, routing measurements through Predict, Associate, and Update phases, and manages safe memory cleanup.
* **`main.cpp`**: A clean, minimal entry point. It enforces the Dependency Injection paradigm by passing configurations to the manager and providing top-level Exception Handling to prevent silent core dumps.

## 3. Architectural Decisions & Deep Understanding (Key Interview Points)

### A. The Zero-Copy Event Loop
Performance in high-frequency tracking systems degrades rapidly if memory is constantly copied.
* **Implementation:** Within the main `run()` loop, the system uses `event_queue_.top().get()` to extract a raw pointer to the measurement without copying the underlying Eigen matrices or triggering reallocation. The measurement is processed by the entire system using this raw pointer. Only at the end of the iteration is `event_queue_.pop()` called, which instantly destroys the `unique_ptr` and frees the memory in $O(1)$ time. This guarantees zero memory leaks and maximum CPU cache efficiency.

### B. Starvation-Based Track Deletion
In traditional radar tracking, tracks are deleted if they miss $X$ consecutive "scans". However, in an asynchronous event-driven system, the concept of a "scan" does not exist.
* **Implementation:** The `cleanupDeadTracks` function implements Starvation-based logic. It evaluates the physical time elapsed since the track's `last_update_time`. If the starvation period exceeds a physical threshold (e.g., $3.0$ seconds), the track is purged. This provides a robust, sensor-agnostic method for tracking target loss.

### C. The Erase-Remove Idiom
Deleting objects from the middle of a continuous memory block (like `std::vector`) causes massive performance penalties due to memory shifting.
* **Implementation:** Track deletion utilizes the C++ standard **Erase-Remove Idiom** (`std::erase(std::remove_if(...))`). This algorithm efficiently shifts all "dead" tracks to the end of the vector in a single pass before physically truncating the memory allocation, completely avoiding fragmented memory shifts.

### D. I/O Filtering (Throttling)
Disk I/O is often the primary bottleneck in C++ real-time systems.
* **Implementation:** Although the internal tracking filter updates dynamically based on the asynchronous sensor rates (e.g., 10Hz EO, 1Hz Radar), the `exportTrackStates` function is throttled to execute only at fixed intervals (e.g., $\Delta t \ge 0.1$ seconds). This prevents the system from overwhelming the disk or network with redundant output states, maintaining a deterministic execution time for the core tracking loop.