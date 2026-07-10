# Phase 1: Data Ingestion & Event Layer Architecture

## 1. Phase Objective
The goal of this phase was to construct a robust, asynchronous data pipeline capable of bridging the gap between raw, heterogeneous sensor files (1Hz Radar, 10Hz EO) and the core tracking algorithm. The system establishes an **Event-Driven Architecture**, ensuring the tracking filter processes measurements in strict chronological order, mirroring real-time hardware environments.

## 2. Core Components
* **`Measurement.h`**: A polymorphic class hierarchy defining the data contract. It includes a pure virtual base class (`Measurement`) and concrete implementations for each sensor type (`RadarMeasurement`, `EOMeasurement`).
* **`SensorDataLoader`**: A static utility class acting as an Adapter. It parses the raw CSV files, handles unit conversions (Degrees to Radians), injects the corresponding statistical noise covariance ($R$), and populates the central queue.
* **`EventQueue` (Priority Queue)**: A `std::priority_queue` configured as a Min-Heap, acting as the primary Jitter Buffer to chronologically interleave asynchronous sensor data.

## 3. Architectural Decisions & Deep Understanding (Key Interview Points)

### A. The Min-Heap Priority Queue (Chronological Interleaving)
In standard C++, a `std::priority_queue` defaults to a Max-Heap (largest element on top). Because time flows forward (from $t=0.0$ upwards), we require the measurement with the *smallest* timestamp to be processed first. 
* **Implementation:** We injected a custom comparator (`MeasurementCompare`) that uses the greater-than (`>`) operator. This logically inverts the queue into a Min-Heap, guaranteeing strict time monotonicity for the Extended Kalman Filter (EKF) and preventing negative time steps ($\Delta t < 0$).

### B. Zero-Copy Memory Management
Sorting large matrix objects in memory is extremely expensive for CPU cycles. 
* **Implementation:** The `EventQueue` does not hold the objects themselves; it holds `std::unique_ptr<Measurement>`. When the queue sorts the measurements chronologically, it is only swapping 8-byte memory addresses rather than copying Eigen matrices. This provides a zero-overhead abstraction while guaranteeing that memory is automatically freed the moment the tracking loop finishes processing a measurement, entirely eliminating memory leaks.

### C. Polymorphism vs. Mathematical Performance
The interface demands polymorphic behavior so the Tracker can blindly pull a generic `Measurement` from the queue. However, allocating dynamic memory for matrices during the parsing loop degrades performance.
* **Implementation:** The virtual interface exposes dynamic-sized Eigen types (`Eigen::VectorXd`, `Eigen::MatrixXd`). However, internally, the derived classes store the data in strictly fixed-size stack arrays (`Eigen::Vector4d` for Radar, `Eigen::Vector2d` for EO). This satisfies the polymorphic interface requirements while retaining the blazing-fast stack allocation and vectorization optimizations of Eigen3 under the hood.

### D. Separation of Concerns (The Adapter Pattern)
The EKF should be an isolated mathematical engine, completely unaware of hardware specifications, file formats, or unit types (like degrees). 
* **Implementation:** The `SensorDataLoader` handles all real-world coupling. It translates degrees to radians and statically injects the Sensor Noise Covariance Matrix ($R$) into the measurement object. Consequently, the tracking algorithm receives mathematically pure data, making the system modular and highly extensible for future sensors.

## 4. Validation
The architecture was successfully validated via `test_data_loader.cpp`. The test explicitly parsed and enqueued 3,696 asynchronous measurements. A strict assertion loop verified that every sequentially popped element had a timestamp greater than or equal to its predecessor, guaranteeing $100\%$ time monotonicity and proving the system is ready to drive the EKF engine.