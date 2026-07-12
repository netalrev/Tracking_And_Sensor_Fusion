# Multi-Sensor Tracking & Fusion System

A real-time multi-sensor target tracking system that fuses **Radar** (3D spherical) and **Electro-Optical (EO)** measurements using an **Extended Kalman Filter (EKF)** with statistical data association.

## Overview

This project implements a complete tracking pipeline capable of handling:
- Asynchronous multi-rate sensors (Radar @ 1Hz, EO @ 10Hz)
- Realistic sensor noise, clutter, and measurement drops
- Continuous tracking during temporary sensor outages
- Real-time performance requirements

The system is built in **C++17** with **Eigen** for linear algebra, following an event-driven architecture.

## Key Features

- **Extended Kalman Filter** with 6-state Constant Velocity model
- **Adaptive process noise (Q)** based on predicted target speed
- **Dynamic measurement noise (R)** with Safety Factor
- **Mahalanobis Distance** gating with χ² validation thresholds
- **Scan Mutex** for one-to-one data association
- **Track lifecycle management** (TENTATIVE → CONFIRMED → DEAD)
- Support for track coasting during measurement drops

## Project Structure

| Path                  | Description                                      |
|-----------------------|--------------------------------------------------|
| `src/`                | Core C++ implementation (EKF, Track, DataAssociation, etc.) |
| `include/`            | Header files                                     |
| `tests/`              | Unit tests                                       |
| `scripts/`            | Python scripts for data generation and evaluation |
| `config/`             | Configuration files (YAML)                       |
| `data/`               | Generated sensor data (CSV files)                |
| `results/`            | Output fused tracks and analysis plots           |
| `docs/`               | Design documents, mathematical formulas, and appendices |
| `CMakeLists.txt`      | CMake build configuration                        |
| `HOW_TO_RUN.md`       | Detailed instructions to build and run the project |
| `requirements.txt`    | Python dependencies                              |

---


## Documentation

- [Design Document](docs/Design_Document_Tracking_Sensor_Fusion.pdf)
- [Mathematical Formulas (EKF)](docs/Math_Formulas_EKF.docx)
- [Experiments & Results](docs/Experiments_Appendix_Simulation_Results.pdf)
- [Visual Analysis Appendix](docs/Design_Appendix_Visual_Analysis.pdf)

## How to Build and Run

Please refer to the detailed guide:

→ [HOW_TO_RUN.md](HOW_TO_RUN.md)

## Technologies

- **C++17** + **CMake**
- **Eigen3** (linear algebra)
- **Python 3.9+** (data generation & evaluation)
- **NumPy, Pandas, Matplotlib, PyYAML**

## Design Highlights

- Event-driven architecture with priority queue for sensor synchronization
- Radar-only track initiation for reliable 3D initialization
- Greedy Nearest Neighbor + Scan Mutex instead of full MHT (performance trade-off)
- Adaptive noise models instead of IMM (computational efficiency)

## Author

**Netanel Reuven**  
Tracking & Sensor Fusion Project – TSG Algorithm Developer Position