# How to Build and Run — Multi-Sensor Tracking & Fusion System

## 1. Project Overview

A real-time multi-sensor target tracker fusing **Radar** (3D spherical) and **EO** (2D angular) measurements via an Extended Kalman Filter (EKF) with Global Nearest Neighbor (GNN) data association.

The repository contains:
- **C++ tracker** (`src/`, `include/`) — the core EKF pipeline, built with CMake
- **Python data generator** (`scripts/data_generator.py`) — produces synthetic sensor CSVs from `config/config.yaml`
- **Python evaluation & visualization scripts** (`scripts/evaluate_results.py`, `scripts/animate_fusion.py`, `scripts/run_experiments.py`)

---

## 2. Dependencies

### C++ (Linux / WSL)

| Tool | Minimum Version | Install command (Ubuntu / Debian) |
|---|---|---|
| GCC / G++ | 11 (C++17 required) | `sudo apt install build-essential` |
| CMake | 3.14 | `sudo apt install cmake` |
| Make | any | included in `build-essential` |
| Eigen3 | 3.3 | `sudo apt install libeigen3-dev` |

**Verified environment:** Ubuntu 24.04, g++ 13.3, CMake 4.4, Eigen 3.4.

### Python

| Tool | Minimum Version |
|---|---|
| Python | 3.9+ |
| pip packages | see `requirements.txt` |

Install Python dependencies (from the project root):

```bash
pip install -r requirements.txt
```

`requirements.txt` installs: `numpy>=1.24`, `pandas>=2.0`, `PyYAML>=6.0`, `matplotlib>=3.7`.

---

## 3. Build Instructions

### Linux / WSL (verified)

**Step 1 — Configure** (run once, or after adding/removing source files):

```bash
cd build
cmake ..
```

**Step 2 — Compile** (run after any C++ source change):

```bash
cd build
cmake --build .
```

This produces four executables inside `build/`:

| Executable | Purpose |
|---|---|
| `tracking_system` | The main tracker |
| `test_data_loader` | Unit test: CSV parsing + queue ordering |
| `test_ekf_math` | Unit test: EKF init / predict / update |
| `test_track_association` | Unit test: GNN data association |

> **Note:** The `build/` directory already contains a configured CMake project committed to the repository. If you are building from scratch (e.g., a fresh clone), run `cmake ..` first. For incremental rebuilds after code edits, `cmake --build .` is sufficient.

### Windows (native — not verified in this repository)

No Visual Studio solution (`.sln`) exists in the repository. The verified build path on Windows is **WSL** (follow the Linux instructions above inside a WSL terminal).

For a native Windows build with **MinGW-w64 + CMake**, the CMakeLists.txt is standard and should work, but this configuration has not been tested:

```cmd
mkdir build && cd build
cmake .. -G "MinGW Makefiles"
cmake --build .
```

You will need Eigen3 available on the include path. One option is [vcpkg](https://vcpkg.io):
```cmd
vcpkg install eigen3
cmake .. -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=[vcpkg root]/scripts/buildsystems/vcpkg.cmake
```

---

## 4. Generate Input Data

The C++ tracker consumes three files from `data/`:

| File | Description |
|---|---|
| `data/radar.csv` | Radar measurements (range, azimuth, elevation, radial velocity) |
| `data/eo.csv` | EO measurements (azimuth, elevation) |
| `data/sensor_noise.txt` | Noise standard deviations read by the C++ noise matrices |

All three are produced by the Python data generator. **The generator must be run from the `scripts/` directory:**

```bash
cd scripts
python data_generator.py
```

This reads `config/config.yaml` and writes:

- `data/radar.csv`
- `data/eo.csv`
- `data/ground_truth.csv` (used only by evaluation scripts)
- `data/sensor_noise.txt`
- `results/fig1_ground_truth.png` through `results/fig4_combined_physical_space.png`

> **Simulation is deterministic.** The config sets `random_seed: 42`, so the same `config.yaml` always produces the same CSV files.

To change the scenario (number of targets, sensor noise, drop probability, clutter rate, simulation duration), edit **`config/config.yaml`** and re-run the generator.

---

## 5. Run the Tracker

The tracker binary must be run from the `build/` directory (it resolves data paths as `../data/` and `../results/`):

```bash
cd build
./tracking_system
```

Expected console output:

```
=========================================
  Multi-Sensor Tracking & Fusion System  
=========================================

[TrackerManager] Loading data from sensors...
[TrackerManager] Data loaded. Queue size: 6776
[TrackerManager] Starting main tracking loop...
[TrackerManager] Tracking complete! Results saved to ../results/fused_tracks.csv

[SUCCESS] System gracefully terminated.
```

---

## 6. Output Files and Their Locations

| File | Produced by | Description |
|---|---|---|
| `results/fused_tracks.csv` | `tracking_system` | Fused track states at 10 Hz |
| `results/fig1_ground_truth.png` | `data_generator.py` | Ground truth trajectories |
| `results/fig2_measurements.png` | `data_generator.py` | Raw sensor measurements |
| `results/fig3_combined_sensor_space.png` | `data_generator.py` | GT + measurements in sensor space |
| `results/fig4_combined_physical_space.png` | `data_generator.py` | GT + measurements in 3D space |
| `results/fig5_*.png` through `fig7_*.png` | `evaluate_results.py` | Performance analysis plots |

### `fused_tracks.csv` format

```
timestamp,target_id,x,y,z,vx,vy,vz,status
0.1,1,944.4,3976.2,3039.4,8.21,34.59,26.44,TENTATIVE
...
```

- Positions are in **metres (Cartesian XYZ)**, velocities in **m/s**.
- `status` is `TENTATIVE` (fewer than 5 confirmed radar hits) or `CONFIRMED`.
- Exported at **10 Hz** (every 0.1 s).

---

## 7. Verification Steps

### 7.1 Unit Tests

Run all three unit tests from `build/`:

```bash
cd build
./test_data_loader
./test_ekf_math
./test_track_association
```

All three should exit with code 0. `test_data_loader` will report the queue size and confirm chronological ordering of all measurements.

### 7.2 Evaluate Tracking Performance

Run the evaluation script from `scripts/`:

```bash
cd scripts
python evaluate_results.py
```

This compares `results/fused_tracks.csv` against `data/ground_truth.csv` and prints a performance report (RMSE, track confirmation delay, false track count) plus saves analysis figures `fig5` through `fig7` to `results/`.

### 7.3 Quick Sanity Check

Run the tracker twice on the same data and confirm identical output:

```bash
cd build
./tracking_system && cp ../results/fused_tracks.csv /tmp/run1.csv
./tracking_system && cp ../results/fused_tracks.csv /tmp/run2.csv
cmp -s /tmp/run1.csv /tmp/run2.csv && echo "IDENTICAL" || echo "DIFFERS"
```

Expected result: `IDENTICAL`.

---

## 8. Optional: Animate the Fusion

Produces an animated GIF of the tracking system over time. Run from `scripts/`:

```bash
cd scripts
python animate_fusion.py
```

Requires `results/fused_tracks.csv` and `data/` files to exist.

---

## 9. Optional: Sensitivity Experiments

Runs a sweep over a configurable parameter (noise, clutter, drop rate) and plots how tracking performance degrades. Configure the sweep in **`config/exp_config.yaml`**, then run from `scripts/`:

```bash
cd scripts
python run_experiments.py
```

> **Note:** `run_experiments.py` modifies `config/config.yaml` during the sweep. It creates an automatic backup (`config/config_backup.yaml`) and restores the original on completion. Do not edit `config/config.yaml` while the script is running.

---

## 10. Full Pipeline Reference

```bash
# 1. Install C++ dependencies (once)
sudo apt install build-essential cmake libeigen3-dev

# 2. Install Python dependencies (once)
pip install -r requirements.txt

# 3. Configure CMake (once per fresh clone)
cd build && cmake .. && cd ..

# 4. Generate sensor data
cd scripts && python data_generator.py && cd ..

# 5. Build the tracker
cd build && cmake --build . && cd ..

# 6. Run the tracker
cd build && ./tracking_system && cd ..

# 7. Evaluate results
cd scripts && python evaluate_results.py && cd ..
```
