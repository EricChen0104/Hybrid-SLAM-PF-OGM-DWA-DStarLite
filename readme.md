# Hybrid-SLAM-PF-OGM-DWA-DStarLite

A modular SLAM system combining **Particle Filter-based localization**, **Occupancy Grid Mapping (OGM)**, **Dynamic Window Approach (DWA)** for real-time obstacle avoidance, and **D\* Lite** for global path replanning. This project integrates both **probabilistic mapping** and **real-time motion planning**, suitable for research and educational use in robotics.

## DEMO

![](https://github.com/EricChen0104/Hybrid-SLAM-PF-OGM-DWA-DStarLite/blob/master/assets/DEMO_gif.gif)

## 🔬 Motivation

Simultaneous Localization and Mapping (SLAM) is a fundamental problem in mobile robotics. While Particle Filters offer robustness in non-Gaussian uncertainty settings, global planning via D\* Lite and local trajectory optimization via DWA are essential for real-world navigation in dynamic environments. This project aims to provide a hybrid SLAM framework that is:

- Easy to modify and extend for research or teaching purposes
- Built with clarity and modularity in mind
- Capable of real-time replanning under environmental changes

## 🧠 Core Algorithms

### 1. Particle Filter (Monte Carlo Localization)

Used for **pose estimation**, leveraging a weighted set of particles. Each particle represents a possible robot pose, and the weight update is based on a LIDAR-like sensor model using the occupancy map.

### 2. Occupancy Grid Mapping

The environment is discretized into a 2D grid, where each cell stores the probability of being occupied. Sensor updates are applied using log-odds to incrementally build the map.

### 3. Dynamic Window Approach (DWA)

Used for **real-time local obstacle avoidance**. Evaluates a set of velocity commands within a dynamic window to find safe and efficient motions towards a local goal.

### 4. D* Lite (Dynamic A*)

An incremental heuristic path planning algorithm that handles dynamic environments by updating paths on the fly when map changes are detected.

## 🗂️ Project Structure

```bash

assets/                      # (Optional) DEMO video
PF_OGM_DWA_D_STAR_LITE.py    # Main script: integrates all modules
README.md                    # Documentation

```

## ▶️ How to Run

```bash

python PF_OGM_DWA_D_STAR_LITE.py

```

Requirements:

- Python 3.8+
- numpy, matplotlib, scipy (for visualization and math)
- (Optional) Use pygame or OpenCV if integrated into GUI-based simulation

## 🔁 Algorithm Interaction Overview

In this hybrid SLAM system, each algorithm serves a complementary role in enabling robust autonomous navigation. The interaction between the modules is orchestrated in a closed perception-action loop, as follows:

Perception Loop (Mapping & Localization)

The robot receives noisy range measurements (e.g., from a simulated LIDAR).

Each Particle Filter particle proposes a hypothetical robot pose. These hypotheses are weighted based on how well they explain the current observation using the current Occupancy Grid Map (OGM).

The map is updated by integrating sensor readings (e.g., beam endpoints and free space) in log-odds form, from the best-estimated pose (i.e., weighted average of particles).

Planning Loop (Global and Local)

Given the robot's current estimated pose and the occupancy grid:

A global goal is specified (or dynamically updated).

D* Lite computes a path from the current position to the goal, using the occupancy grid as a cost map. If obstacles change, D* Lite incrementally updates only the affected parts of the path tree.

The current local waypoint (a segment of the global path) is sent to DWA for short-horizon motion planning.

Action Execution (Safe Motion Selection)

DWA samples a set of feasible (v, ω) velocity pairs within the dynamic limits of the robot.

Each candidate trajectory is:

Simulated forward for a short time,

Scored based on proximity to obstacles (from OGM), heading to the local goal, and velocity smoothness.

The best trajectory is selected and executed.

The executed motion alters the robot’s true pose (in simulation), feeding back into the next sensor update.

### ⛓️ Summary of Interaction Flow

```bash

[Sensor Data]
     ↓
[Particle Filter] ← Occupancy Map
     ↓                         ↑
[Pose Estimate] ──────────────┘
     ↓
[D* Lite Planner] ← Occupancy Map
     ↓
[Global Path → Local Goal]
     ↓
[DWA Local Planner] ← Occupancy Map
     ↓
[Safe Velocity Command]
     ↓
[Motion Execution]
     ↓
[Updated True Pose → Sensor Data]

```

## 📊 Visualization

- Particles are plotted to show belief distribution
- Occupancy grid is updated in real-time
- DWA trajectories shown as candidate curves
- D\* Lite replans are highlighted on the map

## 🔧 Features

- Real-time obstacle avoidance and global replanning
- Sensor noise and uncertainty modeling
- Efficient map update with log-odds filtering
- Modular structure: each component can be tested independently

## 📩 Contact & Citation

If you use this work, please consider citing it or leaving a star 🌟 to support future development.
Created by GUAN-YING, CHEN, 2025.

## 📚 References

Djuric, P. M., Kotecha, J. H., Zhang, J., Huang, Y., Ghirmai, T., Bugallo, M. F., & Miguez, J. (2003). Particle filtering. IEEE signal processing magazine, 20(5), 19-38.

Fox, D., Burgard, W., & Thrun, S. (2002). The dynamic window approach to collision avoidance. IEEE robotics & automation magazine, 4(1), 23-33.

Koenig, S., & Likhachev, M. (2002, July). D\* lite. In Eighteenth national conference on Artificial intelligence (pp. 476-483).
