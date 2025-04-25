# ROB-124 – Robotic Navigation (Reference Repository)

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) 
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-lightgrey)
![Status](https://img.shields.io/badge/Status-Reference%20Material-green)

Welcome to the ROB-124 repository!  
This repository contains **reference materials**, **example code**, and **project templates** for students enrolled in *ROB-124: Robotic Navigation* at WSU Tech.

**⚠ Important:**  
This repository is **not meant to be cloned and run directly**.  
Students must **create their own ROS packages** and **build their own working projects** by following the examples provided here.

---

## 📦 Repository Structure

| Folder | Description |
|:---|:---|
| `/arduino/` | Example Arduino sketches for encoder simulation |
| `/cpp/` | Example C++ scripts for ROS nodes (odometry, RViz click handling) |
| `/launch/` | Example ROS launch files |
| `readme.md` | This file |

---

## 🛠 Student Instructions

1. **Study the example files** carefully.
2. **Create your own ROS package** inside your `catkin_ws/src/` workspace.
3. **Recreate the necessary scripts, launch files, and folder structures** by using these examples as guides.
4. **Build and test** your system step-by-step.

This method ensures you understand:
- ROS package structure
- ROS nodes, topics, services, and TF
- Arduino-to-ROS communication
- Odometry calculation principles
- RViz visualization setup

---

## 🤖 Arduino Encoder Simulation

### ✨ Overview
A sample Arduino project to simulate wheel encoders using push buttons and publish tick counts over ROS.

### ⚙ Hardware Setup
| Purpose | Arduino Pin |
|:---|:---|
| Left Encoder A | 2 |
| Right Encoder A | 3 |
| Left Encoder B | 4 |
| Right Encoder B | 5 |

- No external resistors needed (uses `INPUT_PULLUP`).
- Button press simulates encoder pulses.

### 📡 ROS Topics and Services
- `/left_ticks` and `/right_ticks` (tick counts)
- `/set_to_near_max` (service to simulate rollover behavior)

> Students must create their own version of this Arduino sketch to work with their robot or simulation environment.

---

## 🚀 ROS Node Examples

### `rviz_click_to_2d`
- Converts 3D RViz pose goals into simple 2D goal topics.
- Publishes `/goal_2d` and `/initial_2d`.

### `ekf_odom_pub`
- Computes robot odometry based on simulated encoder ticks.
- Publishes `/odom_data_euler`, `/odom_data_quat`.
- Broadcasts TF transform (`odom ➔ base_link`).

---

## 🧭 Example Topics and TF

| Topic | Type | Description |
|:---|:---|:---|
| `/left_ticks`, `/right_ticks` | `std_msgs/Int16` | Encoder ticks |
| `/goal_2d`, `/initial_2d` | `geometry_msgs/PoseStamped` | Goals from RViz clicks |
| `/odom_data_euler`, `/odom_data_quat` | `nav_msgs/Odometry` | Odometry messages |
| `/tf` | TF | Odometry to robot base link |

---

## 🧠 Important Reminders for Students

- **Do not clone this repository and expect it to run as-is.**
- **Create your own ROS packages** following the examples given here.
- **Build your workspace and packages properly** using `catkin_make`.
- **Upload and test** Arduino code independently before connecting to ROS.
- **Always source your ROS environment**:

```bash
source ~/catkin_ws/devel/setup.bash

