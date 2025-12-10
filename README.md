# PX4 · ROS2 · Hailo — Vision-Assisted Payload Drop

**Vision-assisted autonomous payload delivery for PX4.**  
PX4 executes a pre-planned search mission while a Raspberry Pi 5 (with a Hailo-8L accelerator) runs a YOLOv8s-based perception pipeline. When the required target is detected the stack interrupts the mission, aligns the vehicle using OFFBOARD velocity control, descends to a configured drop altitude, triggers the payload release, and then returns or resumes the mission.

<p align="center">
  <img src="docs/images/hil_demo.gif" alt="Simulation demo" width="800">
</p>

**Key highlights**
- PX4 integration via MAVROS; mission interruption → OFFBOARD → RTL/resume.
- Hailo-8L accelerated YOLOv8s inference for on-device detection.
- Robust detection filtering (min-detections + EWMA smoothing) to avoid jitter.
- OFFBOARD velocity control mapping (pixel error → velocity) with safety fallbacks.
- Hardware-in-the-loop (HIL) tested: laptop (PX4 SITL + Gazebo) ↔ RPi5 (inference + autonomy).
# AeroHades25-Autonomy

ROS 2 Jazzy package implementing a **vision-assisted autonomous payload-drop
system** for Aerothon 2025 by Team AeroHades (CUSAT).

PX4 executes a pre-planned AUTO.MISSION. A Raspberry Pi 5 running a Hailo-8L
accelerator detects a **white cylindrical target** using a custom YOLOv8s model.
When detected, the autonomy stack:

1. Interrupts the mission.
2. Aligns above the target using vision-based velocity control (OFFBOARD).
3. Descends to a configured drop altitude.
4. Releases payload.
5. Performs **RTL** (default) or **mission resume** (parameter controlled).

> Note: Early prototypes used landing behavior *purely for testing.*  
> The final system performs **hover + descend + payload drop**, not landing.

---

## 📦 Repository Overview (Minimal Version)

- **\`hades_lander/\`** — final ROS 2 package (vision node + autonomy node)  
- **\`config/\`** — MAVROS configuration + future parameters  
- **\`launch/\`** — launch file for running the nodes  
- **\`archive/\`** — early monolithic prototypes (v0–v3); kept for history  
- **\`docs/\`** — system architecture + images/GIFs (expanded later)  
- **\`models/\`** — notes about `.hef` model usage (no model files included)  
- **\`test/\`** — optional style/tests from ROS package template  

---

## 🚀 Features

- Hailo-accelerated YOLOv8s target detection  
- OFFBOARD velocity control for alignment + descent  
- Mission interruption logic (AUTO.MISSION → OFFBOARD → RTL/Resume)  
- Supports Hardware-in-the-Loop (HIL) testing  
- Cleanly documented evolution (archive v0 → v3 → final ROS2 architecture)  

---

## 🔧 Runtime Requirements (Tested Configuration)

- **Ubuntu 24.04 (RPi5)**  
- **ROS 2 Jazzy**  
- **Python 3.12**  
- **PX4 v1.17** (SITL + real flight controller)  
- **Gazebo Harmonic**  
- **QGroundControl**

### Hailo Runtime (RPi5)

Tested with:

- **HailoRT 4.22.0**  
- **TAPPAS Core 5.0.0**  
- GStreamer plugins: `hailonet`, `hailofilter`, `hailotracker`, …  
- Python binding: `hailo-tappas-core-python-binding 5.0.0`  
- Dev utilities: `hailo-apps 25.7.0`

➡️ *This repo does not include `.hef` files or installation instructions.*  
See `models/README.md` for how to use your own model.

---

## 🔄 High-Level Behavior

1. PX4 flies GPS-based search mission  
2. YOLOv8s (Hailo) detects target via downward camera  
3. Companion switches PX4 to OFFBOARD  
4. Drone aligns over target  
5. Drone descends to drop altitude  
6. Payload released  
7. RTL or mission resume  

---

## 🖥️ HIL Simulation (Laptop ↔ RPi5)

**Laptop**  
- PX4 SITL 1.17  
- Gazebo Harmonic  
- MAVROS  
- Camera plugin publishes `/camera/image_raw`

**RPi5**  
- ROS 2 Jazzy  
- `hades_lander` package  
- Hailo inference running YOLOv8s `.hef`  
- Publishes OFFBOARD velocities to PX4  

This enables full autonomy-loop testing with real inference timing.

---

## ⚠️ Safety

Autonomous drone operation is hazardous.

Use only:
- in controlled environments  
- with RC override  
- with proper PX4 failsafes  
- in compliance with UAV laws  

This software is for **research/education** only.

---

## 👥 Credits

- **Team AeroHades (CUSAT)** – Aerothon 2025  
- **Autonomy software, HIL setup, integration:**  
  Gautham P Sankar  
- Other members contributed to model training & hardware integration.

---

## 📄 License

MIT License — see `LICENSE`.

