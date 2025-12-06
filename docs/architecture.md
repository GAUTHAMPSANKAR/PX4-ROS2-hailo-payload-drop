# System Architecture – AeroHades25 Autonomy

## 1. Overview

This project implements a vision-assisted autonomous payload drop system
for PX4, running on a Raspberry Pi 5 with a Hailo-8L accelerator.

PX4 executes a pre-planned mission (AUTO.MISSION). The companion computer
monitors a downward-facing camera, detects a white cylindrical target on
the ground, and when the target is detected, performs:

1. Mission interruption / mode change.
2. Alignment and stabilization over the target using vision.
3. Descent to a configured drop altitude.
4. Payload release.
5. Post-drop behavior:
   - Return-to-Launch (RTL) (default), or
   - Resume mission (optional, via parameter).

The mission goal is **payload delivery over the target**, not landing.
Early prototypes used landing-like behavior only as a testing mechanism.

## 2. Nodes and Responsibilities

### `hades_lander.vision_node`

- Interfaces with the Hailo-based GStreamer pipeline.
- Processes detection outputs from the YOLOv8s `.hef` model.
- Filters detections for the white cylindrical target.
- Publishes target information (e.g. pixel position, confidence).

### `hades_lander.bullseye_lander_node`

- Connects to PX4 through MAVROS.
- Subscribes to:
  - `/mavros/state`
  - `/mavros/local_position/odom`
  - camera topics used for debugging / visualization.
- Publishes:
  - `/mavros/setpoint_velocity/cmd_vel_unstamped` (OFFBOARD velocity control).
- Manages:
  - Arming and initial mode setup.
  - Mission interruption when target is detected.
  - Alignment and stabilization over the target.
  - Controlled descent to drop altitude.
  - Payload drop trigger.
  - Post-drop behavior (RTL or mission resume).

Internally, it uses phase-oriented logic (pseudo-FSM) with states such as:
mission flight, target acquired, stabilizing, descending, payload drop, and
recovery.

## 3. PX4 Modes and Key Topics

- Modes:
  - `AUTO.MISSION`
  - `AUTO.LOITER`
  - `OFFBOARD`
  - `AUTO.RTL`

- MAVROS topics:
  - `/mavros/state`
  - `/mavros/local_position/odom`
  - `/mavros/setpoint_velocity/cmd_vel_unstamped`

## 4. HIL Simulation (Concept)

**Laptop:**
- PX4 v1.17 SITL
- Gazebo Harmonic with a downward camera
- MAVROS

**Raspberry Pi 5:**
- ROS 2 Jazzy
- `hades_lander` package
- Hailo-8L running a YOLOv8s `.hef` model
- Publishes velocity setpoints back to PX4 via MAVROS.

This setup allows testing alignment, descent, and post-drop behavior without
flying the real drone.

## 5. To Be Expanded

This document can later be expanded with:
- State machine diagrams.
- Topic graphs.
- Example parameter values.
- Simulation and field-test screenshots / GIFs (in `docs/images/`).
