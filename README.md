# RL-Drone  
**Sim-to-Real Reinforcement Learning Stack for Racing Quadrotors**

End-to-end pipeline to deploy **PyTorch RL policies (.pt)** on real racing drones using  
**ROS 2 + Betaflight MSP + ESP32 (CRSF / RC override)**.

This project bridges **simulation-trained policies (Isaac / IsaacLab / Gym)** to real quadrotors with
explicit safety gating, arming logic, and robust fallback behavior.

---

## 🔥 Project Overview

This repository implements a **full sim-to-real control stack** for RL-based drone racing:

- RL policy inference on an onboard SBC
- ROS 2 as the control, safety, and orchestration layer
- Betaflight MSP for low-latency FC command & telemetry
- ESP32-based RC / AUX handling with visual state feedback
- Explicit arming, disarming, and timeout-based failsafes

The design prioritizes:
- **Deterministic control paths**
- **Clear arming semantics**
- **Recoverable failure modes**
- **Debuggability at every layer**

---

## 🎥 Demo Videos

### Full System Demo (Sim → Real Control Path)
<video src="RL_uav_ws\assets\sim.MP4" controls width="720"></video>

> RL policy inference → ROS 2 → MSP → Betaflight → FC response

---

<video src="RL_uav_ws\assets\IMG_0236.MOV" controls width="720"></video>

> Visual feedback for armed / default / fallback states

---

## 🖼️ System Architecture

### High-Level Control Stack
![System Architecture](RL_uav_ws\assets\pic1.JPG)

---

### Hardware Wiring Overview
![Wiring Diagram](docs/media/wiring_diagram.png)

---

## 🧠 Core Concepts

### Control Philosophy
- **ROS 2 owns authority**
- **ESP32 enforces RC-side safety**
- **Betaflight remains unmodified**
- **No FC firmware hacks required**

### Why MSP + RC Hybrid?
- MSP gives structured access to the FC
- RC channels remain the universal safety interface
- AUX channels provide deterministic arming & mode control
- Hardware kill remains valid at all times

---

## 📦 Repository Structure

```bash
RL_uav_ws/
├── src/
│ ├── uav_msp_bridge/ # ROS2 ↔ Betaflight MSP bridge
│ ├── uav_rl_agent/ # PyTorch policy inference node
│ ├── uav_control_cpp/ # Safety, gating, fallback logic
│ └── uav_msgs/ # (optional) custom messages
│
├── firmware/
│ └── esp_rc_bridge/ # ESP32 CRSF / RC + NeoPixel firmware
│
├── configs/
│ ├── betaflight/ # CLI dumps, ports, AUX configs
│ └── params/ # ROS2 YAML parameter files
│
├── docs/
│ ├── media/ # Images & videos (used in README)
│ ├── wiring/ # Pinouts & schematics
│ └── logs/ # Example logs & captures
│
└── README.md
```

---

## 🧩 Hardware Stack

### Flight System
- Betaflight-compatible flight controller
- ELRS (CRSF) receiver
- ESP32 / ESP32-C3 microcontroller
- NeoPixel (single-pixel status LED)

### Compute
- SBC (Jetson / Radxa / Raspberry Pi class)
- ROS 2 (Humble / Jazzy)
- PyTorch inference runtime

---

## 🛰️ ROS 2 Interface

### Primary Control Topic
/rc_aetr_aux (std_msgs/UInt16MultiArray)


**Channel ordering (example):**
[A, E, T, R, AUX1, AUX2, AUX3, AUX4]


- `AETR` → control inputs
- `AUX1` → ARM / DISARM
- Remaining AUX → modes / failsafe / custom states

---

## 🔐 Arming & Safety Logic

### Supported Modes

#### Mode A — Disarm on Stop (recommended)
- RL node publishes commands
- On timeout or node exit:
  - AUX1 → DISARM
  - AETR → neutral

#### Mode B — Stay Armed on Stop (bench testing only)
- AUX1 remains armed
- AETR + AUX2+ reset to defaults

⚠️ **Always test with props off first**

---

## 🔌 ESP32 Firmware Behavior

### NeoPixel State Encoding (example)
| State            | Color |
|------------------|-------|
| Boot / Idle      | Blue  |
| Armed            | Green |
| Timeout / Error  | Red   |

> NeoPixel provides immediate hardware-level visibility into system state.

---

### ESP32 Notes (Important)
- ESP32-C3 boards often **share USB + UART pins**
- Excessive `Serial.print()` can:
  - delay RC updates
  - block LED state changes
- Recommended:
  - reduce print rate
  - separate debug UART from RC UART
  - avoid USB-conflicting pins

---

## 🛠️ Build & Run

### Build ROS Workspace
```bash
cd RL_uav_ws
colcon build --symlink-install
source install/setup.bash
Flash ESP32 Firmware
Located in:

firmware/esp_rc_bridge/
Expected boot output:

NeoPixel blink

Serial message: READY

Launch Control Stack
ros2 launch uav_msp_bridge bringup.launch.py
Manual RC Test
ros2 topic pub --rate 100 /rc_aetr_aux std_msgs/msg/UInt16MultiArray \
"{data: [1500, 1500, 1100, 1500, 1100, 900, 900, 900]}"
🧪 Known Issues & Debugging
LED updates delayed
Cause: UART contention or print flooding

Fix: lower print rate, move UART pins

Armed state never matches default logic
Do not include AUX1 in is_default checks

AUX1 must be treated independently

Commands persist after node stops
Ensure timeout logic explicitly resets:

AETR

AUX2+

AUX1 based on safety mode

🛣️ Roadmap
 Unified safety supervisor node

 Standardized RC command message

 Launch profiles: bench, props_off, flight

 rosbag + logging recipes

 Wiring diagrams per FC / ESP variant

 Hardware-in-loop latency benchmarks

⚠️ Safety Disclaimer
This project interfaces with real flight hardware.

Test with props removed

Use a physical kill switch

Start with conservative throttle limits

Never trust software alone for safety

📜 License
To be decided (MIT / BSD recommended).

✨ Credits
Designed and implemented as part of an ongoing
sim-to-real RL drone racing research project.


---

