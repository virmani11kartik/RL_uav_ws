# RL-Drone  
**Sim-to-Real Reinforcement Learning Control Stack for Quadrotors**

End-to-end system to deploy **PyTorch RL policies (`.pt`)** on real quadrotors using  
**ROS 2 + Betaflight MSP + ESP32 (CRSF / RC override)**.

This project focuses on **real-world deployment constraints**: deterministic control paths, explicit authority handoff, hardware-enforced safety, and recoverable failure modes. It is designed for operation on real flight hardware, not as a simulator-only demo.

---

## Project Summary

This repository implements a complete **sim-to-real autonomy stack** for RL-controlled drones:

- RL policy inference on an onboard SBC  
- ROS 2 as the control, safety, and arbitration layer  
- Betaflight MSP for low-latency flight-controller command and telemetry  
- ESP32-based RC and AUX enforcement with hardware-visible state feedback  
- Explicit arming, disarming, and timeout-based failsafes  

The system is designed to ensure:

- Control loops never block on perception or inference  
- Loss of software authority always reverts to safe RC behavior  
- Flight-controller firmware remains unmodified  
- All state transitions are externally observable  

---

## Demo Videos

### Full System Demo — Sim to Real Control Path
RL policy inference → ROS 2 arbitration → MSP → Betaflight → FC actuation  

<video controls src="https://github.com/user-attachments/assets/sim.MP4" title="Full System Demo"></video>

### Hardware State Feedback
Visual feedback for armed, idle, and fallback states  

<video controls src="https://github.com/user-attachments/assets/IMG_0236.MOV" title="Hardware State Feedback"></video>

### Hover Test
Stability test under autonomous control  

<video controls src="https://github.com/user-attachments/assets/IMG_0456.MOV" title="Hover Test"></video>

---

## System Architecture

### High-Level Control Stack
![System Architecture](assets/pic1.jpeg)

### Mechanical Design
![CAD Overview](assets/cad.PNG)

Deadcat-style frame selected to preserve a clean forward camera field of view while maintaining sufficient control authority for autonomous flight.

---

## Core Design Principles

### Control Authority Model
- ROS 2 owns high-level control authority  
- ESP32 enforces RC-side safety and arming semantics  
- Betaflight remains unmodified and safety-compliant  
- A physical RC kill always overrides software  

### Why MSP + RC Hybrid
- MSP provides structured, low-latency access to the flight controller  
- RC channels remain the universally trusted safety interface  
- AUX channels enable explicit arming and mode transitions  
- No firmware modification required, improving field safety and maintainability  

---

## Repository Structure

```bash
RL_uav_ws/
├── src/
│   ├── uav_msp_bridge/      # ROS 2 ↔ Betaflight MSP interface
│   ├── uav_rl_agent/        # PyTorch policy inference node
│   ├── uav_control_cpp/     # Safety, gating, timeout & fallback logic
│   └── uav_msgs/            # (optional) custom messages
│
├── firmware/
│   └── esp_rc_bridge/       # ESP32 CRSF / RC + NeoPixel firmware
│
├── configs/
│   ├── betaflight/          # CLI dumps, ports, AUX configs
│   └── params/              # ROS 2 YAML parameter files
│
├── docs/
│   ├── media/               # Images & videos
│   ├── wiring/              # Pinouts & schematics
│   └── logs/                # Example logs & captures
│
└── README.md
```

---

## Hardware Stack

### Flight System

- Betaflight-compatible flight controller
- ELRS (CRSF) receiver
- ESP32 / ESP32-C3 microcontroller
- NeoPixel single-pixel status LED

### Compute

- SBC (Jetson / Radxa / Raspberry Pi class)
- ROS 2 (Humble / Jazzy)
- PyTorch inference runtime

---

## ROS 2 Interface

### Primary Control Topic

```
/rc_aetr_aux   (std_msgs/UInt16MultiArray)
```

Channel ordering (example):

```
[A, E, T, R, AUX1, AUX2, AUX3, AUX4]
```

- **AETR**: control inputs
- **AUX1**: ARM / DISARM
- **Remaining AUX**: modes, failsafe states, experiment flags

ROS publishes fixed-rate, bounded packets. Timeouts are handled explicitly; stale commands are never reused.

---

## Arming and Safety Logic

### Supported Modes

#### Mode A — Disarm on Stop (recommended)

- RL node publishes commands continuously
- On timeout, crash, or node exit:
  - AUX1 set to DISARM
  - AETR reset to neutral
  - Control authority returned fully to RC

#### Mode B — Stay Armed on Stop (bench testing only)

- AUX1 remains armed
- AETR and AUX2+ reset to defaults

**Always test with props removed.**

---

## ESP32 Firmware Behavior

### NeoPixel State Encoding

| State          | Color |
|----------------|-------|
| Boot / Idle    | Blue  |
| Armed          | Green |
| Timeout / Error| Red   |

The NeoPixel provides hardware-level observability of system state, independent of ROS logs or SBC health.

### ESP32 Notes

- ESP32-C3 boards often share USB and UART pins
- Excessive serial printing can delay RC updates and LED state changes
- Recommended practices:
  - Limit print rate
  - Separate debug UART from RC UART
  - Avoid USB-conflicting pins

---

## Build and Run

### Build ROS Workspace

```bash
cd RL_uav_ws
colcon build --symlink-install
source install/setup.bash
```

### Flash ESP32 Firmware

Firmware is located in:

```
firmware/esp_rc_bridge/
```

Expected boot behavior:

- NeoPixel blink
- Serial output: `READY`

### Launch Control Stack

```bash
ros2 launch uav_msp_bridge bringup.launch.py
```

### Manual RC Test

```bash
ros2 topic pub --rate 100 /rc_aetr_aux std_msgs/msg/UInt16MultiArray \
"{data: [1500, 1500, 1100, 1500, 1100, 900, 900, 900]}"
```

---

## Known Issues and Debugging

### LED updates delayed
**Cause**: UART contention or excessive serial printing  
**Fix**: Reduce print rate or reassign UART pins

### Armed state never matches default logic
Do not include AUX1 in default-state checks; AUX1 must be treated independently

### Commands persist after node stops
Ensure timeout logic explicitly resets:

- AETR
- AUX2+
- AUX1 (depending on safety mode)

---

## Roadmap

- Unified safety supervisor node
- Standardized RC command message
- Launch profiles for bench, props-off, and flight testing
- rosbag and logging workflows
- Wiring diagrams per FC and ESP variant
- Hardware-in-loop latency benchmarks

---

## Safety Disclaimer

⚠️ **This project interfaces with real flight hardware.**

- Test with props removed
- Use a physical kill switch
- Start with conservative throttle limits
- Never rely on software alone for safety

---

## License

To be decided .

---

## Credits

Designed and implemented as part of an ongoing sim-to-real reinforcement learning drone autonomy research project.
