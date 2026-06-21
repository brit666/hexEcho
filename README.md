# HexEcho 

**Autonomous 18-DOF Hexapod Robot with Ultrasonic Navigation**

HexEcho is a fully autonomous six-legged walking robot built from scratch — custom CAD, hand-wired electronics, and original firmware. It uses a wave gait for maximum stability and an HC-SR04 ultrasonic sensor for obstacle detection and avoidance.

Built as a course project and designed as the physical platform for future reinforcement learning research.

---

## Quick Overview

| Property | Value |
|----------|-------|
| DOF | 18 (3 per leg × 6 legs) |
| Gait | Wave gait (1 leg at a time, 5 always grounded) |
| Controller | Arduino Nano |
| Servo Driver | 2× PCA9685 (I²C, 0x40 / 0x41) |
| Sensor | HC-SR04 ultrasonic |
| Power | 2S LiPo 7.4V 1000mAh |
| Chassis | Custom PETG, 3D printed |
| Weight | ~600g |

---

## Repository Structure

```
HexEcho/
├── firmware/
│   └── HexEcho_firmware.ino       # Main Arduino sketch
├── CAD/
│   ├── source/                    # Fusion 360 .f3d files
│   └── stl/                       # Export STLs for printing
│       ├── body_cylinder.stl
│       ├── cover_plate.stl
│       ├── body_hook.stl
│       ├── coxa_bracket.stl
│       ├── femur.stl
│       └── tibia.stl
│       └── print_details.txt
├── schematics/
│   └── HexEcho_schematic.png      # EasyEDA circuit diagram
├── docs/
│   └── HexEcho_Build_Manual.pdf  # Complete build manual (start here)
├── README.md
└── LICENSE
```

---

## Getting Started

Read **[`docs/HexEcho_Build_Manual.pdf`](docs/HexEcho_Build_Manual.pdf)** — it covers the full bill of materials, mechanical specs, wiring, firmware architecture, assembly steps, and calibration in detail.

**Quick steps:**
1. Print all STL files per the print settings in the manual
2. Source components from the BOM
3. Wire per the schematic
4. Flash `HexEcho_firmware.ino` to the Arduino Nano
5. Calibrate home angles and PWM range, then tune gait parameters
6. Deploy

---

## Dependencies

Install via Arduino IDE Library Manager before flashing:

- [Adafruit PWMServoDriver](https://github.com/adafruit/Adafruit-PWMServoDriver-Library)
- `Wire.h` (built-in)

---

## Roadmap

- **v1.0** — Wave gait + ultrasonic obstacle avoidance ✅ *(current)*
- **v2.0** — Reinforcement learning locomotion via Unreal Engine sim-to-real
- **v3.0** — Onboard camera + visual navigation
- **v4.0** — Adaptive terrain locomotion

---

## License

MIT License — see [LICENSE](LICENSE).


<img width="1520" height="810" alt="Hexapod Assembly pic trans" src="https://github.com/user-attachments/assets/a57276a2-8383-47d9-bd14-769530c11343" />
<img width="2048" height="1536" alt="image" src="https://github.com/user-attachments/assets/c4744dc2-3100-469d-9971-2bc851a65234" />

