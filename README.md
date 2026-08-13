# Bio-Inspired Falcon Wing Actuation Mechanism

Parametric CAD models, computer vision tracking pipelines, and kinematic geometry for a two-degree-of-freedom (2-DoF) morphing wing mechanism derived from avian flight telemetry.

---

## Overview & Background

Replicating biological avian flight requires translating non-linear joint movement into rigid mechanical linkages. Standard multi-bar linkages often fail to balance out-of-plane torsional stability with low rotational inertia during dynamic flapping cycles.

This project extracts high-speed trajectory coordinates from peregrine falcon flight sequences to synthesize a parametric wing mechanism. By combining computer vision telemetry processing with CadQuery programmatic CAD generation, the platform automates the pipeline from raw biological motion data to 3D-printable manufacturing files.

---

## System Architecture

```text
[ High-Speed Avian Video Telemetry ]
                 │
                 ▼
[ Video Tracking & Noise Suppression ]
   ├── Frame-by-Frame Joint Extraction (falcon_manual_track.py)
   ├── Angle-Wrap Resolution & Interpolation (fix_angle_wrap.py)
   └── Moving-Window Trajectory Smoothing (smooth_and_summarize.py)
                 │
                 ▼
[ Kinematic Synthesis & Optimization ]
   ├── Angular Bounds & Velocity Profiling (falcon_kinematics.py)
   └── Parameter Extraction Engine (extract_design_params.py)
                 │
                 ▼
[ Programmatic CAD Generation ]
   ├── Parametric Linkage Geometry (build_wing.py)
   ├── Interlocking Clevis Hinge & Lightening Pockets
   └── STEP Model Export (hardware/)
```

---

## Mechanical & Kinematic Parameters

All structural dimensions are parameterized in `build_wing.py` via the `WingParameters` class to allow immediate rescaling based on extracted flight data:

| Parameter | Symbol | Default Value | Function & Design Constraint |
|---|---|---|---|
| Humerus Length | `L_h` | 100.0 mm | Distance from shoulder pivot to elbow joint |
| Forearm Length | `L_f` | 125.0 mm | Distance from elbow joint to wingtip pivot |
| Pin Clearance Diameter | `d_pin` | 3.2 mm | Sized for M3 pivot fasteners |
| Interface Clearance | `delta` | 0.25 mm | Tolerance gap for smooth hinge articulation |
| Clevis Hinge Width | `W_c` | 10.0 mm | External fork width resisting out-of-plane torsion |
| Forearm Tongue Thickness | `t_f` | 4.0 mm | Internal male tongue mating dimension |

---

## Applied Kinematics & Analytical Models

### 1. Vector Linkage Positioning

The 2D spatial trajectory of the elbow `(Xe, Ye)` and wingtip `(Xt, Yt)` joints relative to the shoulder origin `(0,0)` is computed using transformation matrices based on humerus extension angle `theta_h(t)` and forearm flex angle `theta_f(t)`:

```
Xe(t) = L_h * cos(theta_h(t))
Ye(t) = L_h * sin(theta_h(t))
Xt(t) = Xe(t) + L_f * cos(theta_h(t) + theta_f(t))
Yt(t) = Ye(t) + L_f * sin(theta_h(t) + theta_f(t))
```

### 2. Rotational Inertia Optimization

To minimize actuator torque requirements during high-frequency flapping, link cross-sections taper along their longitudinal axis. Removing non-structural mass near the wingtip reduces total rotational inertia (`I = ∫ r² dm`) while preserving flexural rigidity under aerodynamic lift forces.

---

## Engineering Challenges & Solutions

### Coordinate Wrapping in Video Telemetry
- **Problem:** High-speed angular tracking across full flap cycles caused phase discontinuities when joint angles crossed the boundary.
- **Solution:** Developed an unwrapping algorithm (`fix_angle_wrap.py`) that detects derivative spikes and converts raw phase steps into continuous angular trajectories.

### Out-of-Plane Torsion at the Elbow
- **Problem:** Single-lap shear joints failed under lateral aerodynamic drag during downstroke simulations.
- **Solution:** Replaced simple pivot joints with a dual-shear interlocking clevis structure on the humerus link, constraining movement strictly to the primary plane of actuation.

### Manufacturing Tolerance Stack-up
- **Problem:** Standard FDM 3D printing causes hole shrinkage and surface friction on tight-fitting pivot pins.
- **Solution:** Programmed dynamic tolerance offsets (`delta = 0.25 mm`) into the CadQuery generation script to ensure smooth rotation without post-machining.

---

## Electronics & Control Architecture

Planned hardware to support V2 actuator integration (see Roadmap):

### Actuation
- **Micro servos** (e.g., digital metal-gear servos such as Hitec HS-5065MG) driving the humerus and forearm joints via servo-horn/pushrod linkages. Prioritizes torque-to-weight over raw speed, since flap-cycle torque peaks at direction reversal.
- **Alternative:** BLDC gimbal motors with closed-loop control for higher bandwidth than a hobby servo's internal PWM loop, at the cost of a custom driver + encoder.

### Position / Motion Sensing
- **Magnetic rotary encoders (AS5048A / AS5600)** at each joint pivot — mount around the shaft with a diametrically magnetized magnet, compatible with the clevis hinge geometry, avoids potentiometer wear.
- **IMU (e.g., ICM-42688 or BNO085)** on the wingtip or forearm link to validate CV-derived kinematic trajectories against real angular velocity/acceleration during bench testing.

### Control / Compute
- **Microcontroller:** STM32F4 or Teensy 4.x for sufficient PWM channels and headroom to run dual servo-position control loops plus encoder feedback in real time.
- **ESP32** as a rapid-prototyping alternative — PWM, ADC for encoder feedback, and Wi-Fi/BLE for wireless telemetry logging during bench tests.

### Motor Drivers
- **BLDC route:** small FOC driver per joint (e.g., SimpleFOC-compatible boards using DRV8313).
- **Servo route:** no separate driver required — ensure clean, adequately-fused power distribution, as servos under load can pull current spikes.

### Power
- **2S–3S LiPo** (7.4–11.1V), stepped down to 5–6V for the servo rail via a BEC or buck converter, with the MCU on a separate regulated 3.3V rail to isolate servo electrical noise from sensor lines.

### Data / Telemetry Link
- **UART-to-USB or BLE** streaming encoder and IMU data to the Python analysis pipeline for live logging during dual-motor bench testing.

---

## Project Execution & Setup

### Prerequisites
- Python 3.9+
- CadQuery 2.1+

```bash
pip install cadquery numpy pandas matplotlib opencv-python
```

### 1. Run Trajectory Analysis & Pipeline

Process raw biological tracking data and plot joint kinematics:

```bash
python3 falcon_kinematics.py
```

### 2. Generate 3D Parametric CAD Files

Build the assembly geometry and export STEP files:

```bash
python3 build_wing.py
```

Generated CAD files will be exported to the `hardware/` directory:

- `hardware/humerus_clevis_link.step`
- `hardware/forearm_tapered_link.step`

---

## Development Roadmap

- **V2 Actuator Integration:** Design internal servo horns and pushrod linkage mountings for direct micro-servo actuation.
- **FEA Structural Analysis:** Perform stress distribution modeling under peak downstroke aerodynamic load conditions.
- **Physical Testing Platform:** Construct a dual-motor bench setup to test dynamic flap cycles against telemetry predictions.
