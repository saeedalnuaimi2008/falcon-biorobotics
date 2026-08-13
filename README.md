# Bio-Inspired Falcon Wing Actuation Mechanism

Parametric 3D CAD models and kinematic geometry for a two-degree-of-freedom (2-DoF) morphing wing mechanism derived from avian telemetry.

## System Overview

This repository contains programmatic CAD scripts generated using [CadQuery](https://cadquery.readthedocs.io/) to model a lightweight, articulated wing structure.

### Key Features
* **Interlocking Clevis Hinge:** Female clevis fork on the humerus link mated to a male tongue on the forearm link to resist out-of-plane torsional aerodynamic forces.
* **Integrated Actuator Horn:** Rigid moment arm on the humerus link designed for servo pushrod attachment.
* **Mass Optimization:** Tapered link profiles and internal lightening pockets to minimize rotational inertia along the wing span.
* **Parametric Hardware Integration:** Sized for standard M3 pivot fasteners with configurable manufacturing clearance gaps.

---

## Kinematic & Structural Parameters

All geometric parameters can be configured directly in `build_wing.py` via the `WingParameters` data class:

| Parameter | Symbol | Default Value | Description |
| :--- | :--- | :--- | :--- |
| **Humerus Length** | $L_h$ | $100.0\text{ mm}$ | Distance from shoulder pivot to elbow pivot |
| **Forearm Length** | $L_f$ | $125.0\text{ mm}$ | Distance from elbow pivot to wingtip pivot |
| **Pin Diameter** | $d_{\text{pin}}$ | $3.2\text{ mm}$ | M3 fastener clearance hole diameter |
| **Print Clearance** | $\delta$ | $0.25\text{ mm}$ | Joint interface clearance for smooth articulation |
| **Clevis Width** | $W_c$ | $10.0\text{ mm}$ | Total humerus joint thickness at the elbow |
| **Forearm Thickness** | $t_f$ | $4.0\text{ mm}$ | Male tongue thickness |

---

## Getting Started

### Prerequisites

* Python 3.8+
* CadQuery

```bash
pip install cadquery
