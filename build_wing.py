"""
Falcon BioRobotics - Kinematic Wing Mechanism CAD Generator
-----------------------------------------------------------
Parametric CAD generation script for a bio-inspired falcon wing linkage.
Defines geometry, interlocking clevis joint, and servo moment arm.
"""

from dataclasses import dataclass
import cadquery as cq


@dataclass
class WingParameters:
    """Design parameters (all linear units in mm)."""
    # Linkage Lengths
    l_humerus: float = 100.0       # Upper wing arm length
    l_forearm: float = 125.0       # Lower wing arm length
    
    # Fastener & Manufacturing Tolerances (M3 Hardware)
    pin_dia: float = 3.2           # Clearance hole diameter for M3 pins
    print_gap: float = 0.25        # Radial clearance for printed joint movement
    
    # Structural Profile Dimensions
    clevis_width: float = 10.0     # Total thickness of humerus clevis fork
    forearm_thick: float = 4.0     # Thickness of forearm male tongue
    r_shoulder: float = 9.0        # Shoulder joint radius
    r_elbow: float = 7.0           # Elbow joint radius
    r_wingtip: float = 4.5         # Wingtip radius
    
    # Actuator Servo Horn Attachment
    horn_dist: float = 22.0        # Distance along humerus for pushrod linkage
    horn_offset: float = 15.0      # Moment arm height from beam center


def create_humerus(cfg: WingParameters) -> cq.Workplane:
    """Generates the humerus arm featuring an integrated female clevis and servo horn."""
    # 1. Main Tapered Profile
    taper_pts = [
        (0.0, -cfg.r_shoulder),
        (cfg.l_humerus, -cfg.r_elbow),
        (cfg.l_humerus, cfg.r_elbow),
        (0.0, cfg.r_shoulder)
    ]
    beam = cq.Workplane("XY").polyline(taper_pts).close().extrude(cfg.clevis_width)
    
    # 2. Joint End Bosses
    shoulder_boss = cq.Workplane("XY").circle(cfg.r_shoulder).extrude(cfg.clevis_width)
    elbow_boss = cq.Workplane("XY").center(cfg.l_humerus, 0).circle(cfg.r_elbow).extrude(cfg.clevis_width)
    
    # 3. Servo Horn Extension
    horn_pts = [
        (cfg.horn_dist - 7.0, 0.0),
        (cfg.horn_dist, cfg.horn_offset),
        (cfg.horn_dist + 7.0, 0.0)
    ]
    servo_horn = cq.Workplane("XY").polyline(horn_pts).close().extrude(cfg.forearm_thick)
    
    # Combine geometry
    humerus = beam.union(shoulder_boss).union(elbow_boss).union(servo_horn)
    
    # 4. Female Clevis Slot Cutout
    slot_gap = cfg.forearm_thick + (2 * cfg.print_gap)
    z_offset = (cfg.clevis_width - slot_gap) / 2.0
    clevis_slot = (
        cq.Workplane("XY")
        .workplane(offset=z_offset)
        .center(cfg.l_humerus, 0)
        .rect(cfg.r_elbow * 2 + 2.0, cfg.r_elbow * 2 + 2.0)
        .extrude(slot_gap)
    )
    humerus = humerus.cut(clevis_slot)
    
    # 5. Drill Hardware Pin Holes
    humerus = (
        humerus.faces(">Z")
        .workplane()
        .pushPoints([
            (0.0, 0.0),
            (cfg.l_humerus, 0.0),
            (cfg.horn_dist, cfg.horn_offset)
        ])
        .hole(cfg.pin_dia)
    )
    
    return humerus


def create_forearm(cfg: WingParameters) -> cq.Workplane:
    """Generates the forearm link with male tongue and internal weight-reduction pocket."""
    # 1. Main Tapered Profile
    profile_pts = [
        (0.0, -cfg.r_elbow),
        (cfg.l_forearm, -cfg.r_wingtip),
        (cfg.l_forearm, cfg.r_wingtip),
        (0.0, cfg.r_elbow)
    ]
    beam = cq.Workplane("XY").polyline(profile_pts).close().extrude(cfg.forearm_thick)
    
    # 2. Bosses
    elbow_boss = cq.Workplane("XY").circle(cfg.r_elbow).extrude(cfg.forearm_thick)
    wingtip_boss = cq.Workplane("XY").center(cfg.l_forearm, 0).circle(cfg.r_wingtip).extrude(cfg.forearm_thick)
    
    forearm = beam.union(elbow_boss).union(wingtip_boss)
    
    # 3. Lightening Pocket Cutout
    pocket_pts = [
        (15.0, -2.5),
        (cfg.l_forearm - 12.0, -1.2),
        (cfg.l_forearm - 12.0, 1.2),
        (15.0, 2.5)
    ]
    pocket = cq.Workplane("XY").polyline(pocket_pts).close().extrude(cfg.forearm_thick)
    forearm = forearm.cut(pocket)
    
    # 4. Pin Holes
    forearm = (
        forearm.faces(">Z")
        .workplane()
        .pushPoints([(0.0, 0.0), (cfg.l_forearm, 0.0)])
        .hole(cfg.pin_dia)
    )
    
    return forearm


if __name__ == "__main__":
    params = WingParameters()
    
    humerus_solid = create_humerus(params)
    forearm_solid = create_forearm(params)
    
    # Export universal STEP CAD models
    cq.exporters.export(humerus_solid, "humerus_clevis_link.step")
    cq.exporters.export(forearm_solid, "forearm_tapered_link.step")
    
    print("Exported humerus_clevis_link.step and forearm_tapered_link.step successfully.")
