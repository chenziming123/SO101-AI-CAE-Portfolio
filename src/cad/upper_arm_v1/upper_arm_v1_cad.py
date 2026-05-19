from __future__ import annotations

import argparse
import json
from pathlib import Path

from build123d import Box, Compound, Cylinder, Pos, export_step, export_stl


# Units: millimeters.
# Coordinate convention:
# - origin is the shoulder_lift joint center used as the inboard mating datum;
# - +X points from shoulder_lift toward elbow_flex;
# - +Y is the joint shaft / servo width direction;
# - +Z is the weak-axis bending direction used by the Step 7 screening model.

SPAN_MM = 116.0
SHOULDER_BOSS_RADIUS_MM = 18.0
ELBOW_BOSS_RADIUS_MM = 16.5
JOINT_BORE_RADIUS_MM = 4.2
BOSS_WIDTH_MM = 34.0

RAIL_LENGTH_MM = SPAN_MM - 18.0
RAIL_CENTER_X_MM = SPAN_MM / 2.0
RAIL_WIDTH_Y_MM = 15.0
RAIL_THICKNESS_Z_MM = 4.8
RAIL_OFFSET_Z_MM = 13.5

WEB_LENGTH_MM = SPAN_MM - 28.0
WEB_THICKNESS_Y_MM = 2.8
WEB_HEIGHT_Z_MM = 26.0

RIB_THICKNESS_X_MM = 5.0
RIB_WIDTH_Y_MM = 17.0
RIB_HEIGHT_Z_MM = 34.0
RIB_X_POSITIONS_MM = (26.0, 58.0, 90.0)

CABLE_CHANNEL_LENGTH_MM = SPAN_MM - 34.0
CABLE_CHANNEL_WIDTH_Y_MM = 8.0
CABLE_CHANNEL_WALL_MM = 1.8
CABLE_CHANNEL_Z_MM = RAIL_OFFSET_Z_MM + RAIL_THICKNESS_Z_MM / 2.0 + 1.2

LIGHTENING_HOLE_RADIUS_MM = 6.0
LIGHTENING_HOLE_X_POSITIONS_MM = (38.0, 58.0, 78.0)

SERVO_CLEARANCE_BLOCK_X_MM = 16.0
SERVO_CLEARANCE_BLOCK_Y_MM = 38.0
SERVO_CLEARANCE_BLOCK_Z_MM = 24.0


def y_cylinder(radius: float, length: float):
    return Cylinder(radius=radius, height=length, rotation=(90, 0, 0))


def bored_y_cylinder(radius: float, bore_radius: float, length: float):
    return y_cylinder(radius, length) - y_cylinder(bore_radius, length + 2.0)


def rail_pair():
    top = Pos(RAIL_CENTER_X_MM, 0, RAIL_OFFSET_Z_MM) * Box(
        RAIL_LENGTH_MM,
        RAIL_WIDTH_Y_MM,
        RAIL_THICKNESS_Z_MM,
    )
    bottom = Pos(RAIL_CENTER_X_MM, 0, -RAIL_OFFSET_Z_MM) * Box(
        RAIL_LENGTH_MM,
        RAIL_WIDTH_Y_MM,
        RAIL_THICKNESS_Z_MM,
    )
    return [top, bottom]


def lightened_center_web():
    web = Pos(RAIL_CENTER_X_MM, 0, 0) * Box(
        WEB_LENGTH_MM,
        WEB_THICKNESS_Y_MM,
        WEB_HEIGHT_Z_MM,
    )
    for x_pos in LIGHTENING_HOLE_X_POSITIONS_MM:
        web = web - (Pos(x_pos, 0, 0) * y_cylinder(LIGHTENING_HOLE_RADIUS_MM, WEB_THICKNESS_Y_MM + 4.0))
    return web


def vertical_ribs():
    ribs = []
    for x_pos in RIB_X_POSITIONS_MM:
        ribs.append(Pos(x_pos, 0, 0) * Box(RIB_THICKNESS_X_MM, RIB_WIDTH_Y_MM, RIB_HEIGHT_Z_MM))
    return ribs


def shoulder_interface():
    boss = Pos(0, 0, 0) * bored_y_cylinder(SHOULDER_BOSS_RADIUS_MM, JOINT_BORE_RADIUS_MM, BOSS_WIDTH_MM)
    servo_clearance = Pos(-6.0, 0, 0) * Box(
        SERVO_CLEARANCE_BLOCK_X_MM,
        SERVO_CLEARANCE_BLOCK_Y_MM,
        SERVO_CLEARANCE_BLOCK_Z_MM,
    )
    side_pad = Pos(8.0, 0, 0) * Box(14.0, 30.0, 28.0)
    return [boss, servo_clearance, side_pad]


def elbow_interface():
    boss = Pos(SPAN_MM, 0, 0) * bored_y_cylinder(ELBOW_BOSS_RADIUS_MM, JOINT_BORE_RADIUS_MM, BOSS_WIDTH_MM)
    servo_pad = Pos(SPAN_MM - 7.0, 0, 0) * Box(14.0, 31.0, 27.0)
    outer_stiffener = Pos(SPAN_MM - 15.0, 0, 0) * Box(10.0, 24.0, 34.0)
    return [boss, servo_pad, outer_stiffener]


def cable_channel():
    floor = Pos(RAIL_CENTER_X_MM, 0, CABLE_CHANNEL_Z_MM) * Box(
        CABLE_CHANNEL_LENGTH_MM,
        CABLE_CHANNEL_WIDTH_Y_MM,
        CABLE_CHANNEL_WALL_MM,
    )
    left_wall = Pos(RAIL_CENTER_X_MM, CABLE_CHANNEL_WIDTH_Y_MM / 2.0, CABLE_CHANNEL_Z_MM + 2.1) * Box(
        CABLE_CHANNEL_LENGTH_MM,
        CABLE_CHANNEL_WALL_MM,
        4.2,
    )
    right_wall = Pos(RAIL_CENTER_X_MM, -CABLE_CHANNEL_WIDTH_Y_MM / 2.0, CABLE_CHANNEL_Z_MM + 2.1) * Box(
        CABLE_CHANNEL_LENGTH_MM,
        CABLE_CHANNEL_WALL_MM,
        4.2,
    )
    return [floor, left_wall, right_wall]


def upper_arm_v1():
    children = []
    children.extend(rail_pair())
    children.append(lightened_center_web())
    children.extend(vertical_ribs())
    children.extend(shoulder_interface())
    children.extend(elbow_interface())
    children.extend(cable_channel())

    fused = children[0]
    for child in children[1:]:
        fused = fused + child
    fused.label = "so101_upper_arm_v1_ai_rebuild"
    return fused


def parameters() -> dict[str, float | list[float] | str]:
    return {
        "units": "mm",
        "span_shoulder_to_elbow_mm": SPAN_MM,
        "shoulder_boss_radius_mm": SHOULDER_BOSS_RADIUS_MM,
        "elbow_boss_radius_mm": ELBOW_BOSS_RADIUS_MM,
        "joint_bore_radius_mm": JOINT_BORE_RADIUS_MM,
        "boss_width_mm": BOSS_WIDTH_MM,
        "rail_length_mm": RAIL_LENGTH_MM,
        "rail_width_y_mm": RAIL_WIDTH_Y_MM,
        "rail_thickness_z_mm": RAIL_THICKNESS_Z_MM,
        "rail_offset_z_mm": RAIL_OFFSET_Z_MM,
        "web_length_mm": WEB_LENGTH_MM,
        "web_thickness_y_mm": WEB_THICKNESS_Y_MM,
        "web_height_z_mm": WEB_HEIGHT_Z_MM,
        "rib_x_positions_mm": list(RIB_X_POSITIONS_MM),
        "lightening_hole_x_positions_mm": list(LIGHTENING_HOLE_X_POSITIONS_MM),
        "design_intent": "first AI-assisted parametric upper_arm_link concept; preserve shoulder/elbow joint span and add ribs, cable channel, and lightening holes",
    }


def gen_step():
    return upper_arm_v1()


def export_all(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    shape = upper_arm_v1()
    export_step(shape, out_dir / "upper_arm_v1_ai_rebuild.step")
    export_stl(shape, out_dir / "upper_arm_v1_ai_rebuild.stl")
    (out_dir / "upper_arm_v1_parameters.json").write_text(
        json.dumps(parameters(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    export_all(args.out_dir)
    print(f"Exported STEP: {args.out_dir / 'upper_arm_v1_ai_rebuild.step'}")
    print(f"Exported STL : {args.out_dir / 'upper_arm_v1_ai_rebuild.stl'}")
    print(f"Exported params: {args.out_dir / 'upper_arm_v1_parameters.json'}")


if __name__ == "__main__":
    main()
