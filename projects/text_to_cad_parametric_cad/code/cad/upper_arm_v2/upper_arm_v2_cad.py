from __future__ import annotations

import argparse
import json
from pathlib import Path

from build123d import Box, Cylinder, Pos, export_step, export_stl


# Units: millimeters.
# Coordinate convention follows upper_arm_v1:
# - origin is the shoulder_lift joint center;
# - +X points from shoulder_lift toward elbow_flex;
# - +Y is the joint shaft / servo width direction;
# - +Z is the weak-axis bending direction.

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
WEB_THICKNESS_Y_MM = 3.0
WEB_HEIGHT_Z_MM = 28.0

RIB_THICKNESS_X_MM = 5.5
RIB_WIDTH_Y_MM = 18.0
RIB_HEIGHT_Z_MM = 35.0
RIB_X_POSITIONS_MM = (26.0, 58.0, 90.0)

CABLE_CHANNEL_LENGTH_MM = SPAN_MM - 34.0
CABLE_CHANNEL_WIDTH_Y_MM = 8.0
CABLE_CHANNEL_WALL_MM = 1.8
CABLE_CHANNEL_Z_MM = RAIL_OFFSET_Z_MM + RAIL_THICKNESS_Z_MM / 2.0 + 1.2

LIGHTENING_HOLE_RADIUS_MM = 5.4
LIGHTENING_HOLE_X_POSITIONS_MM = (38.0, 58.0, 78.0)

SHOULDER_PAD_X_MM = -5.0
SHOULDER_PAD_SIZE_X_MM = 22.0
SHOULDER_PAD_SIZE_Y_MM = 36.0
SHOULDER_PAD_SIZE_Z_MM = 30.0
SHOULDER_CLAMP_HOLE_X_MM = -8.0
SHOULDER_CLAMP_HOLE_Z_MM = 10.5
SHOULDER_CLAMP_HOLE_RADIUS_MM = 2.0
SHOULDER_CLAMP_HOLE_SPAN_Z_MM = 21.0

ELBOW_PAD_X_MM = SPAN_MM - 11.0
ELBOW_PAD_SIZE_X_MM = 28.0
ELBOW_PAD_SIZE_Y_MM = 32.0
ELBOW_PAD_SIZE_Z_MM = 35.0
ELBOW_SERVO_MOUNT_CENTER_X_MM = SPAN_MM - 11.0
ELBOW_SERVO_MOUNT_SPACING_X_MM = 9.9
ELBOW_SERVO_MOUNT_SPACING_Y_MM = 9.9
ELBOW_SERVO_HOLE_RADIUS_MM = 1.6
ELBOW_SERVO_COUNTERBORE_RADIUS_MM = 2.7
ELBOW_SERVO_COUNTERBORE_DEPTH_MM = 2.2


def y_cylinder(radius: float, length: float):
    return Cylinder(radius=radius, height=length, rotation=(90, 0, 0))


def z_cylinder(radius: float, length: float):
    return Cylinder(radius=radius, height=length)


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
    return [
        Pos(x_pos, 0, 0) * Box(RIB_THICKNESS_X_MM, RIB_WIDTH_Y_MM, RIB_HEIGHT_Z_MM)
        for x_pos in RIB_X_POSITIONS_MM
    ]


def shoulder_interface():
    boss = Pos(0, 0, 0) * bored_y_cylinder(
        SHOULDER_BOSS_RADIUS_MM,
        JOINT_BORE_RADIUS_MM,
        BOSS_WIDTH_MM,
    )
    pad = Pos(SHOULDER_PAD_X_MM, 0, 0) * Box(
        SHOULDER_PAD_SIZE_X_MM,
        SHOULDER_PAD_SIZE_Y_MM,
        SHOULDER_PAD_SIZE_Z_MM,
    )
    side_pad = Pos(8.0, 0, 0) * Box(14.0, 30.0, 28.0)
    return [boss, pad, side_pad]


def elbow_interface():
    boss = Pos(SPAN_MM, 0, 0) * bored_y_cylinder(
        ELBOW_BOSS_RADIUS_MM,
        JOINT_BORE_RADIUS_MM,
        BOSS_WIDTH_MM,
    )
    servo_pad = Pos(ELBOW_PAD_X_MM, 0, 0) * Box(
        ELBOW_PAD_SIZE_X_MM,
        ELBOW_PAD_SIZE_Y_MM,
        ELBOW_PAD_SIZE_Z_MM,
    )
    outer_stiffener = Pos(SPAN_MM - 15.0, 0, 0) * Box(10.0, 24.0, 35.0)
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


def elbow_servo_mount_holes():
    holes = []
    half_x = ELBOW_SERVO_MOUNT_SPACING_X_MM / 2.0
    half_y = ELBOW_SERVO_MOUNT_SPACING_Y_MM / 2.0
    through_length = ELBOW_PAD_SIZE_Z_MM + 8.0
    top_z = ELBOW_PAD_SIZE_Z_MM / 2.0
    bottom_z = -ELBOW_PAD_SIZE_Z_MM / 2.0
    for x_offset in (-half_x, half_x):
        for y_offset in (-half_y, half_y):
            x_pos = ELBOW_SERVO_MOUNT_CENTER_X_MM + x_offset
            y_pos = y_offset
            holes.append(Pos(x_pos, y_pos, 0) * z_cylinder(ELBOW_SERVO_HOLE_RADIUS_MM, through_length))
            holes.append(
                Pos(x_pos, y_pos, top_z - ELBOW_SERVO_COUNTERBORE_DEPTH_MM / 2.0)
                * z_cylinder(ELBOW_SERVO_COUNTERBORE_RADIUS_MM, ELBOW_SERVO_COUNTERBORE_DEPTH_MM + 0.1)
            )
            holes.append(
                Pos(x_pos, y_pos, bottom_z + ELBOW_SERVO_COUNTERBORE_DEPTH_MM / 2.0)
                * z_cylinder(ELBOW_SERVO_COUNTERBORE_RADIUS_MM, ELBOW_SERVO_COUNTERBORE_DEPTH_MM + 0.1)
            )
    return holes


def shoulder_clamp_holes():
    return [
        Pos(SHOULDER_CLAMP_HOLE_X_MM, 0, -SHOULDER_CLAMP_HOLE_Z_MM)
        * y_cylinder(SHOULDER_CLAMP_HOLE_RADIUS_MM, SHOULDER_PAD_SIZE_Y_MM + 8.0),
        Pos(SHOULDER_CLAMP_HOLE_X_MM, 0, SHOULDER_CLAMP_HOLE_Z_MM)
        * y_cylinder(SHOULDER_CLAMP_HOLE_RADIUS_MM, SHOULDER_PAD_SIZE_Y_MM + 8.0),
    ]


def primary_joint_bores():
    # Re-cut after all pads are fused so reinforcement blocks cannot fill the
    # actual shoulder/elbow shaft holes.
    bore_length = max(BOSS_WIDTH_MM, SHOULDER_PAD_SIZE_Y_MM, ELBOW_PAD_SIZE_Y_MM) + 8.0
    return [
        Pos(0, 0, 0) * y_cylinder(JOINT_BORE_RADIUS_MM, bore_length),
        Pos(SPAN_MM, 0, 0) * y_cylinder(JOINT_BORE_RADIUS_MM, bore_length),
    ]


def upper_arm_v2():
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

    for bore in primary_joint_bores():
        fused = fused - bore
    for hole in elbow_servo_mount_holes():
        fused = fused - hole
    for hole in shoulder_clamp_holes():
        fused = fused - hole

    fused.label = "so101_upper_arm_v2_ai_rebuild_mounting_features"
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
        "lightening_hole_radius_mm": LIGHTENING_HOLE_RADIUS_MM,
        "lightening_hole_x_positions_mm": list(LIGHTENING_HOLE_X_POSITIONS_MM),
        "shoulder_clamp_hole_x_mm": SHOULDER_CLAMP_HOLE_X_MM,
        "shoulder_clamp_hole_z_mm": SHOULDER_CLAMP_HOLE_Z_MM,
        "shoulder_clamp_hole_radius_mm": SHOULDER_CLAMP_HOLE_RADIUS_MM,
        "shoulder_clamp_hole_span_z_mm": SHOULDER_CLAMP_HOLE_SPAN_Z_MM,
        "elbow_servo_mount_center_x_mm": ELBOW_SERVO_MOUNT_CENTER_X_MM,
        "elbow_servo_mount_spacing_x_mm": ELBOW_SERVO_MOUNT_SPACING_X_MM,
        "elbow_servo_mount_spacing_y_mm": ELBOW_SERVO_MOUNT_SPACING_Y_MM,
        "elbow_servo_hole_radius_mm": ELBOW_SERVO_HOLE_RADIUS_MM,
        "elbow_servo_counterbore_radius_mm": ELBOW_SERVO_COUNTERBORE_RADIUS_MM,
        "elbow_servo_counterbore_depth_mm": ELBOW_SERVO_COUNTERBORE_DEPTH_MM,
        "design_intent": (
            "upper_arm_v2 extends v1 by adding explicit servo mounting holes, "
            "counterbores, and shoulder clamp/locating holes while preserving "
            "the shoulder-to-elbow joint span"
        ),
    }


def export_all(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    shape = upper_arm_v2()
    export_step(shape, out_dir / "upper_arm_v2_ai_rebuild.step")
    export_stl(shape, out_dir / "upper_arm_v2_ai_rebuild.stl")
    (out_dir / "upper_arm_v2_parameters.json").write_text(
        json.dumps(parameters(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    export_all(args.out_dir)
    print(f"Exported STEP: {args.out_dir / 'upper_arm_v2_ai_rebuild.step'}")
    print(f"Exported STL : {args.out_dir / 'upper_arm_v2_ai_rebuild.stl'}")
    print(f"Exported params: {args.out_dir / 'upper_arm_v2_parameters.json'}")


if __name__ == "__main__":
    main()
