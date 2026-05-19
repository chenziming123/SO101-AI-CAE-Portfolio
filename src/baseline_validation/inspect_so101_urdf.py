from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import pybullet as p
import pybullet_data


JOINT_TYPE_NAMES = {
    p.JOINT_REVOLUTE: "REVOLUTE",
    p.JOINT_PRISMATIC: "PRISMATIC",
    p.JOINT_SPHERICAL: "SPHERICAL",
    p.JOINT_PLANAR: "PLANAR",
    p.JOINT_FIXED: "FIXED",
}


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf",
    )
    parser.add_argument("--out-dir", type=Path, default=project_root / "02_baseline_validation")
    return parser.parse_args()


def joint_type_name(joint_type: int) -> str:
    return JOINT_TYPE_NAMES.get(joint_type, str(joint_type))


def load_robot(urdf_path: Path, client: int) -> int:
    original_cwd = Path.cwd()
    os.chdir(urdf_path.parent)
    try:
        return p.loadURDF(
            urdf_path.name,
            basePosition=[0, 0, 0],
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=client,
        )
    finally:
        os.chdir(original_cwd)


def collect_joint_rows(robot_id: int, client: int) -> list[dict]:
    rows = []
    for idx in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, idx, physicsClientId=client)
        dyn = p.getDynamicsInfo(robot_id, idx, physicsClientId=client)
        rows.append(
            {
                "index": idx,
                "joint_name": info[1].decode("utf-8"),
                "joint_type": joint_type_name(info[2]),
                "parent_index": info[16],
                "parent_link": "base_link" if info[16] == -1 else "",
                "child_link": info[12].decode("utf-8"),
                "axis": tuple(float(v) for v in info[13]),
                "origin_xyz": tuple(float(v) for v in info[14]),
                "lower": float(info[8]),
                "upper": float(info[9]),
                "effort": float(info[10]),
                "velocity": float(info[11]),
                "mass": float(dyn[0]),
                "local_inertia": tuple(float(v) for v in dyn[2]),
            }
        )

    child_to_parent = {row["child_link"]: row for row in rows}
    for row in rows:
        if row["parent_index"] >= 0:
            parent_info = p.getJointInfo(robot_id, row["parent_index"], physicsClientId=client)
            row["parent_link"] = parent_info[12].decode("utf-8")
        elif row["joint_name"] not in child_to_parent:
            row["parent_link"] = "base_link"
    return rows


def find_link_index(rows: list[dict], link_name: str) -> int | None:
    for row in rows:
        if row["child_link"] == link_name:
            return int(row["index"])
    return None


def render_preview(robot_id: int, out_path: Path, client: int) -> None:
    p.resetDebugVisualizerCamera(0.65, 45, -25, [0.0, 0.0, 0.12], physicsClientId=client)
    view = p.computeViewMatrix(
        cameraEyePosition=[0.45, -0.65, 0.35],
        cameraTargetPosition=[-0.05, 0.0, 0.08],
        cameraUpVector=[0, 0, 1],
    )
    projection = p.computeProjectionMatrixFOV(
        fov=55,
        aspect=4 / 3,
        nearVal=0.01,
        farVal=3.0,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width=1280,
        height=960,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=client,
    )
    image = np.asarray(rgba, dtype=np.uint8).reshape(960, 1280, 4)
    imageio.imwrite(out_path, image[:, :, :3])


def write_joint_csv(path: Path, rows: list[dict]) -> None:
    fieldnames = [
        "index",
        "joint_name",
        "joint_type",
        "parent_index",
        "parent_link",
        "child_link",
        "axis",
        "origin_xyz",
        "lower",
        "upper",
        "effort",
        "velocity",
        "mass",
        "local_inertia",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_report(path: Path, urdf_path: Path, rows: list[dict], ee_link_index: int | None, ee_pos) -> None:
    revolute_rows = [row for row in rows if row["joint_type"] == "REVOLUTE"]
    fixed_rows = [row for row in rows if row["joint_type"] == "FIXED"]
    arm_joint_names = [
        row["joint_name"]
        for row in revolute_rows
        if row["joint_name"] in {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}
    ]
    gripper_joints = [row["joint_name"] for row in revolute_rows if "gripper" in row["joint_name"]]

    lines = [
        "# SO-101 Official URDF Baseline Report",
        "",
        "## Source",
        "",
        f"- URDF: `{urdf_path}`",
        "- Model: SO-101 Follower, new calibration URDF",
        "",
        "## Load Result",
        "",
        "- PyBullet load: success",
        f"- Total joints: {len(rows)}",
        f"- Revolute joints: {len(revolute_rows)}",
        f"- Fixed joints: {len(fixed_rows)}",
        f"- Active arm joints selected for control: {', '.join(arm_joint_names)}",
        f"- Gripper joints: {', '.join(gripper_joints) if gripper_joints else 'none detected'}",
        f"- End-effector link index: {ee_link_index if ee_link_index is not None else 'not found'}",
        f"- End-effector world position at zero pose: {tuple(round(float(v), 5) for v in ee_pos)}",
        "",
        "## Joint Table",
        "",
        "| idx | joint | type | parent | child | axis | lower | upper | range deg |",
        "| ---: | --- | --- | --- | --- | --- | ---: | ---: | ---: |",
    ]
    for row in rows:
        lower = row["lower"]
        upper = row["upper"]
        if row["joint_type"] == "FIXED":
            range_deg = 0.0
        else:
            range_deg = np.degrees(upper - lower)
        lines.append(
            f"| {row['index']} | {row['joint_name']} | {row['joint_type']} | "
            f"{row['parent_link']} | {row['child_link']} | {row['axis']} | "
            f"{lower:.4f} | {upper:.4f} | {range_deg:.1f} |"
        )

    lines.extend(
        [
            "",
            "## Engineering Notes",
            "",
            "- The official model has five main arm revolute joints plus a revolute gripper joint.",
            "- The gripper is represented differently in LeRobot, where it is mapped as a linear value from 0 to 100; the official Simulation README notes this mapping is not fully reflected in the URDF/MJCF files.",
            "- The URDF mesh paths are relative to the simulation directory, so validation scripts load the URDF from its own folder.",
            "- This report is the baseline before any AI-assisted rebuild or structural improvement.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if not args.urdf.exists():
        raise FileNotFoundError(args.urdf)

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    robot_id = load_robot(args.urdf.resolve(), client)

    rows = collect_joint_rows(robot_id, client)
    ee_link_index = find_link_index(rows, "gripper_frame_link")
    if ee_link_index is None:
        ee_link_index = p.getNumJoints(robot_id, physicsClientId=client) - 1
    ee_state = p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True, physicsClientId=client)
    ee_pos = ee_state[4]

    write_joint_csv(args.out_dir / "so101_joint_summary.csv", rows)
    write_report(args.out_dir / "urdf_joint_report.md", args.urdf, rows, ee_link_index, ee_pos)
    render_preview(robot_id, args.out_dir / "baseline_preview.png", client)
    p.disconnect(client)

    print("SO-101 URDF loaded successfully")
    print(f"Saved joint summary: {args.out_dir / 'so101_joint_summary.csv'}")
    print(f"Saved report: {args.out_dir / 'urdf_joint_report.md'}")
    print(f"Saved preview: {args.out_dir / 'baseline_preview.png'}")
    print(f"Total joints: {len(rows)}")
    print(f"End-effector link index: {ee_link_index}")


if __name__ == "__main__":
    main()
