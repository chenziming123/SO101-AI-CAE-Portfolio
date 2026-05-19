from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data


ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf",
    )
    parser.add_argument("--out-dir", type=Path, default=project_root / "02_baseline_validation")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--steps", type=int, default=1800)
    return parser.parse_args()


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


def joint_map(robot_id: int, client: int) -> dict[str, dict]:
    joints = {}
    for idx in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, idx, physicsClientId=client)
        name = info[1].decode("utf-8")
        joints[name] = {
            "index": idx,
            "type": info[2],
            "lower": float(info[8]),
            "upper": float(info[9]),
            "force": max(float(info[10]), 10.0),
            "velocity": max(float(info[11]), 1.0),
            "child_link": info[12].decode("utf-8"),
        }
    return joints


def find_link_index(joints: dict[str, dict], link_name: str) -> int | None:
    for joint in joints.values():
        if joint["child_link"] == link_name:
            return int(joint["index"])
    return None


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(args.seed)
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    p.setTimeStep(1.0 / 240.0, physicsClientId=client)
    p.loadURDF("plane.urdf", physicsClientId=client)
    robot_id = load_robot(args.urdf.resolve(), client)

    joints = joint_map(robot_id, client)
    arm_joints = [joints[name] for name in ARM_JOINT_NAMES if name in joints]
    gripper = joints.get("gripper")
    ee_link = find_link_index(joints, "gripper_frame_link")
    if ee_link is None:
        ee_link = p.getNumJoints(robot_id, physicsClientId=client) - 1

    for joint in arm_joints:
        mid = 0.5 * (joint["lower"] + joint["upper"])
        p.resetJointState(robot_id, joint["index"], mid, physicsClientId=client)
    if gripper:
        p.resetJointState(robot_id, gripper["index"], 0.25, physicsClientId=client)

    log_lines = [
        "# SO-101 Random Motion Smoke Test",
        "",
        f"URDF: `{args.urdf}`",
        f"Seed: {args.seed}",
        f"Steps: {args.steps}",
        f"Controlled arm joints: {', '.join([name for name in ARM_JOINT_NAMES if name in joints])}",
        f"End-effector link index: {ee_link}",
        "",
    ]
    ee_positions = []
    target_q = np.array([0.5 * (j["lower"] + j["upper"]) for j in arm_joints], dtype=np.float32)

    for step in range(args.steps):
        if step % 240 == 0:
            sampled = []
            for joint in arm_joints:
                center = 0.5 * (joint["lower"] + joint["upper"])
                half_range = 0.5 * (joint["upper"] - joint["lower"])
                sampled.append(center + rng.uniform(-0.35, 0.35) * half_range)
            target_q = np.asarray(sampled, dtype=np.float32)

        for joint, q in zip(arm_joints, target_q):
            p.setJointMotorControl2(
                robot_id,
                joint["index"],
                p.POSITION_CONTROL,
                targetPosition=float(q),
                force=joint["force"],
                maxVelocity=joint["velocity"],
                positionGain=0.08,
                velocityGain=1.0,
                physicsClientId=client,
            )
        if gripper:
            p.setJointMotorControl2(
                robot_id,
                gripper["index"],
                p.POSITION_CONTROL,
                targetPosition=0.25,
                force=gripper["force"],
                maxVelocity=gripper["velocity"],
                physicsClientId=client,
            )

        p.stepSimulation(physicsClientId=client)

        if step % 240 == 0:
            joint_states = [p.getJointState(robot_id, joint["index"], physicsClientId=client)[0] for joint in arm_joints]
            ee_pos = np.asarray(
                p.getLinkState(robot_id, ee_link, computeForwardKinematics=True, physicsClientId=client)[4],
                dtype=np.float32,
            )
            ee_positions.append(ee_pos)
            log_lines.extend(
                [
                    f"step={step}",
                    f"  target_q={np.round(target_q, 4).tolist()}",
                    f"  joint_q={np.round(joint_states, 4).tolist()}",
                    f"  ee_pos={np.round(ee_pos, 4).tolist()}",
                    "",
                ]
            )

    ee_positions = np.asarray(ee_positions, dtype=np.float32)
    movement_range = ee_positions.max(axis=0) - ee_positions.min(axis=0)
    log_lines.extend(
        [
            "## Result",
            "",
            "Random motion smoke test completed successfully.",
            f"End-effector sampled movement range xyz: {np.round(movement_range, 4).tolist()} m",
            "",
            "Interpretation: the official URDF can be loaded and controlled in PyBullet. This is sufficient as a baseline before AI-assisted rebuild.",
        ]
    )
    out_path = args.out_dir / "random_motion_log.md"
    out_path.write_text("\n".join(log_lines) + "\n", encoding="utf-8")
    p.disconnect(client)

    print("SO-101 random motion smoke test finished")
    print(f"Saved log: {out_path}")
    print(f"End-effector movement range: {np.round(movement_range, 4)}")


if __name__ == "__main__":
    main()
