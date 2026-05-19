from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data


ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINT_NAME = "gripper"
EE_LINK_NAME = "gripper_frame_link"

TASK_LOW = np.array([0.15, -0.25, 0.02], dtype=np.float32)
TASK_HIGH = np.array([0.40, 0.25, 0.35], dtype=np.float32)


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf",
    )
    parser.add_argument("--out-dir", type=Path, default=project_root / "04_structural_analysis")
    parser.add_argument("--samples", type=int, default=30000)
    parser.add_argument("--task-samples", type=int, default=1600)
    parser.add_argument("--threshold", type=float, default=0.035)
    parser.add_argument("--voxel-size", type=float, default=0.025)
    parser.add_argument("--seed", type=int, default=2026)
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
            "name": name,
            "index": idx,
            "type": info[2],
            "lower": float(info[8]),
            "upper": float(info[9]),
            "child_link": info[12].decode("utf-8"),
        }
    return joints


def link_index_by_name(joints: dict[str, dict], link_name: str) -> int | None:
    for joint in joints.values():
        if joint["child_link"] == link_name:
            return int(joint["index"])
    return None


def sample_workspace(robot_id: int, client: int, joints: list[dict], ee_link: int, samples: int, seed: int):
    rng = np.random.default_rng(seed)
    lower = np.array([joint["lower"] for joint in joints], dtype=np.float32)
    upper = np.array([joint["upper"] for joint in joints], dtype=np.float32)
    joint_samples = rng.uniform(lower, upper, size=(samples, len(joints))).astype(np.float32)
    positions = np.empty((samples, 3), dtype=np.float32)

    for row_idx, q in enumerate(joint_samples):
        for joint, q_value in zip(joints, q):
            p.resetJointState(robot_id, joint["index"], float(q_value), physicsClientId=client)
        ee_state = p.getLinkState(robot_id, ee_link, computeForwardKinematics=True, physicsClientId=client)
        positions[row_idx] = np.asarray(ee_state[4], dtype=np.float32)

    return joint_samples, positions


def nearest_distances(points: np.ndarray, targets: np.ndarray, chunk_size: int = 100) -> np.ndarray:
    nearest = np.empty(targets.shape[0], dtype=np.float32)
    for start in range(0, targets.shape[0], chunk_size):
        stop = min(start + chunk_size, targets.shape[0])
        diff = targets[start:stop, None, :] - points[None, :, :]
        dist_sq = np.sum(diff * diff, axis=2)
        nearest[start:stop] = np.sqrt(np.min(dist_sq, axis=1))
    return nearest


def estimate_reachability(points: np.ndarray, task_samples: int, threshold: float, seed: int):
    rng = np.random.default_rng(seed + 1)
    targets = rng.uniform(TASK_LOW, TASK_HIGH, size=(task_samples, 3)).astype(np.float32)
    nearest = nearest_distances(points, targets)
    return targets, nearest, float(np.mean(nearest <= threshold))


def estimate_voxel_volume(points: np.ndarray, voxel_size: float) -> tuple[int, float]:
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)
    occupied = np.unique(voxel_indices, axis=0).shape[0]
    return occupied, occupied * voxel_size**3


def write_points_csv(path: Path, joint_samples: np.ndarray, points: np.ndarray, joint_names: list[str]) -> None:
    fieldnames = joint_names + ["ee_x", "ee_y", "ee_z", "radius_xy"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for q, pos in zip(joint_samples, points):
            row = {name: float(value) for name, value in zip(joint_names, q)}
            row["ee_x"] = float(pos[0])
            row["ee_y"] = float(pos[1])
            row["ee_z"] = float(pos[2])
            row["radius_xy"] = float(np.linalg.norm(pos[:2]))
            writer.writerow(row)


def plot_workspace(path: Path, points: np.ndarray, task_reachability: float, threshold: float) -> None:
    radius = np.linalg.norm(points[:, :2], axis=1)
    fig = plt.figure(figsize=(12, 9))

    ax_top = fig.add_subplot(2, 2, 1)
    top = ax_top.scatter(points[:, 0], points[:, 1], s=2, c=points[:, 2], cmap="viridis", alpha=0.32)
    ax_top.add_patch(
        plt.Rectangle(
            (TASK_LOW[0], TASK_LOW[1]),
            TASK_HIGH[0] - TASK_LOW[0],
            TASK_HIGH[1] - TASK_LOW[1],
            fill=False,
            edgecolor="tab:red",
            linewidth=1.5,
        )
    )
    ax_top.set_title("SO-101 Baseline Workspace: Top View")
    ax_top.set_xlabel("x / m")
    ax_top.set_ylabel("y / m")
    ax_top.axis("equal")
    fig.colorbar(top, ax=ax_top, label="z / m")

    ax_side = fig.add_subplot(2, 2, 2)
    ax_side.scatter(radius, points[:, 2], s=2, color="tab:blue", alpha=0.25)
    ax_side.add_patch(
        plt.Rectangle(
            (float(np.linalg.norm([TASK_LOW[0], 0])), TASK_LOW[2]),
            float(np.linalg.norm([TASK_HIGH[0], 0]) - np.linalg.norm([TASK_LOW[0], 0])),
            TASK_HIGH[2] - TASK_LOW[2],
            fill=False,
            edgecolor="tab:red",
            linewidth=1.5,
        )
    )
    ax_side.set_title("SO-101 Baseline Workspace: Radius-Z")
    ax_side.set_xlabel("horizontal radius / m")
    ax_side.set_ylabel("z / m")

    ax_hist = fig.add_subplot(2, 2, 3)
    ax_hist.hist(radius, bins=55, color="tab:green", alpha=0.82)
    ax_hist.set_title("Reach Radius Distribution")
    ax_hist.set_xlabel("horizontal radius / m")
    ax_hist.set_ylabel("sample count")

    ax_text = fig.add_subplot(2, 2, 4)
    ax_text.axis("off")
    summary = (
        f"Task reachability estimate: {task_reachability:.1%}\n"
        f"Reach threshold: {threshold:.3f} m\n"
        f"x range: {points[:, 0].min():.3f} to {points[:, 0].max():.3f} m\n"
        f"y range: {points[:, 1].min():.3f} to {points[:, 1].max():.3f} m\n"
        f"z range: {points[:, 2].min():.3f} to {points[:, 2].max():.3f} m\n"
        f"max radius: {radius.max():.3f} m\n"
    )
    ax_text.text(0.02, 0.95, summary, va="top", ha="left", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def write_summary(
    path: Path,
    urdf_path: Path,
    joint_names: list[str],
    points: np.ndarray,
    occupied_voxels: int,
    volume: float,
    task_reachability: float,
    nearest: np.ndarray,
    args: argparse.Namespace,
) -> None:
    radius = np.linalg.norm(points[:, :2], axis=1)
    lines = [
        "# SO-101 Baseline Workspace and Reachability Analysis",
        "",
        "## Purpose",
        "",
        "This report quantifies the official SO-101 Follower workspace before any AI-assisted rebuild or structural improvement.",
        "",
        "## Source",
        "",
        f"- URDF: `{urdf_path}`",
        f"- End-effector: `{EE_LINK_NAME}`",
        f"- Sampled arm joints: {', '.join(joint_names)}",
        f"- Joint samples: {args.samples}",
        "",
        "## Workspace Statistics",
        "",
        f"- x range: {points[:, 0].min():.3f} to {points[:, 0].max():.3f} m",
        f"- y range: {points[:, 1].min():.3f} to {points[:, 1].max():.3f} m",
        f"- z range: {points[:, 2].min():.3f} to {points[:, 2].max():.3f} m",
        f"- horizontal radius: {radius.min():.3f} to {radius.max():.3f} m",
        f"- occupied voxels at {args.voxel_size:.3f} m resolution: {occupied_voxels}",
        f"- approximate sampled workspace volume: {volume:.5f} m^3",
        "",
        "## Task Region Reachability",
        "",
        f"- task box x: {TASK_LOW[0]:.2f} to {TASK_HIGH[0]:.2f} m",
        f"- task box y: {TASK_LOW[1]:.2f} to {TASK_HIGH[1]:.2f} m",
        f"- task box z: {TASK_LOW[2]:.2f} to {TASK_HIGH[2]:.2f} m",
        f"- nearest-distance threshold: {args.threshold:.3f} m",
        f"- estimated reachable target ratio: {task_reachability:.3f}",
        f"- mean nearest sampled distance: {nearest.mean():.4f} m",
        f"- 95th percentile nearest sampled distance: {np.percentile(nearest, 95):.4f} m",
        "",
        "## Engineering Interpretation",
        "",
        "- This is the official SO-101 baseline workspace. Later AI rebuild and improved designs should be compared against these ranges.",
        "- A structural improvement is acceptable only if it preserves the useful workspace for the chosen desktop task region.",
        "- If an improved design reduces reachability, the report must explain the tradeoff, such as improved stiffness or lower torque demand.",
        "",
        "## Generated Files",
        "",
        "- `04_structural_analysis/baseline_workspace_points.csv`",
        "- `04_structural_analysis/baseline_workspace_plot.png`",
        "- `04_structural_analysis/baseline_workspace_summary.md`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if not args.urdf.exists():
        raise FileNotFoundError(args.urdf)

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    robot_id = load_robot(args.urdf.resolve(), client)
    joints_by_name = joint_map(robot_id, client)
    arm_joints = [joints_by_name[name] for name in ARM_JOINT_NAMES if name in joints_by_name]
    if len(arm_joints) != len(ARM_JOINT_NAMES):
        missing = sorted(set(ARM_JOINT_NAMES) - set(joints_by_name))
        raise RuntimeError(f"Missing expected arm joints: {missing}")

    gripper = joints_by_name.get(GRIPPER_JOINT_NAME)
    if gripper:
        p.resetJointState(robot_id, gripper["index"], 0.25, physicsClientId=client)

    ee_link = link_index_by_name(joints_by_name, EE_LINK_NAME)
    if ee_link is None:
        raise RuntimeError(f"End-effector link not found: {EE_LINK_NAME}")

    joint_samples, points = sample_workspace(robot_id, client, arm_joints, ee_link, args.samples, args.seed)
    _, nearest, task_reachability = estimate_reachability(points, args.task_samples, args.threshold, args.seed)
    occupied_voxels, volume = estimate_voxel_volume(points, args.voxel_size)
    p.disconnect(client)

    joint_names = [joint["name"] for joint in arm_joints]
    write_points_csv(args.out_dir / "baseline_workspace_points.csv", joint_samples, points, joint_names)
    plot_workspace(args.out_dir / "baseline_workspace_plot.png", points, task_reachability, args.threshold)
    write_summary(
        args.out_dir / "baseline_workspace_summary.md",
        args.urdf,
        joint_names,
        points,
        occupied_voxels,
        volume,
        task_reachability,
        nearest,
        args,
    )

    radius = np.linalg.norm(points[:, :2], axis=1)
    print(f"Saved workspace points to {args.out_dir / 'baseline_workspace_points.csv'}")
    print(f"Saved workspace plot to {args.out_dir / 'baseline_workspace_plot.png'}")
    print(f"Saved workspace summary to {args.out_dir / 'baseline_workspace_summary.md'}")
    print(f"samples: {args.samples}")
    print(f"x_range: {points[:, 0].min():.3f} to {points[:, 0].max():.3f} m")
    print(f"y_range: {points[:, 1].min():.3f} to {points[:, 1].max():.3f} m")
    print(f"z_range: {points[:, 2].min():.3f} to {points[:, 2].max():.3f} m")
    print(f"max_radius: {radius.max():.3f} m")
    print(f"task_reachability: {task_reachability:.3f}")


if __name__ == "__main__":
    main()
