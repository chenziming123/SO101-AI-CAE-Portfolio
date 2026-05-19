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
GRAVITY = 9.81


def parse_payloads(value: str) -> list[float]:
    payloads = [float(item.strip()) for item in value.split(",") if item.strip()]
    if not payloads:
        raise argparse.ArgumentTypeError("At least one payload mass is required")
    if any(payload < 0 for payload in payloads):
        raise argparse.ArgumentTypeError("Payload masses must be non-negative")
    return payloads


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf",
    )
    parser.add_argument("--out-dir", type=Path, default=project_root / "04_structural_analysis")
    parser.add_argument("--samples", type=int, default=10000)
    parser.add_argument("--payloads", type=parse_payloads, default=parse_payloads("0.0,0.05,0.10,0.20"))
    parser.add_argument("--report-payload", type=float, default=0.10)
    parser.add_argument("--seed", type=int, default=2027)
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


def collect_joints(robot_id: int, client: int) -> tuple[dict[str, dict], list[dict]]:
    joints_by_name: dict[str, dict] = {}
    active_joints: list[dict] = []
    for idx in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, idx, physicsClientId=client)
        name = info[1].decode("utf-8")
        joint = {
            "name": name,
            "index": idx,
            "type": info[2],
            "lower": float(info[8]),
            "upper": float(info[9]),
            "child_link": info[12].decode("utf-8"),
        }
        joints_by_name[name] = joint
        if info[2] != p.JOINT_FIXED:
            joint["dof_index"] = len(active_joints)
            active_joints.append(joint)
    return joints_by_name, active_joints


def link_index_by_name(joints_by_name: dict[str, dict], link_name: str) -> int | None:
    for joint in joints_by_name.values():
        if joint["child_link"] == link_name:
            return int(joint["index"])
    return None


def default_joint_position(joint: dict) -> float:
    lower = float(joint["lower"])
    upper = float(joint["upper"])
    if np.isfinite(lower) and np.isfinite(upper) and lower < upper:
        if lower <= 0.0 <= upper:
            return 0.0
        return 0.5 * (lower + upper)
    return 0.0


def sample_arm_joint_values(arm_joints: list[dict], samples: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    lower = np.array([joint["lower"] for joint in arm_joints], dtype=np.float64)
    upper = np.array([joint["upper"] for joint in arm_joints], dtype=np.float64)
    if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)) or np.any(lower >= upper):
        raise RuntimeError("All sampled arm joints must have finite lower/upper limits")
    return rng.uniform(lower, upper, size=(samples, len(arm_joints)))


def set_joint_states(robot_id: int, active_joints: list[dict], q_active: np.ndarray, client: int) -> None:
    for joint, q_value in zip(active_joints, q_active):
        p.resetJointState(robot_id, joint["index"], float(q_value), physicsClientId=client)


def calculate_static_torques(
    robot_id: int,
    active_joints: list[dict],
    ee_link: int,
    q_active: np.ndarray,
    payload_masses: list[float],
    client: int,
) -> tuple[np.ndarray, dict[float, np.ndarray], np.ndarray]:
    set_joint_states(robot_id, active_joints, q_active, client)
    zeros = np.zeros(len(active_joints), dtype=np.float64)
    q_list = q_active.astype(float).tolist()
    zero_list = zeros.tolist()

    gravity_tau = np.asarray(
        p.calculateInverseDynamics(robot_id, q_list, zero_list, zero_list, physicsClientId=client),
        dtype=np.float64,
    )
    jac_linear, _jac_angular = p.calculateJacobian(
        robot_id,
        ee_link,
        [0.0, 0.0, 0.0],
        q_list,
        zero_list,
        zero_list,
        physicsClientId=client,
    )
    jac_linear = np.asarray(jac_linear, dtype=np.float64)

    totals: dict[float, np.ndarray] = {}
    for payload_mass in payload_masses:
        external_force = np.array([0.0, 0.0, -payload_mass * GRAVITY], dtype=np.float64)
        payload_hold_tau = -jac_linear.T @ external_force
        totals[payload_mass] = gravity_tau + payload_hold_tau

    ee_state = p.getLinkState(robot_id, ee_link, computeForwardKinematics=True, physicsClientId=client)
    ee_pos = np.asarray(ee_state[4], dtype=np.float64)
    return gravity_tau, totals, ee_pos


def write_sample_csv(
    path: Path,
    arm_samples: np.ndarray,
    ee_positions: np.ndarray,
    payload_torques: dict[float, np.ndarray],
    arm_joint_names: list[str],
    arm_active_cols: list[int],
    payloads: list[float],
) -> None:
    fieldnames = ["sample_id"] + arm_joint_names + ["ee_x", "ee_y", "ee_z", "radius_xy"]
    for payload in payloads:
        payload_tag = f"p{payload:.2f}kg"
        for joint_name in arm_joint_names:
            fieldnames.append(f"tau_{payload_tag}_{joint_name}_Nm")

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for sample_id, (q_arm, ee_pos) in enumerate(zip(arm_samples, ee_positions)):
            row = {"sample_id": sample_id}
            row.update({joint_name: float(value) for joint_name, value in zip(arm_joint_names, q_arm)})
            row["ee_x"] = float(ee_pos[0])
            row["ee_y"] = float(ee_pos[1])
            row["ee_z"] = float(ee_pos[2])
            row["radius_xy"] = float(np.linalg.norm(ee_pos[:2]))
            for payload in payloads:
                payload_tag = f"p{payload:.2f}kg"
                arm_tau = payload_torques[payload][sample_id, arm_active_cols]
                for joint_name, tau in zip(arm_joint_names, arm_tau):
                    row[f"tau_{payload_tag}_{joint_name}_Nm"] = float(tau)
            writer.writerow(row)


def torque_stats(torque_matrix: np.ndarray) -> list[dict[str, float]]:
    abs_tau = np.abs(torque_matrix)
    stats = []
    for col in range(abs_tau.shape[1]):
        stats.append(
            {
                "max_abs": float(np.max(abs_tau[:, col])),
                "p95_abs": float(np.percentile(abs_tau[:, col], 95)),
                "mean_abs": float(np.mean(abs_tau[:, col])),
                "rms": float(np.sqrt(np.mean(torque_matrix[:, col] ** 2))),
            }
        )
    return stats


def format_stats_table(payloads: list[float], stats_by_payload: dict[float, list[dict[str, float]]], joint_names: list[str]) -> list[str]:
    lines = [
        "| payload kg | joint | max abs Nm | p95 abs Nm | mean abs Nm | rms Nm |",
        "|---:|---|---:|---:|---:|---:|",
    ]
    for payload in payloads:
        for joint_name, stats in zip(joint_names, stats_by_payload[payload]):
            lines.append(
                f"| {payload:.2f} | {joint_name} | {stats['max_abs']:.4f} | {stats['p95_abs']:.4f} | "
                f"{stats['mean_abs']:.4f} | {stats['rms']:.4f} |"
            )
    return lines


def plot_torque_summary(
    path: Path,
    payload_torques: dict[float, np.ndarray],
    ee_positions: np.ndarray,
    arm_joint_names: list[str],
    arm_active_cols: list[int],
    payloads: list[float],
    report_payload: float,
) -> None:
    payloads_to_plot = [payload for payload in payloads if payload in {0.0, report_payload, max(payloads)}]
    if report_payload not in payloads_to_plot:
        payloads_to_plot.append(report_payload)
    payloads_to_plot = sorted(set(payloads_to_plot))

    x = np.arange(len(arm_joint_names))
    width = 0.22 if len(payloads_to_plot) <= 3 else 0.16
    fig = plt.figure(figsize=(13, 9))

    ax_max = fig.add_subplot(2, 2, 1)
    for offset_idx, payload in enumerate(payloads_to_plot):
        abs_tau = np.abs(payload_torques[payload][:, arm_active_cols])
        offset = (offset_idx - (len(payloads_to_plot) - 1) / 2) * width
        ax_max.bar(x + offset, np.max(abs_tau, axis=0), width, label=f"{payload:.2f} kg")
    ax_max.set_title("Max Static Joint Torque")
    ax_max.set_ylabel("Nm")
    ax_max.set_xticks(x)
    ax_max.set_xticklabels(arm_joint_names, rotation=25, ha="right")
    ax_max.legend(title="Payload")

    ax_p95 = fig.add_subplot(2, 2, 2)
    for offset_idx, payload in enumerate(payloads_to_plot):
        abs_tau = np.abs(payload_torques[payload][:, arm_active_cols])
        offset = (offset_idx - (len(payloads_to_plot) - 1) / 2) * width
        ax_p95.bar(x + offset, np.percentile(abs_tau, 95, axis=0), width, label=f"{payload:.2f} kg")
    ax_p95.set_title("95th Percentile Static Joint Torque")
    ax_p95.set_ylabel("Nm")
    ax_p95.set_xticks(x)
    ax_p95.set_xticklabels(arm_joint_names, rotation=25, ha="right")
    ax_p95.legend(title="Payload")

    report_abs = np.abs(payload_torques[report_payload][:, arm_active_cols])
    bottleneck_col = int(np.argmax(np.percentile(report_abs, 95, axis=0)))
    ax_hist = fig.add_subplot(2, 2, 3)
    ax_hist.hist(report_abs[:, bottleneck_col], bins=55, color="tab:orange", alpha=0.82)
    ax_hist.set_title(f"{arm_joint_names[bottleneck_col]} Torque Distribution ({report_payload:.2f} kg payload)")
    ax_hist.set_xlabel("abs torque / Nm")
    ax_hist.set_ylabel("sample count")

    ax_scatter = fig.add_subplot(2, 2, 4)
    radius = np.linalg.norm(ee_positions[:, :2], axis=1)
    max_joint_abs = np.max(report_abs, axis=1)
    scatter = ax_scatter.scatter(radius, max_joint_abs, s=4, c=ee_positions[:, 2], cmap="viridis", alpha=0.35)
    ax_scatter.set_title(f"Reach Radius vs Max Joint Torque ({report_payload:.2f} kg payload)")
    ax_scatter.set_xlabel("horizontal radius / m")
    ax_scatter.set_ylabel("max abs joint torque / Nm")
    fig.colorbar(scatter, ax=ax_scatter, label="ee z / m")

    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def write_summary(
    path: Path,
    args: argparse.Namespace,
    arm_joint_names: list[str],
    active_joint_names: list[str],
    payloads: list[float],
    payload_torques: dict[float, np.ndarray],
    arm_active_cols: list[int],
    ee_positions: np.ndarray,
) -> None:
    stats_by_payload = {
        payload: torque_stats(payload_torques[payload][:, arm_active_cols]) for payload in payloads
    }
    report_abs = np.abs(payload_torques[args.report_payload][:, arm_active_cols])
    p95_by_joint = np.percentile(report_abs, 95, axis=0)
    max_by_joint = np.max(report_abs, axis=0)
    bottleneck_p95_col = int(np.argmax(p95_by_joint))
    bottleneck_max_col = int(np.argmax(max_by_joint))
    radius = np.linalg.norm(ee_positions[:, :2], axis=1)

    lines = [
        "# SO-101 Baseline Static Joint Torque Analysis",
        "",
        "## Purpose",
        "",
        "This report estimates the official SO-101 Follower static joint torque demand before any AI-assisted structural changes.",
        "",
        "## Source and Assumptions",
        "",
        f"- URDF: `{args.urdf}`",
        f"- End-effector payload application point: `{EE_LINK_NAME}`",
        f"- Sampled main arm joints: {', '.join(arm_joint_names)}",
        f"- Active PyBullet joints used by inverse dynamics: {', '.join(active_joint_names)}",
        f"- Random static postures: {args.samples}",
        f"- Gravity: {GRAVITY:.2f} m/s^2",
        f"- Payload masses: {', '.join(f'{payload:.2f} kg' for payload in payloads)}",
        f"- Main reported payload: {args.report_payload:.2f} kg",
        "",
        "Method:",
        "- Link self-weight torque is computed with PyBullet inverse dynamics using `qdot=0` and `qddot=0`.",
        "- Payload torque is estimated as `-J^T F`, where `J` is the end-effector translational Jacobian and `F` is the downward payload force.",
        "- This is a static estimate. It excludes acceleration torque, friction, cable drag, contact force, servo thermal limits, and manufacturer torque curves.",
        "",
        "## Torque Statistics",
        "",
        *format_stats_table(payloads, stats_by_payload, arm_joint_names),
        "",
        f"## Main Finding at {args.report_payload:.2f} kg Payload",
        "",
        f"- Highest 95th-percentile torque joint: `{arm_joint_names[bottleneck_p95_col]}` = {p95_by_joint[bottleneck_p95_col]:.4f} Nm.",
        f"- Highest sampled peak torque joint: `{arm_joint_names[bottleneck_max_col]}` = {max_by_joint[bottleneck_max_col]:.4f} Nm.",
        f"- Sampled horizontal reach radius: {radius.min():.3f} to {radius.max():.3f} m.",
        f"- Sampled end-effector z range: {ee_positions[:, 2].min():.3f} to {ee_positions[:, 2].max():.3f} m.",
        "",
        "## Engineering Interpretation",
        "",
        "- Static gravity load mainly matters for pitch-chain joints: shoulder_lift, elbow_flex, and wrist_flex.",
        "- shoulder_pan is not expected to be the gravity bottleneck because its axis is close to vertical; it matters more for dynamic yaw motion and base stiffness.",
        "- Distal mass has strong leverage. Reducing wrist/gripper/lower-arm mass usually reduces upstream shoulder and elbow torque demand.",
        "- For AI-assisted redesign, the safest first targets are upper_arm, lower_arm, wrist/gripper, and base fixture features while preserving official joint axes and link lengths.",
        "",
        "## Generated Files",
        "",
        "- `04_structural_analysis/baseline_static_torque_samples.csv`",
        "- `04_structural_analysis/baseline_static_torque_plot.png`",
        "- `04_structural_analysis/baseline_static_torque_summary.md`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if not args.urdf.exists():
        raise FileNotFoundError(args.urdf)
    payloads = sorted(set(args.payloads))
    if args.report_payload not in payloads:
        payloads.append(args.report_payload)
        payloads = sorted(set(payloads))

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -GRAVITY, physicsClientId=client)
    robot_id = load_robot(args.urdf.resolve(), client)
    joints_by_name, active_joints = collect_joints(robot_id, client)

    missing = sorted(set(ARM_JOINT_NAMES) - set(joints_by_name))
    if missing:
        raise RuntimeError(f"Missing expected arm joints: {missing}")
    arm_joints = [joints_by_name[name] for name in ARM_JOINT_NAMES]
    active_joint_names = [joint["name"] for joint in active_joints]
    arm_active_cols = [active_joint_names.index(joint_name) for joint_name in ARM_JOINT_NAMES]

    ee_link = link_index_by_name(joints_by_name, EE_LINK_NAME)
    if ee_link is None:
        raise RuntimeError(f"End-effector link not found: {EE_LINK_NAME}")

    q_default = np.array([default_joint_position(joint) for joint in active_joints], dtype=np.float64)
    gripper = joints_by_name.get(GRIPPER_JOINT_NAME)
    if gripper and GRIPPER_JOINT_NAME in active_joint_names:
        q_default[active_joint_names.index(GRIPPER_JOINT_NAME)] = default_joint_position(gripper)

    arm_samples = sample_arm_joint_values(arm_joints, args.samples, args.seed)
    payload_torques = {payload: np.empty((args.samples, len(active_joints)), dtype=np.float64) for payload in payloads}
    ee_positions = np.empty((args.samples, 3), dtype=np.float64)

    for sample_idx, q_arm in enumerate(arm_samples):
        q_active = q_default.copy()
        for joint_name, q_value in zip(ARM_JOINT_NAMES, q_arm):
            q_active[active_joint_names.index(joint_name)] = q_value
        _gravity_tau, totals, ee_pos = calculate_static_torques(
            robot_id,
            active_joints,
            ee_link,
            q_active,
            payloads,
            client,
        )
        for payload in payloads:
            payload_torques[payload][sample_idx] = totals[payload]
        ee_positions[sample_idx] = ee_pos

    p.disconnect(client)

    write_sample_csv(
        args.out_dir / "baseline_static_torque_samples.csv",
        arm_samples,
        ee_positions,
        payload_torques,
        ARM_JOINT_NAMES,
        arm_active_cols,
        payloads,
    )
    plot_torque_summary(
        args.out_dir / "baseline_static_torque_plot.png",
        payload_torques,
        ee_positions,
        ARM_JOINT_NAMES,
        arm_active_cols,
        payloads,
        args.report_payload,
    )
    write_summary(
        args.out_dir / "baseline_static_torque_summary.md",
        args,
        ARM_JOINT_NAMES,
        active_joint_names,
        payloads,
        payload_torques,
        arm_active_cols,
        ee_positions,
    )

    report_abs = np.abs(payload_torques[args.report_payload][:, arm_active_cols])
    p95 = np.percentile(report_abs, 95, axis=0)
    peak = np.max(report_abs, axis=0)
    print(f"Saved samples to {args.out_dir / 'baseline_static_torque_samples.csv'}")
    print(f"Saved plot to {args.out_dir / 'baseline_static_torque_plot.png'}")
    print(f"Saved summary to {args.out_dir / 'baseline_static_torque_summary.md'}")
    print(f"samples: {args.samples}")
    print(f"report_payload_kg: {args.report_payload:.2f}")
    print(f"p95_abs_torque_Nm: {dict(zip(ARM_JOINT_NAMES, [float(value) for value in np.round(p95, 4)]))}")
    print(f"peak_abs_torque_Nm: {dict(zip(ARM_JOINT_NAMES, [float(value) for value in np.round(peak, 4)]))}")
    print(f"bottleneck_p95: {ARM_JOINT_NAMES[int(np.argmax(p95))]}")
    print(f"bottleneck_peak: {ARM_JOINT_NAMES[int(np.argmax(peak))]}")


if __name__ == "__main__":
    main()
