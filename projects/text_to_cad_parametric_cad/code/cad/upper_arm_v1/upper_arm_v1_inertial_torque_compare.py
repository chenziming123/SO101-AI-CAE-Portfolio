from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import xml.etree.ElementTree as ET
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
PAYLOADS_KG = [0.0, 0.05, 0.10, 0.20]
REPORT_PAYLOAD_KG = 0.10
INERTIA_FIELDS = ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[2]
    upper_arm_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=project_root)
    parser.add_argument("--upper-arm-dir", type=Path, default=upper_arm_dir)
    parser.add_argument("--samples", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=2029)
    return parser.parse_args()


def read_volume_ratio(validation_json: Path) -> dict[str, float]:
    data = json.loads(validation_json.read_text(encoding="utf-8"))
    official = data["official_upper_arm"]
    improved = data["improved_upper_arm_v1"]
    official_volume = float(official["volume"])
    improved_volume = float(improved["volume"])
    if official_volume <= 0 or improved_volume <= 0:
        raise RuntimeError("Invalid mesh volume in validation JSON")
    return {
        "official_volume_mm3": official_volume,
        "improved_volume_mm3": improved_volume,
        "volume_ratio": improved_volume / official_volume,
        "official_estimated_mass_g_pla": float(official["estimated_mass_g_pla"]),
        "improved_estimated_mass_g_pla": float(improved["estimated_mass_g_pla"]),
    }


def float_text(value: float) -> str:
    return f"{value:.10g}"


def get_upper_arm_inertial(root: ET.Element) -> tuple[float, dict[str, float], str]:
    link = root.find("./link[@name='upper_arm_link']")
    if link is None:
        raise RuntimeError("upper_arm_link not found")
    inertial = link.find("inertial")
    if inertial is None:
        raise RuntimeError("upper_arm_link inertial not found")
    mass_tag = inertial.find("mass")
    inertia_tag = inertial.find("inertia")
    origin_tag = inertial.find("origin")
    if mass_tag is None or inertia_tag is None:
        raise RuntimeError("upper_arm_link mass/inertia tag missing")
    mass = float(mass_tag.attrib["value"])
    inertia = {field: float(inertia_tag.attrib[field]) for field in INERTIA_FIELDS}
    origin_xyz = origin_tag.attrib.get("xyz", "") if origin_tag is not None else ""
    return mass, inertia, origin_xyz


def prepare_scaled_inertial_urdf(project_root: Path, upper_arm_dir: Path, volume_ratio: float) -> tuple[Path, Path, dict]:
    official_urdf = project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf"
    visual_smoke_dir = upper_arm_dir / "urdf_smoke"
    visual_smoke_urdf = visual_smoke_dir / "so101_upper_arm_v1_visual_smoke.urdf"
    out_dir = upper_arm_dir / "inertial_compare"
    out_assets = out_dir / "assets"
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copytree(visual_smoke_dir / "assets", out_assets, dirs_exist_ok=True)

    tree = ET.parse(visual_smoke_urdf)
    root = tree.getroot()
    link = root.find("./link[@name='upper_arm_link']")
    if link is None:
        raise RuntimeError("upper_arm_link not found in visual smoke URDF")
    inertial = link.find("inertial")
    if inertial is None:
        raise RuntimeError("upper_arm_link inertial not found")
    mass_tag = inertial.find("mass")
    inertia_tag = inertial.find("inertia")
    if mass_tag is None or inertia_tag is None:
        raise RuntimeError("upper_arm_link mass/inertia tag missing")

    official_mass = float(mass_tag.attrib["value"])
    official_inertia = {field: float(inertia_tag.attrib[field]) for field in INERTIA_FIELDS}
    scaled_mass = official_mass * volume_ratio
    scaled_inertia = {field: value * volume_ratio for field, value in official_inertia.items()}
    mass_tag.attrib["value"] = float_text(scaled_mass)
    for field, value in scaled_inertia.items():
        inertia_tag.attrib[field] = float_text(value)

    output_urdf = out_dir / "so101_upper_arm_v1_inertial_scaled.urdf"
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return official_urdf, output_urdf, {
        "official_upper_arm_link_mass_kg": official_mass,
        "scaled_upper_arm_link_mass_kg": scaled_mass,
        "mass_delta_kg": scaled_mass - official_mass,
        "mass_delta_pct": (scaled_mass / official_mass - 1.0) * 100.0,
        "official_inertia": official_inertia,
        "scaled_inertia": scaled_inertia,
    }


def load_robot(urdf_path: Path, client: int) -> int:
    old_cwd = Path.cwd()
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
        os.chdir(old_cwd)


def collect_joints(robot_id: int, client: int) -> tuple[dict[str, dict], list[dict]]:
    joints_by_name = {}
    active_joints = []
    for idx in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, idx, physicsClientId=client)
        name = info[1].decode("utf-8")
        joint = {
            "name": name,
            "index": idx,
            "type": int(info[2]),
            "lower": float(info[8]),
            "upper": float(info[9]),
            "child_link": info[12].decode("utf-8"),
        }
        joints_by_name[name] = joint
        if int(info[2]) != p.JOINT_FIXED:
            joint["dof_index"] = len(active_joints)
            active_joints.append(joint)
    return joints_by_name, active_joints


def link_index_by_name(joints_by_name: dict[str, dict], link_name: str) -> int:
    for joint in joints_by_name.values():
        if joint["child_link"] == link_name:
            return int(joint["index"])
    raise RuntimeError(f"link not found: {link_name}")


def default_joint_position(joint: dict) -> float:
    lower = float(joint["lower"])
    upper = float(joint["upper"])
    if np.isfinite(lower) and np.isfinite(upper) and lower < upper:
        if lower <= 0.0 <= upper:
            return 0.0
        return 0.5 * (lower + upper)
    return 0.0


def active_default_q(active_joints: list[dict]) -> np.ndarray:
    return np.asarray([default_joint_position(joint) for joint in active_joints], dtype=float)


def set_joint_states(robot_id: int, active_joints: list[dict], q_active: np.ndarray, client: int) -> None:
    for joint, value in zip(active_joints, q_active):
        p.resetJointState(robot_id, int(joint["index"]), float(value), physicsClientId=client)


def calculate_static_torque(
    robot_id: int,
    active_joints: list[dict],
    ee_link: int,
    q_active: np.ndarray,
    payloads: list[float],
    client: int,
) -> dict[float, np.ndarray]:
    set_joint_states(robot_id, active_joints, q_active, client)
    zeros = np.zeros(len(active_joints), dtype=float)
    q_list = q_active.astype(float).tolist()
    zero_list = zeros.tolist()
    gravity_tau = np.asarray(
        p.calculateInverseDynamics(robot_id, q_list, zero_list, zero_list, physicsClientId=client),
        dtype=float,
    )
    jac_linear, _ = p.calculateJacobian(
        robot_id,
        ee_link,
        [0.0, 0.0, 0.0],
        q_list,
        zero_list,
        zero_list,
        physicsClientId=client,
    )
    jac_linear = np.asarray(jac_linear, dtype=float)
    totals = {}
    for payload in payloads:
        force = np.asarray([0.0, 0.0, -payload * GRAVITY], dtype=float)
        payload_hold_tau = -jac_linear.T @ force
        totals[payload] = gravity_tau + payload_hold_tau
    return totals


def sample_q(official_joints: dict[str, dict], samples: int, seed: int) -> np.ndarray:
    lower = np.asarray([official_joints[name]["lower"] for name in ARM_JOINT_NAMES], dtype=float)
    upper = np.asarray([official_joints[name]["upper"] for name in ARM_JOINT_NAMES], dtype=float)
    rng = np.random.default_rng(seed)
    sampled = rng.uniform(lower, upper, size=(samples, len(ARM_JOINT_NAMES)))
    return np.vstack([np.zeros((1, len(ARM_JOINT_NAMES))), sampled])


def run_comparison(official_urdf: Path, improved_urdf: Path, samples: int, seed: int):
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0.0, 0.0, -GRAVITY, physicsClientId=client)
    official_robot = load_robot(official_urdf, client)
    improved_robot = load_robot(improved_urdf, client)
    official_joints, official_active = collect_joints(official_robot, client)
    improved_joints, improved_active = collect_joints(improved_robot, client)
    active_names = [joint["name"] for joint in official_active]
    improved_active_names = [joint["name"] for joint in improved_active]
    if active_names != improved_active_names:
        raise RuntimeError(f"active joint mismatch: {active_names} != {improved_active_names}")

    arm_cols = [active_names.index(name) for name in ARM_JOINT_NAMES]
    official_ee = link_index_by_name(official_joints, EE_LINK_NAME)
    improved_ee = link_index_by_name(improved_joints, EE_LINK_NAME)
    q_default = active_default_q(official_active)
    if GRIPPER_JOINT_NAME in active_names:
        q_default[active_names.index(GRIPPER_JOINT_NAME)] = default_joint_position(official_joints[GRIPPER_JOINT_NAME])

    q_samples = sample_q(official_joints, samples, seed)
    official_tau = {payload: np.empty((len(q_samples), len(official_active)), dtype=float) for payload in PAYLOADS_KG}
    improved_tau = {payload: np.empty((len(q_samples), len(improved_active)), dtype=float) for payload in PAYLOADS_KG}

    for sample_idx, q_arm in enumerate(q_samples):
        q_active = q_default.copy()
        for name, value in zip(ARM_JOINT_NAMES, q_arm):
            q_active[active_names.index(name)] = value
        official_totals = calculate_static_torque(
            official_robot, official_active, official_ee, q_active, PAYLOADS_KG, client
        )
        improved_totals = calculate_static_torque(
            improved_robot, improved_active, improved_ee, q_active, PAYLOADS_KG, client
        )
        for payload in PAYLOADS_KG:
            official_tau[payload][sample_idx] = official_totals[payload]
            improved_tau[payload][sample_idx] = improved_totals[payload]

    p.disconnect(client)
    return {
        "active_joint_names": active_names,
        "arm_cols": arm_cols,
        "q_samples": q_samples,
        "official_tau": official_tau,
        "improved_tau": improved_tau,
    }


def stats(values: np.ndarray) -> dict[str, float]:
    abs_values = np.abs(values)
    return {
        "max_abs_nm": float(np.max(abs_values)),
        "p95_abs_nm": float(np.percentile(abs_values, 95)),
        "mean_abs_nm": float(np.mean(abs_values)),
        "rms_nm": float(np.sqrt(np.mean(values * values))),
    }


def build_stats_rows(comparison: dict) -> list[dict[str, object]]:
    rows = []
    arm_cols = comparison["arm_cols"]
    official_tau = comparison["official_tau"]
    improved_tau = comparison["improved_tau"]
    for payload in PAYLOADS_KG:
        for joint_name, col in zip(ARM_JOINT_NAMES, arm_cols):
            off = stats(official_tau[payload][:, col])
            imp = stats(improved_tau[payload][:, col])
            row = {"payload_kg": payload, "joint": joint_name}
            for key in off:
                row[f"official_{key}"] = off[key]
                row[f"improved_{key}"] = imp[key]
                row[f"delta_{key}"] = imp[key] - off[key]
                row[f"delta_pct_{key}"] = ((imp[key] / off[key] - 1.0) * 100.0) if abs(off[key]) > 1e-12 else 0.0
            rows.append(row)
    return rows


def write_stats_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_comparison(path: Path, rows: list[dict[str, object]]) -> None:
    report_rows = [row for row in rows if abs(float(row["payload_kg"]) - REPORT_PAYLOAD_KG) < 1e-9]
    joints = [str(row["joint"]) for row in report_rows]
    x = np.arange(len(joints))
    off_p95 = [float(row["official_p95_abs_nm"]) for row in report_rows]
    imp_p95 = [float(row["improved_p95_abs_nm"]) for row in report_rows]
    delta_p95 = [float(row["delta_p95_abs_nm"]) for row in report_rows]
    delta_pct = [float(row["delta_pct_p95_abs_nm"]) for row in report_rows]

    fig = plt.figure(figsize=(12, 8))
    ax_bar = fig.add_subplot(2, 2, 1)
    width = 0.36
    ax_bar.bar(x - width / 2, off_p95, width, label="official")
    ax_bar.bar(x + width / 2, imp_p95, width, label="upper_arm_v1")
    ax_bar.set_title("p95 static torque at 0.10 kg payload")
    ax_bar.set_ylabel("Nm")
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(joints, rotation=25, ha="right")
    ax_bar.legend()

    ax_delta = fig.add_subplot(2, 2, 2)
    ax_delta.bar(joints, delta_p95, color="tab:green")
    ax_delta.axhline(0, color="black", linewidth=0.8)
    ax_delta.set_title("p95 torque delta")
    ax_delta.set_ylabel("Nm")
    ax_delta.tick_params(axis="x", rotation=25)

    ax_pct = fig.add_subplot(2, 2, 3)
    ax_pct.bar(joints, delta_pct, color="tab:orange")
    ax_pct.axhline(0, color="black", linewidth=0.8)
    ax_pct.set_title("p95 torque percent change")
    ax_pct.set_ylabel("%")
    ax_pct.tick_params(axis="x", rotation=25)

    ax_text = fig.add_subplot(2, 2, 4)
    ax_text.axis("off")
    shoulder = next(row for row in report_rows if row["joint"] == "shoulder_lift")
    elbow = next(row for row in report_rows if row["joint"] == "elbow_flex")
    text = (
        f"shoulder_lift p95: {float(shoulder['official_p95_abs_nm']):.4f} -> "
        f"{float(shoulder['improved_p95_abs_nm']):.4f} Nm\n"
        f"shoulder_lift change: {float(shoulder['delta_p95_abs_nm']):.4f} Nm "
        f"({float(shoulder['delta_pct_p95_abs_nm']):.2f}%)\n"
        f"elbow_flex p95: {float(elbow['official_p95_abs_nm']):.4f} -> "
        f"{float(elbow['improved_p95_abs_nm']):.4f} Nm\n"
        f"elbow_flex change: {float(elbow['delta_p95_abs_nm']):.4f} Nm "
        f"({float(elbow['delta_pct_p95_abs_nm']):.2f}%)\n"
    )
    ax_text.text(0.02, 0.95, text, va="top", ha="left", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def write_report(path: Path, volume_info: dict, inertial_info: dict, rows: list[dict[str, object]], samples: int) -> None:
    report_rows = [row for row in rows if abs(float(row["payload_kg"]) - REPORT_PAYLOAD_KG) < 1e-9]
    shoulder = next(row for row in report_rows if row["joint"] == "shoulder_lift")
    elbow = next(row for row in report_rows if row["joint"] == "elbow_flex")
    lines = [
        "# upper_arm_v1 惯量更新与静载力矩对比报告",
        "",
        "## 目的",
        "",
        "验证 `upper_arm_v1` 减重后，在 URDF 惯量层面是否会降低 SO-101 的静载关节力矩。",
        "",
        "## 方法",
        "",
        "- 保留官方 URDF 作为 baseline，不覆盖 `00_source_snapshot/`。",
        "- 使用 Step 8 的 STL 体积对比得到 V1 / 官方 upper arm 的体积比例。",
        "- 将官方 `upper_arm_link` 的质量和惯量张量按同一比例缩放，生成 inertial screening 版 URDF。",
        "- 使用同一批随机关节姿态，对官方版和 V1 inertial 版分别计算静载关节力矩。",
        "- 该方法是工程筛查，不是最终真实惯量标定。",
        "",
        "## 质量与惯量更新",
        "",
        f"- 官方 upper arm STL 体积：{volume_info['official_volume_mm3']:.2f} mm^3",
        f"- V1 upper arm STL 体积：{volume_info['improved_volume_mm3']:.2f} mm^3",
        f"- V1 / 官方体积比例：{volume_info['volume_ratio']:.4f}",
        f"- 官方 URDF `upper_arm_link` 质量：{inertial_info['official_upper_arm_link_mass_kg']:.6f} kg",
        f"- V1 screening 质量：{inertial_info['scaled_upper_arm_link_mass_kg']:.6f} kg",
        f"- 质量变化：{inertial_info['mass_delta_kg']:.6f} kg ({inertial_info['mass_delta_pct']:.2f}%)",
        "",
        "说明：官方 URDF link 质量和 STL 体积估算质量并不完全一致。因此本步骤采用“URDF 质量按 STL 体积比例缩放”的筛查方式，目的是观察趋势，不作为最终物理参数。",
        "",
        f"## {REPORT_PAYLOAD_KG:.2f} kg Payload 下的 p95 静载力矩对比",
        "",
        "| 关节 | 官方 p95 Nm | V1 p95 Nm | 变化 Nm | 变化比例 |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in report_rows:
        lines.append(
            f"| {row['joint']} | {float(row['official_p95_abs_nm']):.4f} | "
            f"{float(row['improved_p95_abs_nm']):.4f} | "
            f"{float(row['delta_p95_abs_nm']):.4f} | "
            f"{float(row['delta_pct_p95_abs_nm']):.2f}% |"
        )
    lines.extend(
        [
            "",
            "## 主要结论",
            "",
            f"- `shoulder_lift` p95 力矩：{float(shoulder['official_p95_abs_nm']):.4f} -> {float(shoulder['improved_p95_abs_nm']):.4f} Nm，变化 {float(shoulder['delta_p95_abs_nm']):.4f} Nm ({float(shoulder['delta_pct_p95_abs_nm']):.2f}%)。",
            f"- `elbow_flex` p95 力矩：{float(elbow['official_p95_abs_nm']):.4f} -> {float(elbow['improved_p95_abs_nm']):.4f} Nm，变化 {float(elbow['delta_p95_abs_nm']):.4f} Nm ({float(elbow['delta_pct_p95_abs_nm']):.2f}%)。",
            "- 结果符合机械直觉：减轻 upper arm 主要降低 shoulder_lift 负担，对 elbow_flex 的影响很小，因为 upper arm 位于 elbow 上游。",
            "",
            "## 当前限制",
            "",
            "- 仍未进行真实 CAD 装配孔位校核。",
            "- 仍未进行真实材料、打印方向、局部螺丝柱的 FEA。",
            "- 当前 inertia 是按比例缩放的筛查值，不是从完整 CAD 质量属性直接导出的最终惯量。",
            "- 下一步应把这个结论整理成 before/after 对比表，并决定是否继续优化 lower_arm 或 gripper。",
            "",
            "## 生成文件",
            "",
            "- `05_improved_design/upper_arm_v1/inertial_compare/so101_upper_arm_v1_inertial_scaled.urdf`",
            "- `05_improved_design/upper_arm_v1/inertial_compare/upper_arm_v1_torque_comparison.csv`",
            "- `05_improved_design/upper_arm_v1/inertial_compare/upper_arm_v1_torque_comparison_plot.png`",
            "- `05_improved_design/upper_arm_v1/inertial_compare/upper_arm_v1_torque_comparison_report_zh.md`",
            f"- 静态姿态数量（含 zero pose）：{samples + 1}",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    out_dir = args.upper_arm_dir / "inertial_compare"
    out_dir.mkdir(parents=True, exist_ok=True)
    volume_info = read_volume_ratio(args.upper_arm_dir / "upper_arm_v1_validation.json")
    official_urdf, improved_urdf, inertial_info = prepare_scaled_inertial_urdf(
        args.project_root, args.upper_arm_dir, volume_info["volume_ratio"]
    )
    comparison = run_comparison(official_urdf, improved_urdf, args.samples, args.seed)
    rows = build_stats_rows(comparison)
    write_stats_csv(out_dir / "upper_arm_v1_torque_comparison.csv", rows)
    plot_comparison(out_dir / "upper_arm_v1_torque_comparison_plot.png", rows)
    write_report(out_dir / "upper_arm_v1_torque_comparison_report_zh.md", volume_info, inertial_info, rows, args.samples)
    (out_dir / "upper_arm_v1_inertial_update.json").write_text(
        json.dumps({"volume_info": volume_info, "inertial_info": inertial_info}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"Saved improved inertial URDF: {improved_urdf}")
    print(f"Saved torque comparison report: {out_dir / 'upper_arm_v1_torque_comparison_report_zh.md'}")
    print(f"Saved torque comparison plot: {out_dir / 'upper_arm_v1_torque_comparison_plot.png'}")
    print(f"scaled mass kg: {inertial_info['scaled_upper_arm_link_mass_kg']:.6f}")
    report_rows = [row for row in rows if abs(float(row["payload_kg"]) - REPORT_PAYLOAD_KG) < 1e-9]
    for joint in ["shoulder_lift", "elbow_flex", "wrist_flex"]:
        row = next(item for item in report_rows if item["joint"] == joint)
        print(
            f"{joint}: p95 {float(row['official_p95_abs_nm']):.4f} -> "
            f"{float(row['improved_p95_abs_nm']):.4f} Nm "
            f"({float(row['delta_pct_p95_abs_nm']):.2f}%)"
        )


if __name__ == "__main__":
    main()
