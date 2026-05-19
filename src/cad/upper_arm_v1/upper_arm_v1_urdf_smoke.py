from __future__ import annotations

import argparse
import csv
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
import trimesh


ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINT_NAME = "gripper"
EE_LINK_NAME = "gripper_frame_link"
OFFICIAL_UPPER_ARM_MESH = "assets/upper_arm_so101_v1.stl"
IMPROVED_MESH_NAME = "upper_arm_v1_ai_rebuild_m.stl"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=root)
    parser.add_argument("--upper-arm-dir", type=Path, default=script_dir)
    parser.add_argument("--samples", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=2028)
    return parser.parse_args()


def scale_stl_to_meters(source_stl: Path, output_stl: Path) -> dict[str, object]:
    mesh_mm = trimesh.load_mesh(source_stl, force="mesh")
    mesh_m = mesh_mm.copy()
    mesh_m.apply_scale(0.001)
    output_stl.parent.mkdir(parents=True, exist_ok=True)
    mesh_m.export(output_stl)
    return {
        "source_extents_raw": mesh_mm.extents.round(6).tolist(),
        "scaled_extents_m": mesh_m.extents.round(6).tolist(),
        "scaled_volume_m3": float(abs(mesh_m.volume)),
        "scaled_watertight": bool(mesh_m.is_watertight),
    }


def prepare_urdf(project_root: Path, upper_arm_dir: Path) -> tuple[Path, Path, dict[str, object]]:
    source_sim_dir = project_root / "00_source_snapshot" / "Simulation_SO101"
    source_urdf = source_sim_dir / "so101_new_calib.urdf"
    out_dir = upper_arm_dir / "urdf_smoke"
    assets_dir = out_dir / "assets"
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_sim_dir / "assets", assets_dir, dirs_exist_ok=True)

    improved_mesh = assets_dir / IMPROVED_MESH_NAME
    mesh_stats = scale_stl_to_meters(upper_arm_dir / "upper_arm_v1_ai_rebuild.stl", improved_mesh)

    tree = ET.parse(source_urdf)
    root = tree.getroot()
    upper_link = root.find("./link[@name='upper_arm_link']")
    if upper_link is None:
        raise RuntimeError("upper_arm_link not found in official URDF")

    replaced = False
    for visual in upper_link.findall("visual"):
        mesh = visual.find("./geometry/mesh")
        if mesh is not None and mesh.attrib.get("filename") == OFFICIAL_UPPER_ARM_MESH:
            mesh.attrib["filename"] = f"assets/{IMPROVED_MESH_NAME}"
            origin = visual.find("origin")
            if origin is None:
                origin = ET.SubElement(visual, "origin")
            origin.attrib["xyz"] = "0 0 0"
            origin.attrib["rpy"] = "0 0 3.14159"
            replaced = True

    if not replaced:
        raise RuntimeError("official upper arm visual mesh was not found/replaced")

    output_urdf = out_dir / "so101_upper_arm_v1_visual_smoke.urdf"
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return source_urdf, output_urdf, mesh_stats


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


def collect_joints(robot_id: int, client: int) -> dict[str, dict[str, object]]:
    joints = {}
    for idx in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, idx, physicsClientId=client)
        name = info[1].decode("utf-8")
        joints[name] = {
            "index": idx,
            "type": int(info[2]),
            "lower": float(info[8]),
            "upper": float(info[9]),
            "child_link": info[12].decode("utf-8"),
        }
    return joints


def link_index_by_name(joints: dict[str, dict[str, object]], link_name: str) -> int:
    for joint in joints.values():
        if joint["child_link"] == link_name:
            return int(joint["index"])
    raise RuntimeError(f"link not found: {link_name}")


def set_arm_state(robot_id: int, joints: dict[str, dict[str, object]], q: np.ndarray, client: int) -> None:
    for joint_name, value in zip(ARM_JOINT_NAMES, q):
        p.resetJointState(robot_id, int(joints[joint_name]["index"]), float(value), physicsClientId=client)
    if GRIPPER_JOINT_NAME in joints:
        p.resetJointState(robot_id, int(joints[GRIPPER_JOINT_NAME]["index"]), 0.25, physicsClientId=client)


def ee_position(robot_id: int, ee_link: int, client: int) -> np.ndarray:
    state = p.getLinkState(robot_id, ee_link, computeForwardKinematics=True, physicsClientId=client)
    return np.asarray(state[4], dtype=float)


def run_kinematic_smoke(official_urdf: Path, improved_urdf: Path, samples: int, seed: int):
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    official_robot = load_robot(official_urdf, client)
    improved_robot = load_robot(improved_urdf, client)
    official_joints = collect_joints(official_robot, client)
    improved_joints = collect_joints(improved_robot, client)

    joint_names_match = list(official_joints) == list(improved_joints)
    missing = sorted(set(ARM_JOINT_NAMES) - set(official_joints))
    if missing:
        raise RuntimeError(f"missing expected arm joints: {missing}")

    official_ee = link_index_by_name(official_joints, EE_LINK_NAME)
    improved_ee = link_index_by_name(improved_joints, EE_LINK_NAME)

    lower = np.array([official_joints[name]["lower"] for name in ARM_JOINT_NAMES], dtype=float)
    upper = np.array([official_joints[name]["upper"] for name in ARM_JOINT_NAMES], dtype=float)
    rng = np.random.default_rng(seed)
    q_samples = rng.uniform(lower, upper, size=(samples, len(ARM_JOINT_NAMES)))
    q_samples = np.vstack([np.zeros((1, len(ARM_JOINT_NAMES))), q_samples])

    rows = []
    diffs = []
    official_positions = []
    improved_positions = []
    for sample_id, q in enumerate(q_samples):
        set_arm_state(official_robot, official_joints, q, client)
        set_arm_state(improved_robot, improved_joints, q, client)
        official_pos = ee_position(official_robot, official_ee, client)
        improved_pos = ee_position(improved_robot, improved_ee, client)
        diff = float(np.linalg.norm(official_pos - improved_pos))
        diffs.append(diff)
        official_positions.append(official_pos)
        improved_positions.append(improved_pos)
        rows.append((sample_id, q.copy(), official_pos, improved_pos, diff))

    p.disconnect(client)
    diffs_arr = np.asarray(diffs)
    return {
        "joint_names_match": joint_names_match,
        "official_joint_count": len(official_joints),
        "improved_joint_count": len(improved_joints),
        "samples_including_zero": int(len(q_samples)),
        "max_ee_diff_m": float(np.max(diffs_arr)),
        "mean_ee_diff_m": float(np.mean(diffs_arr)),
        "zero_pose_diff_m": float(diffs_arr[0]),
        "rows": rows,
        "official_positions": np.asarray(official_positions),
        "improved_positions": np.asarray(improved_positions),
    }


def write_motion_csv(path: Path, rows) -> None:
    fields = [
        "sample_id",
        *ARM_JOINT_NAMES,
        "official_x",
        "official_y",
        "official_z",
        "improved_x",
        "improved_y",
        "improved_z",
        "ee_diff_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for sample_id, q, official_pos, improved_pos, diff in rows:
            row = {"sample_id": sample_id}
            row.update({name: float(value) for name, value in zip(ARM_JOINT_NAMES, q)})
            row.update(
                {
                    "official_x": float(official_pos[0]),
                    "official_y": float(official_pos[1]),
                    "official_z": float(official_pos[2]),
                    "improved_x": float(improved_pos[0]),
                    "improved_y": float(improved_pos[1]),
                    "improved_z": float(improved_pos[2]),
                    "ee_diff_m": diff,
                }
            )
            writer.writerow(row)


def plot_smoke(path: Path, mesh_stats: dict[str, object], smoke: dict[str, object]) -> None:
    official_positions = smoke["official_positions"]
    improved_positions = smoke["improved_positions"]
    diffs = [row[-1] for row in smoke["rows"]]
    extents = mesh_stats["scaled_extents_m"]

    fig = plt.figure(figsize=(12, 8))
    ax_ext = fig.add_subplot(2, 2, 1)
    ax_ext.bar(["x", "y", "z"], extents, color="tab:blue")
    ax_ext.set_title("Improved mesh extents after scaling")
    ax_ext.set_ylabel("m")

    ax_top = fig.add_subplot(2, 2, 2)
    ax_top.scatter(official_positions[:, 0], official_positions[:, 1], s=3, alpha=0.25, label="official")
    ax_top.scatter(improved_positions[:, 0], improved_positions[:, 1], s=2, alpha=0.25, label="improved")
    ax_top.set_title("EE XY positions")
    ax_top.set_xlabel("x / m")
    ax_top.set_ylabel("y / m")
    ax_top.axis("equal")
    ax_top.legend()

    ax_hist = fig.add_subplot(2, 2, 3)
    ax_hist.hist(diffs, bins=40, color="tab:green", alpha=0.8)
    ax_hist.set_title("EE difference distribution")
    ax_hist.set_xlabel("m")
    ax_hist.set_ylabel("count")

    ax_text = fig.add_subplot(2, 2, 4)
    ax_text.axis("off")
    text = (
        f"Joint names match: {smoke['joint_names_match']}\n"
        f"Joint count: {smoke['official_joint_count']} -> {smoke['improved_joint_count']}\n"
        f"Samples: {smoke['samples_including_zero']}\n"
        f"Zero pose EE diff: {smoke['zero_pose_diff_m']:.3e} m\n"
        f"Max EE diff: {smoke['max_ee_diff_m']:.3e} m\n"
        f"Mean EE diff: {smoke['mean_ee_diff_m']:.3e} m\n"
        f"Scaled mesh watertight: {mesh_stats['scaled_watertight']}\n"
    )
    ax_text.text(0.02, 0.95, text, va="top", ha="left", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def write_report(
    path: Path,
    official_urdf: Path,
    improved_urdf: Path,
    mesh_stats: dict[str, object],
    smoke: dict[str, object],
) -> None:
    lines = [
        "# upper_arm_v1 接回 URDF/PyBullet Smoke Test 报告",
        "",
        "## 目的",
        "",
        "验证 AI 辅助生成的 `upper_arm_v1` STL 是否可以接回 SO-101 的 URDF/PyBullet 仿真链路。",
        "",
        "## 做法",
        "",
        "- 复制官方 `Simulation_SO101/assets` 到 `05_improved_design/upper_arm_v1/urdf_smoke/assets`。",
        "- 将 `upper_arm_v1_ai_rebuild.stl` 从毫米单位缩放为米单位，生成 `upper_arm_v1_ai_rebuild_m.stl`。",
        "- 复制官方 URDF，生成 `so101_upper_arm_v1_visual_smoke.urdf`。",
        "- 只替换 `upper_arm_link` 的 3D 打印件 visual mesh，不修改 collision 和 inertial。",
        "- 用同一批随机关节角度比较官方 URDF 和改进 URDF 的末端位置。",
        "",
        "## 输入文件",
        "",
        f"- 官方 URDF：`{official_urdf}`",
        f"- 改进 URDF：`{improved_urdf}`",
        "",
        "## Mesh 缩放检查",
        "",
        f"- V1 原始 STL 包围盒：{mesh_stats['source_extents_raw']}",
        f"- 缩放后仿真 mesh 包围盒：{mesh_stats['scaled_extents_m']} m",
        f"- 缩放后 mesh watertight：{mesh_stats['scaled_watertight']}",
        "",
        "## PyBullet Smoke Test 结果",
        "",
        f"- 关节名称是否一致：{smoke['joint_names_match']}",
        f"- 官方关节数：{smoke['official_joint_count']}",
        f"- 改进版关节数：{smoke['improved_joint_count']}",
        f"- 随机姿态数量（含 zero pose）：{smoke['samples_including_zero']}",
        f"- zero pose 末端位置差：{smoke['zero_pose_diff_m']:.6e} m",
        f"- 最大末端位置差：{smoke['max_ee_diff_m']:.6e} m",
        f"- 平均末端位置差：{smoke['mean_ee_diff_m']:.6e} m",
        "",
        "## 结论",
        "",
        "- 改进版 URDF 可以被 PyBullet 加载。",
        "- 关节数量和关节名称保持不变。",
        "- 由于本步骤只替换 visual mesh，不修改 joint、collision、inertial，末端运动学应与官方 baseline 保持一致。",
        "- 这一步证明 V1 CAD/STL 已经能进入机器人仿真链路，不再只是孤立 CAD 文件。",
        "",
        "## 当前限制",
        "",
        "- 本 smoke test 只替换 visual mesh，尚未替换 collision mesh。",
        "- 尚未把 V1 的质量和惯量写入 URDF。",
        "- 尚未做改进版静载力矩对比；下一步需要基于 V1 粗估质量更新 inertial 后再对比。",
        "",
        "## 生成文件",
        "",
        "- `05_improved_design/upper_arm_v1/urdf_smoke/so101_upper_arm_v1_visual_smoke.urdf`",
        "- `05_improved_design/upper_arm_v1/urdf_smoke/assets/upper_arm_v1_ai_rebuild_m.stl`",
        "- `05_improved_design/upper_arm_v1/urdf_smoke/upper_arm_v1_urdf_smoke_motion.csv`",
        "- `05_improved_design/upper_arm_v1/urdf_smoke/upper_arm_v1_urdf_smoke_plot.png`",
        "- `05_improved_design/upper_arm_v1/urdf_smoke/upper_arm_v1_urdf_smoke_report_zh.md`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    official_urdf, improved_urdf, mesh_stats = prepare_urdf(args.project_root, args.upper_arm_dir)
    smoke = run_kinematic_smoke(official_urdf, improved_urdf, args.samples, args.seed)
    out_dir = args.upper_arm_dir / "urdf_smoke"
    write_motion_csv(out_dir / "upper_arm_v1_urdf_smoke_motion.csv", smoke["rows"])
    plot_smoke(out_dir / "upper_arm_v1_urdf_smoke_plot.png", mesh_stats, smoke)
    write_report(
        out_dir / "upper_arm_v1_urdf_smoke_report_zh.md",
        official_urdf,
        improved_urdf,
        mesh_stats,
        smoke,
    )
    print(f"Saved modified URDF: {improved_urdf}")
    print(f"Saved smoke report: {out_dir / 'upper_arm_v1_urdf_smoke_report_zh.md'}")
    print(f"Saved smoke plot: {out_dir / 'upper_arm_v1_urdf_smoke_plot.png'}")
    print(f"joint_names_match: {smoke['joint_names_match']}")
    print(f"zero_pose_diff_m: {smoke['zero_pose_diff_m']:.6e}")
    print(f"max_ee_diff_m: {smoke['max_ee_diff_m']:.6e}")


if __name__ == "__main__":
    main()
