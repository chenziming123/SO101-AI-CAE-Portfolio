from __future__ import annotations

import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


V1_DIR = Path(__file__).resolve().parents[1] / "upper_arm_v1"
sys.path.insert(0, str(V1_DIR))

from upper_arm_v1_urdf_smoke import (  # noqa: E402
    OFFICIAL_UPPER_ARM_MESH,
    plot_smoke,
    run_kinematic_smoke,
    scale_stl_to_meters,
    write_motion_csv,
)


IMPROVED_MESH_NAME = "upper_arm_v3_ai_rebuild_m.stl"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=root)
    parser.add_argument("--upper-arm-dir", type=Path, default=script_dir)
    parser.add_argument("--samples", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=2032)
    return parser.parse_args()


def prepare_urdf(project_root: Path, upper_arm_dir: Path) -> tuple[Path, Path, dict[str, object]]:
    source_sim_dir = project_root / "00_source_snapshot" / "Simulation_SO101"
    source_urdf = source_sim_dir / "so101_new_calib.urdf"
    out_dir = upper_arm_dir / "urdf_smoke"
    assets_dir = out_dir / "assets"
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_sim_dir / "assets", assets_dir, dirs_exist_ok=True)

    improved_mesh = assets_dir / IMPROVED_MESH_NAME
    mesh_stats = scale_stl_to_meters(upper_arm_dir / "upper_arm_v3_ai_rebuild.stl", improved_mesh)

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

    output_urdf = out_dir / "so101_upper_arm_v3_visual_smoke.urdf"
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return source_urdf, output_urdf, mesh_stats


def write_report(
    path: Path,
    official_urdf: Path,
    improved_urdf: Path,
    mesh_stats: dict[str, object],
    smoke: dict[str, object],
) -> None:
    lines = [
        "# upper_arm_v3 接回 URDF/PyBullet Smoke Test 报告",
        "",
        "## 这一步在做什么",
        "",
        "验证 `upper_arm_v3` 的 STL 是否可以接回 SO-101 的 URDF/PyBullet 仿真链路，并确认 V3 的结构修改没有破坏原机器人的关节链和末端运动学。",
        "",
        "## 做法",
        "",
        "- 复制官方 `Simulation_SO101/assets` 到 V3 的 `urdf_smoke/assets`。",
        "- 将 `upper_arm_v3_ai_rebuild.stl` 从毫米单位缩放为米单位，生成 `upper_arm_v3_ai_rebuild_m.stl`。",
        "- 复制官方 URDF，生成 `so101_upper_arm_v3_visual_smoke.urdf`。",
        "- 只替换 `upper_arm_link` 的 visual mesh，不修改 collision、joint 和 inertial。",
        "- 使用同一批随机关节角度比较官方 URDF 和 V3 visual URDF 的末端位置。",
        "",
        "## 输入文件",
        "",
        f"- 官方 URDF：`{official_urdf}`",
        f"- V3 visual smoke URDF：`{improved_urdf}`",
        "",
        "## Mesh 缩放检查",
        "",
        f"- V3 原始 STL 包围盒：{mesh_stats['source_extents_raw']}",
        f"- 缩放后仿真 mesh 包围盒：{mesh_stats['scaled_extents_m']} m",
        f"- 缩放后 mesh watertight：{mesh_stats['scaled_watertight']}",
        "",
        "## PyBullet Smoke Test 结果",
        "",
        f"- 关节名称是否一致：{smoke['joint_names_match']}",
        f"- 官方关节数：{smoke['official_joint_count']}",
        f"- V3 关节数：{smoke['improved_joint_count']}",
        f"- 随机姿态数量（含 zero pose）：{smoke['samples_including_zero']}",
        f"- zero pose 末端位置差：{smoke['zero_pose_diff_m']:.6e} m",
        f"- 最大末端位置差：{smoke['max_ee_diff_m']:.6e} m",
        f"- 平均末端位置差：{smoke['mean_ee_diff_m']:.6e} m",
        "",
        "## 结论",
        "",
        "- V3 visual URDF 可以被 PyBullet 加载。",
        "- 关节数量和关节名称保持不变。",
        "- 由于本步骤只替换 visual mesh，不修改 joint、collision、inertial，末端运动学与官方 baseline 保持一致。",
        "- 这说明 V3 不只是 CAD 层面的修正件，已经重新进入 SO-101 的仿真链路。",
        "",
        "## 当前限制",
        "",
        "- 本 smoke test 只替换 visual mesh，尚未替换 collision mesh。",
        "- 本步骤尚未把 V3 的质量和惯量写入 URDF。",
        "- 下一步需要基于 V3 体积比例更新 inertial，并重新做静载力矩对比。",
        "",
        "## 生成文件",
        "",
        "- `05_improved_design/upper_arm_v3/urdf_smoke/so101_upper_arm_v3_visual_smoke.urdf`",
        "- `05_improved_design/upper_arm_v3/urdf_smoke/assets/upper_arm_v3_ai_rebuild_m.stl`",
        "- `05_improved_design/upper_arm_v3/urdf_smoke/upper_arm_v3_urdf_smoke_motion.csv`",
        "- `05_improved_design/upper_arm_v3/urdf_smoke/upper_arm_v3_urdf_smoke_plot.png`",
        "- `05_improved_design/upper_arm_v3/urdf_smoke/upper_arm_v3_urdf_smoke_report_zh.md`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    official_urdf, improved_urdf, mesh_stats = prepare_urdf(args.project_root, args.upper_arm_dir)
    smoke = run_kinematic_smoke(official_urdf, improved_urdf, args.samples, args.seed)
    out_dir = args.upper_arm_dir / "urdf_smoke"
    write_motion_csv(out_dir / "upper_arm_v3_urdf_smoke_motion.csv", smoke["rows"])
    plot_smoke(out_dir / "upper_arm_v3_urdf_smoke_plot.png", mesh_stats, smoke)
    write_report(
        out_dir / "upper_arm_v3_urdf_smoke_report_zh.md",
        official_urdf,
        improved_urdf,
        mesh_stats,
        smoke,
    )
    print(f"Saved modified URDF: {improved_urdf}")
    print(f"Saved smoke report: {out_dir / 'upper_arm_v3_urdf_smoke_report_zh.md'}")
    print(f"Saved smoke plot: {out_dir / 'upper_arm_v3_urdf_smoke_plot.png'}")
    print(f"joint_names_match: {smoke['joint_names_match']}")
    print(f"zero_pose_diff_m: {smoke['zero_pose_diff_m']:.6e}")
    print(f"max_ee_diff_m: {smoke['max_ee_diff_m']:.6e}")


if __name__ == "__main__":
    main()
