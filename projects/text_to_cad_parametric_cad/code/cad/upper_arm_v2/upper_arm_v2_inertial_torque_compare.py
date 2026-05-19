from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


V1_DIR = Path(__file__).resolve().parents[1] / "upper_arm_v1"
sys.path.insert(0, str(V1_DIR))

from upper_arm_v1_inertial_torque_compare import (  # noqa: E402
    ARM_JOINT_NAMES,
    INERTIA_FIELDS,
    REPORT_PAYLOAD_KG,
    build_stats_rows,
    float_text,
    run_comparison,
    write_stats_csv,
)


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[2]
    upper_arm_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=project_root)
    parser.add_argument("--upper-arm-dir", type=Path, default=upper_arm_dir)
    parser.add_argument("--samples", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=2031)
    return parser.parse_args()


def read_volume_info(validation_json: Path) -> dict[str, float]:
    data = json.loads(validation_json.read_text(encoding="utf-8"))
    official = data["official_upper_arm"]
    v1 = data["upper_arm_v1"]
    v2 = data["upper_arm_v2"]
    official_volume = float(official["volume"])
    v1_volume = float(v1["volume"])
    v2_volume = float(v2["volume"])
    if min(official_volume, v1_volume, v2_volume) <= 0:
        raise RuntimeError("Invalid mesh volume in validation JSON")
    return {
        "official_volume_mm3": official_volume,
        "v1_volume_mm3": v1_volume,
        "v2_volume_mm3": v2_volume,
        "v1_volume_ratio": v1_volume / official_volume,
        "v2_volume_ratio": v2_volume / official_volume,
        "v2_vs_v1_volume_ratio": v2_volume / v1_volume,
        "official_estimated_mass_g_pla": float(official["estimated_mass_g_pla"]),
        "v1_estimated_mass_g_pla": float(v1["estimated_mass_g_pla"]),
        "v2_estimated_mass_g_pla": float(v2["estimated_mass_g_pla"]),
    }


def prepare_scaled_inertial_urdf(
    project_root: Path,
    upper_arm_dir: Path,
    volume_ratio: float,
) -> tuple[Path, Path, dict[str, object]]:
    official_urdf = project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf"
    visual_smoke_dir = upper_arm_dir / "urdf_smoke"
    visual_smoke_urdf = visual_smoke_dir / "so101_upper_arm_v2_visual_smoke.urdf"
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

    output_urdf = out_dir / "so101_upper_arm_v2_inertial_scaled.urdf"
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return official_urdf, output_urdf, {
        "official_upper_arm_link_mass_kg": official_mass,
        "scaled_upper_arm_link_mass_kg": scaled_mass,
        "mass_delta_kg": scaled_mass - official_mass,
        "mass_delta_pct": (scaled_mass / official_mass - 1.0) * 100.0,
        "official_inertia": official_inertia,
        "scaled_inertia": scaled_inertia,
    }


def run_v1_same_seed_rows(project_root: Path, samples: int, seed: int) -> list[dict[str, object]]:
    official_urdf = project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf"
    v1_urdf = (
        project_root
        / "05_improved_design"
        / "upper_arm_v1"
        / "inertial_compare"
        / "so101_upper_arm_v1_inertial_scaled.urdf"
    )
    if not v1_urdf.exists():
        return []
    comparison = run_comparison(official_urdf, v1_urdf, samples, seed)
    return build_stats_rows(comparison)


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
    ax_bar.bar(x + width / 2, imp_p95, width, label="upper_arm_v2")
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


def v1_value(v1_rows: list[dict[str, object]], joint: str, key: str) -> float | None:
    for row in v1_rows:
        if abs(float(row["payload_kg"]) - REPORT_PAYLOAD_KG) < 1e-9 and row["joint"] == joint:
            return float(row[key])
    return None


def write_report(
    path: Path,
    volume_info: dict[str, float],
    inertial_info: dict[str, object],
    rows: list[dict[str, object]],
        v1_rows: list[dict[str, object]],
    samples: int,
) -> None:
    report_rows = [row for row in rows if abs(float(row["payload_kg"]) - REPORT_PAYLOAD_KG) < 1e-9]
    shoulder = next(row for row in report_rows if row["joint"] == "shoulder_lift")
    elbow = next(row for row in report_rows if row["joint"] == "elbow_flex")
    lines = [
        "# upper_arm_v2 惯量更新与静载力矩对比报告",
        "",
        "## 这一步在做什么",
        "",
        "将 `upper_arm_v2` 的体积变化转化为 URDF inertial screening 参数，并与官方 SO-101 baseline 做静载关节力矩对比。",
        "",
        "## 方法",
        "",
        "- 保留官方 URDF 作为 baseline，不覆盖 `00_source_snapshot/`。",
        "- 使用 V2 STL 与官方 upper arm STL 的体积比例，按比例缩放官方 `upper_arm_link` 的质量和惯量张量。",
        "- 生成 `so101_upper_arm_v2_inertial_scaled.urdf`。",
        "- 使用同一批随机关节姿态，对官方版和 V2 inertial 版计算静载关节力矩。",
        "- 该方法是工程筛查，不是最终真实惯量标定。",
        "",
        "## 质量与惯量更新",
        "",
        f"- 官方 upper arm STL 体积：{volume_info['official_volume_mm3']:.2f} mm^3",
        f"- V1 upper arm STL 体积：{volume_info['v1_volume_mm3']:.2f} mm^3",
        f"- V2 upper arm STL 体积：{volume_info['v2_volume_mm3']:.2f} mm^3",
        f"- V1 / 官方体积比例：{volume_info['v1_volume_ratio']:.4f}",
        f"- V2 / 官方体积比例：{volume_info['v2_volume_ratio']:.4f}",
        f"- V2 / V1 体积比例：{volume_info['v2_vs_v1_volume_ratio']:.4f}",
        f"- 官方 URDF `upper_arm_link` 质量：{float(inertial_info['official_upper_arm_link_mass_kg']):.6f} kg",
        f"- V2 screening 质量：{float(inertial_info['scaled_upper_arm_link_mass_kg']):.6f} kg",
        f"- V2 相对官方质量变化：{float(inertial_info['mass_delta_kg']):.6f} kg ({float(inertial_info['mass_delta_pct']):.2f}%)",
        "",
        "说明：官方 URDF link 质量和 STL 体积估算质量并不完全一致。因此本步骤采用“URDF 质量按 STL 体积比例缩放”的筛查方式，目的是观察趋势，不作为最终物理参数。",
        "",
        f"## {REPORT_PAYLOAD_KG:.2f} kg Payload 下的 p95 静载力矩对比",
        "",
        "| 关节 | 官方 p95 Nm | V2 p95 Nm | 变化 Nm | 变化比例 |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in report_rows:
        lines.append(
            f"| {row['joint']} | {float(row['official_p95_abs_nm']):.4f} | "
            f"{float(row['improved_p95_abs_nm']):.4f} | "
            f"{float(row['delta_p95_abs_nm']):.4f} | "
            f"{float(row['delta_pct_p95_abs_nm']):.2f}% |"
        )

    if v1_rows:
        lines.extend(
            [
                "",
                "## Official / V1 / V2 三版本对比",
                "",
                "| 关节 | 官方 p95 Nm | V1 p95 Nm | V2 p95 Nm | V2 相对 V1 变化 Nm |",
                "|---|---:|---:|---:|---:|",
            ]
        )
        for row in report_rows:
            joint = str(row["joint"])
            v1_p95 = v1_value(v1_rows, joint, "improved_p95_abs_nm")
            if v1_p95 is None:
                continue
            v2_p95 = float(row["improved_p95_abs_nm"])
            lines.append(
                f"| {joint} | {float(row['official_p95_abs_nm']):.4f} | "
                f"{v1_p95:.4f} | {v2_p95:.4f} | {v2_p95 - v1_p95:.4f} |"
            )

    lines.extend(
        [
            "",
            "## 主要结论",
            "",
            f"- `shoulder_lift` p95 力矩：{float(shoulder['official_p95_abs_nm']):.4f} -> {float(shoulder['improved_p95_abs_nm']):.4f} Nm，变化 {float(shoulder['delta_p95_abs_nm']):.4f} Nm ({float(shoulder['delta_pct_p95_abs_nm']):.2f}%)。",
            f"- `elbow_flex` p95 力矩：{float(elbow['official_p95_abs_nm']):.4f} -> {float(elbow['improved_p95_abs_nm']):.4f} Nm，变化 {float(elbow['delta_p95_abs_nm']):.4f} Nm ({float(elbow['delta_pct_p95_abs_nm']):.2f}%)。",
            "- V2 比 V1 略重，因为补充了装配孔、沉孔和局部加强 pad；它的意义不是追求最轻，而是在保持明显减重的同时提升装配表达能力。",
            "- 结果仍符合机械直觉：upper arm 的质量变化主要影响 `shoulder_lift`，对下游 `elbow_flex` 和 `wrist_flex` 影响很小。",
            "",
            "## 当前限制",
            "",
            "- 当前 inertia 是按体积比例缩放的筛查值，不是从 CAD 质量属性直接导出的最终惯量。",
            "- Official / V1 / V2 三版本对比使用同一批随机关节姿态重新计算，避免不同随机采样导致的 p95 细微偏差。",
            "- 尚未替换 collision mesh，也尚未引入舵机、轴承、螺丝等标准件做完整装配干涉检查。",
            "- 下一步建议整理 official/V1/V2 总对比报告，或者继续做标准件装配与干涉检查。",
            "",
            "## 生成文件",
            "",
            "- `05_improved_design/upper_arm_v2/inertial_compare/so101_upper_arm_v2_inertial_scaled.urdf`",
            "- `05_improved_design/upper_arm_v2/inertial_compare/upper_arm_v2_torque_comparison.csv`",
            "- `05_improved_design/upper_arm_v2/inertial_compare/upper_arm_v2_torque_comparison_plot.png`",
            "- `05_improved_design/upper_arm_v2/inertial_compare/upper_arm_v2_torque_comparison_report_zh.md`",
            f"- 静态姿态数量（含 zero pose）：{samples + 1}",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    out_dir = args.upper_arm_dir / "inertial_compare"
    out_dir.mkdir(parents=True, exist_ok=True)
    volume_info = read_volume_info(args.upper_arm_dir / "upper_arm_v2_validation.json")
    official_urdf, improved_urdf, inertial_info = prepare_scaled_inertial_urdf(
        args.project_root, args.upper_arm_dir, volume_info["v2_volume_ratio"]
    )
    comparison = run_comparison(official_urdf, improved_urdf, args.samples, args.seed)
    rows = build_stats_rows(comparison)
    v1_rows = run_v1_same_seed_rows(args.project_root, args.samples, args.seed)
    write_stats_csv(out_dir / "upper_arm_v2_torque_comparison.csv", rows)
    if v1_rows:
        write_stats_csv(out_dir / "upper_arm_v1_same_seed_torque_comparison.csv", v1_rows)
    plot_comparison(out_dir / "upper_arm_v2_torque_comparison_plot.png", rows)
    write_report(
        out_dir / "upper_arm_v2_torque_comparison_report_zh.md",
        volume_info,
        inertial_info,
        rows,
        v1_rows,
        args.samples,
    )
    (out_dir / "upper_arm_v2_inertial_update.json").write_text(
        json.dumps({"volume_info": volume_info, "inertial_info": inertial_info}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"Saved improved inertial URDF: {improved_urdf}")
    print(f"Saved torque comparison report: {out_dir / 'upper_arm_v2_torque_comparison_report_zh.md'}")
    print(f"Saved torque comparison plot: {out_dir / 'upper_arm_v2_torque_comparison_plot.png'}")
    print(f"scaled mass kg: {float(inertial_info['scaled_upper_arm_link_mass_kg']):.6f}")
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
