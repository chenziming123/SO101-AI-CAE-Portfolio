from __future__ import annotations

import json
import sys
from pathlib import Path


V2_DIR = Path(__file__).resolve().parents[1] / "upper_arm_v2"
sys.path.insert(0, str(V2_DIR))

from validate_upper_arm_v2 import mesh_stats, pct  # noqa: E402


def main() -> None:
    project_root = Path(__file__).resolve().parents[2]
    out_dir = Path(__file__).resolve().parent
    official = project_root / "00_source_snapshot" / "STL_SO101" / "Individual" / "Upper_arm_SO101.stl"
    v1 = project_root / "05_improved_design" / "upper_arm_v1" / "upper_arm_v1_ai_rebuild.stl"
    v2 = project_root / "05_improved_design" / "upper_arm_v2" / "upper_arm_v2_ai_rebuild.stl"
    v3 = out_dir / "upper_arm_v3_ai_rebuild.stl"
    v3_step = out_dir / "upper_arm_v3_ai_rebuild.step"
    report = out_dir / "upper_arm_v3_validation_zh.md"
    summary_json = out_dir / "upper_arm_v3_validation.json"

    meshes = {
        "official_upper_arm": mesh_stats(official),
        "upper_arm_v1": mesh_stats(v1),
        "upper_arm_v2": mesh_stats(v2),
        "upper_arm_v3": mesh_stats(v3),
    }
    official_stats = meshes["official_upper_arm"]
    v2_stats = meshes["upper_arm_v2"]
    v3_stats = meshes["upper_arm_v3"]
    v3_vs_official_volume_delta = v3_stats["volume"] - official_stats["volume"]
    v3_vs_v2_volume_delta = v3_stats["volume"] - v2_stats["volume"]

    lines = [
        "# upper_arm_link V3 几何验证报告",
        "",
        "## 目的",
        "",
        "检查 `upper_arm_v3` 是否成功导出为 STEP/STL，并与官方 upper arm、V1、V2 做基础网格几何对比。",
        "",
        "V3 的目标不是重新设计整根 upper arm，而是针对 Step 14 暴露的肘部孔系余量问题做局部修复。",
        "",
        "## 文件",
        "",
        f"- 官方 STL：`{official}`",
        f"- V1 STL：`{v1}`",
        f"- V2 STL：`{v2}`",
        f"- V3 STL：`{v3}`",
        f"- V3 STEP：`{v3_step}`",
        "",
        "## 基础几何统计",
        "",
        "| 模型 | 顶点数 | 面数 | watertight | 包围盒尺寸 xyz | 体积 | PLA 估算质量 |",
        "|---|---:|---:|---|---:|---:|---:|",
    ]
    for label, item in meshes.items():
        extents = item["extents"]
        lines.append(
            f"| {label} | {item['vertices']} | {item['faces']} | {item['watertight']} | "
            f"{extents[0]:.2f}, {extents[1]:.2f}, {extents[2]:.2f} mm | "
            f"{item['volume']:.2f} mm^3 | {item['estimated_mass_g_pla']:.2f} g |"
        )

    lines.extend(
        [
            "",
            "## 对比结论",
            "",
            f"- V3 相对官方 STL 体积变化：{v3_vs_official_volume_delta:.2f} mm^3，比例 {pct(v3_vs_official_volume_delta, official_stats['volume']):.2f}%。",
            f"- V3 相对 V2 体积变化：{v3_vs_v2_volume_delta:.2f} mm^3，比例 {pct(v3_vs_v2_volume_delta, v2_stats['volume']):.2f}%。",
            "- V3 应结合 `assembly_check/upper_arm_v3_standard_part_check_report_zh.md` 判断是否真正修复了 V2 的装配余量风险。",
        ]
    )
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    summary_json.write_text(json.dumps(meshes, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(report)
    print(summary_json)
    print(json.dumps(meshes, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
