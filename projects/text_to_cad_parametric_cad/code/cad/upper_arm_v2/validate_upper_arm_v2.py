from __future__ import annotations

import json
from pathlib import Path

import trimesh


def mesh_stats(path: Path) -> dict:
    mesh = trimesh.load_mesh(path, force="mesh")
    bbox = mesh.bounds
    extents = mesh.extents
    volume = abs(float(mesh.volume))
    density_g_per_mm3 = 1.24 / 1000.0
    return {
        "path": str(path),
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "bbox_min": bbox[0].round(4).tolist(),
        "bbox_max": bbox[1].round(4).tolist(),
        "extents": extents.round(4).tolist(),
        "surface_area": float(mesh.area),
        "volume": volume,
        "estimated_mass_g_pla": volume * density_g_per_mm3,
        "watertight": bool(mesh.is_watertight),
    }


def pct(delta: float, baseline: float) -> float:
    return delta / baseline * 100.0 if baseline else 0.0


def main() -> None:
    project_root = Path(__file__).resolve().parents[2]
    out_dir = Path(__file__).resolve().parent
    official = project_root / "00_source_snapshot" / "STL_SO101" / "Individual" / "Upper_arm_SO101.stl"
    v1 = project_root / "05_improved_design" / "upper_arm_v1" / "upper_arm_v1_ai_rebuild.stl"
    v2 = out_dir / "upper_arm_v2_ai_rebuild.stl"
    v2_step = out_dir / "upper_arm_v2_ai_rebuild.step"
    report = out_dir / "upper_arm_v2_validation_zh.md"
    summary_json = out_dir / "upper_arm_v2_validation.json"

    meshes = {
        "official_upper_arm": mesh_stats(official),
        "upper_arm_v1": mesh_stats(v1),
        "upper_arm_v2": mesh_stats(v2),
    }
    official_stats = meshes["official_upper_arm"]
    v1_stats = meshes["upper_arm_v1"]
    v2_stats = meshes["upper_arm_v2"]
    v2_vs_official_volume_delta = v2_stats["volume"] - official_stats["volume"]
    v2_vs_v1_volume_delta = v2_stats["volume"] - v1_stats["volume"]

    lines = [
        "# upper_arm_link V2 几何验证报告",
        "",
        "## 目的",
        "",
        "检查 `upper_arm_v2` 是否成功导出为 STEP/STL，并与官方 upper arm、V1 做基础网格几何对比。",
        "",
        "V2 的设计重点是补充装配孔系，因此质量可能相对 V1 略有变化；这一步只做几何与体积筛查，孔位由单独的 mounting check 报告判断。",
        "",
        "## 文件",
        "",
        f"- 官方 STL：`{official}`",
        f"- V1 STL：`{v1}`",
        f"- V2 STL：`{v2}`",
        f"- V2 STEP：`{v2_step}`",
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
            f"- V2 相对官方 STL 体积变化：{v2_vs_official_volume_delta:.2f} mm^3，比例 {pct(v2_vs_official_volume_delta, official_stats['volume']):.2f}%。",
            f"- V2 相对 V1 体积变化：{v2_vs_v1_volume_delta:.2f} mm^3，比例 {pct(v2_vs_v1_volume_delta, v1_stats['volume']):.2f}%。",
            "- 若 V2 比 V1 略重，这是为了加入舵机安装孔周边加厚 pad 与定位孔结构，属于装配可靠性换取质量的取舍。",
            "- 是否真正补上目标孔系，需要看 `mounting_check/upper_arm_v2_mounting_check_report_zh.md`。",
        ]
    )
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    summary_json.write_text(json.dumps(meshes, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(report)
    print(summary_json)
    print(json.dumps(meshes, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
