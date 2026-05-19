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


def main() -> None:
    project_root = Path(__file__).resolve().parents[2]
    out_dir = Path(__file__).resolve().parent
    official = project_root / "00_source_snapshot" / "STL_SO101" / "Individual" / "Upper_arm_SO101.stl"
    improved = out_dir / "upper_arm_v1_ai_rebuild.stl"
    improved_step = out_dir / "upper_arm_v1_ai_rebuild.step"
    report = out_dir / "upper_arm_v1_validation_zh.md"
    summary_json = out_dir / "upper_arm_v1_validation.json"

    meshes = {
        "official_upper_arm": mesh_stats(official),
        "improved_upper_arm_v1": mesh_stats(improved),
    }

    official_stats = meshes["official_upper_arm"]
    improved_stats = meshes["improved_upper_arm_v1"]
    volume_delta = improved_stats["volume"] - official_stats["volume"]
    volume_delta_pct = volume_delta / official_stats["volume"] * 100.0 if official_stats["volume"] else 0.0
    span_mm = improved_stats["extents"][0]

    lines = [
        "# upper_arm_link V1 几何验证报告",
        "",
        "## 目的",
        "",
        "检查 AI 辅助生成的 `upper_arm_v1` 是否成功导出为 STEP/STL，并与官方 upper arm STL 做基础几何对比。",
        "",
        "## 文件",
        "",
        f"- 官方 STL：`{official}`",
        f"- 改进版 STL：`{improved}`",
        f"- 改进版 STEP：`{improved_step}`",
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
            f"- 改进版 X 向总包围盒长度约 {span_mm:.2f} mm；设计中保留的 shoulder 到 elbow 轴线间距为 116.0 mm。",
            f"- 与官方 STL 相比，改进版体积变化约 {volume_delta:.2f} mm^3，比例约 {volume_delta_pct:.1f}%。",
            "- V1 是参数化概念重建件，重点是建立可编辑 CAD 源文件和结构改进方向，不直接声称已经满足装机打印要求。",
            "- 下一步需要可视化检查外观，并准备把改进 mesh 接回 URDF/PyBullet 做加载测试。",
        ]
    )
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    summary_json.write_text(json.dumps(meshes, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(report)
    print(summary_json)
    print(json.dumps(meshes, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
