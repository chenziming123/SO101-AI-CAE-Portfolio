#!/usr/bin/env python3
"""Step 20: full V3 CAE-clean FEA with refined mesh.

The previous 7 mm mesh was too coarse for V3's small M3 holes and counterbores.
This script uses a 5 mm tetrahedral mesh so the full V3 feature set can be
included in the FEA model.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import run_upper_arm_static_fea as base


PROJECT_ROOT = base.PROJECT_ROOT
OUT_DIR = (
    PROJECT_ROOT
    / "07_fea_analysis"
    / "upper_arm_static_fea"
    / "results_full_v3_clean"
)
TMP_DIR = OUT_DIR / "tmp"


@dataclass
class CleanCaseResult:
    name: str
    source: str
    geometry_type: str
    mesh_size_mm: float
    node_count: int
    tet_count: int
    span_axis: str
    bbox_extent_mm: str
    mesh_volume_mm3: float
    mass_g: float
    fixed_node_count: int
    loaded_node_count: int
    max_disp_mm: float
    mean_loaded_disp_mm: float
    max_von_mises_mpa: float
    p95_von_mises_mpa: float
    safety_factor_yield: float
    vtk_path: str


def mesh_official(mesh_size_mm: float) -> tuple[Path, str]:
    official_step = base.candidate_path(
        [
            PROJECT_ROOT / "00_source_snapshot/STEP_SO101/Upper_arm_SO101.step",
            PROJECT_ROOT / "00_source_snapshot/STEP_SO101/Upper_arm_SO101.stp",
        ]
    )
    msh_path = TMP_DIR / "official_upper_arm_clean_5mm.msh"
    base.gmsh_mesh_geometry(official_step, msh_path, mesh_size_mm)
    return msh_path, str(official_step)


def mesh_full_v3(mesh_size_mm: float) -> tuple[Path, str]:
    msh_path = TMP_DIR / "v3_full_upper_arm_clean_5mm.msh"
    step_path = OUT_DIR / "v3_full_cae_clean_5mm.step"
    base.gmsh_mesh_v3_parametric(
        msh_path,
        step_path,
        mesh_size_mm=mesh_size_mm,
        include_cable_channel=True,
        include_lightening_holes=True,
        include_mount_holes=True,
        include_counterbores=True,
        cut_strategy="fused",
        heal_after_cut=False,
    )
    return msh_path, str(step_path)


def solve_case(
    name: str,
    source: str,
    geometry_type: str,
    mesh_path: Path,
    mesh_size_mm: float,
    material: base.Material,
    loadcase: base.LoadCase,
) -> tuple[CleanCaseResult, dict]:
    points, tetra = base.read_tet_mesh(mesh_path)
    displacement, vm, volume_mm3, fixed_nodes, loaded_nodes, span_axis = base.solve_static_case(
        name, points, tetra, material, loadcase
    )
    disp_mag = np.linalg.norm(displacement, axis=1)
    vtk_path = OUT_DIR / f"{name}_upper_arm_full_v3_clean_fea.vtu"
    base.save_vtk(vtk_path, points, tetra, displacement, vm)
    faces, owners = base.boundary_faces(tetra)
    plot_payload = {
        "points": points,
        "tetra": tetra,
        "faces": faces,
        "face_owners": owners,
        "span_axis": span_axis,
        "von_mises_mpa": vm,
        "disp_by_elem_mm": disp_mag[tetra].mean(axis=1),
    }
    extents = np.ptp(points, axis=0)
    result = CleanCaseResult(
        name=name,
        source=source,
        geometry_type=geometry_type,
        mesh_size_mm=mesh_size_mm,
        node_count=int(points.shape[0]),
        tet_count=int(tetra.shape[0]),
        span_axis="xyz"[span_axis],
        bbox_extent_mm=",".join(f"{v:.3f}" for v in extents),
        mesh_volume_mm3=float(volume_mm3),
        mass_g=float(volume_mm3 / 1000.0 * material.density_g_per_cm3),
        fixed_node_count=int(fixed_nodes.size),
        loaded_node_count=int(loaded_nodes.size),
        max_disp_mm=float(disp_mag.max()),
        mean_loaded_disp_mm=float(disp_mag[loaded_nodes].mean()),
        max_von_mises_mpa=float(vm.max()),
        p95_von_mises_mpa=float(np.percentile(vm, 95.0)),
        safety_factor_yield=float(material.yield_mpa / max(vm.max(), 1e-12)),
        vtk_path=str(vtk_path),
    )
    return result, plot_payload


def run() -> None:
    mesh_size_mm = 5.0
    material = base.Material()
    loadcase = base.LoadCase(mesh_size_mm=mesh_size_mm)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    TMP_DIR.mkdir(parents=True, exist_ok=True)

    official_mesh, official_source = mesh_official(mesh_size_mm)
    v3_mesh, v3_source = mesh_full_v3(mesh_size_mm)

    official, official_plot = solve_case(
        "official",
        official_source,
        "step_refined_5mm",
        official_mesh,
        mesh_size_mm,
        material,
        loadcase,
    )
    v3, v3_plot = solve_case(
        "v3_full",
        v3_source,
        "full_v3_cae_clean_5mm",
        v3_mesh,
        mesh_size_mm,
        material,
        loadcase,
    )
    results = [official, v3]
    plot_payload = {"official": official_plot, "v3_full": v3_plot}

    metrics_json = OUT_DIR / "upper_arm_full_v3_clean_fea_metrics.json"
    metrics_csv = OUT_DIR / "upper_arm_full_v3_clean_fea_metrics.csv"
    with metrics_json.open("w", encoding="utf-8") as f:
        json.dump(
            {
                "material": material.__dict__,
                "loadcase": loadcase.__dict__,
                "mesh_resolution_note": (
                    "5 mm target mesh is required to include V3 small mounting holes "
                    "and counterbores without overlapping boundary facets."
                ),
                "results": [r.__dict__ for r in results],
            },
            f,
            indent=2,
            ensure_ascii=False,
        )
    with metrics_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].__dict__.keys()))
        writer.writeheader()
        writer.writerows(r.__dict__ for r in results)

    base.plot_case_surfaces(
        plot_payload,
        "von_mises_mpa",
        OUT_DIR / "upper_arm_full_v3_clean_fea_von_mises.png",
        "full V3 clean mesh von Mises stress / MPa",
        "inferno",
    )
    base.plot_case_surfaces(
        plot_payload,
        "disp_by_elem_mm",
        OUT_DIR / "upper_arm_full_v3_clean_fea_displacement.png",
        "full V3 clean mesh displacement magnitude / mm",
        "viridis",
    )

    mass_change = (v3.mass_g / official.mass_g - 1.0) * 100.0
    disp_change = (v3.max_disp_mm / official.max_disp_mm - 1.0) * 100.0
    stress_change = (v3.max_von_mises_mpa / official.max_von_mises_mpa - 1.0) * 100.0

    report = OUT_DIR / "upper_arm_full_v3_clean_fea_report_zh.md"
    report.write_text(
        f"""# SO-101 upper_arm_link full V3 CAE 几何清理与 FEA 报告

## 这一步在做什么

Step 20 针对 full V3 在 7 mm 网格下出现的 `overlapping facets` 问题进行 CAE 几何清理验证。诊断发现，小装配孔和沉孔不是不能做 FEA，而是 7 mm 目标网格相对 M3 通孔、沉孔和夹紧孔过粗。将目标四面体网格尺寸降到 {mesh_size_mm:.1f} mm 后，full V3 带完整小孔和沉孔可以成功网格化。

## 工况设定

- official：官方 `Upper_arm_SO101.step`，重新用 {mesh_size_mm:.1f} mm 网格划分。
- V3：Gmsh/OpenCASCADE 参数化重建的 full V3 CAE 几何，包含肩部夹紧孔、肘部安装通孔和沉孔。
- 材料：{material.name}，弹性模量 {material.young_mpa:.0f} MPa，泊松比 {material.poisson:.2f}，屈服强度按 {material.yield_mpa:.0f} MPa 做 screening。
- 约束/载荷：沿零件最长方向，shoulder 侧前 {loadcase.fixed_x_fraction:.0%} 固定，elbow 侧后 {loadcase.loaded_x_fraction:.0%} 施加 {loadcase.total_force_n:.1f} N 向下力。

## 核心结果

| 版本 | 几何 | 网格尺寸 mm | 节点数 | 四面体数 | 质量 g | 最大位移 mm | 最大 von Mises MPa | p95 von Mises MPa | 屈服安全系数 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| official | {official.geometry_type} | {official.mesh_size_mm:.1f} | {official.node_count} | {official.tet_count} | {official.mass_g:.2f} | {official.max_disp_mm:.6f} | {official.max_von_mises_mpa:.3f} | {official.p95_von_mises_mpa:.3f} | {official.safety_factor_yield:.2f} |
| full V3 | {v3.geometry_type} | {v3.mesh_size_mm:.1f} | {v3.node_count} | {v3.tet_count} | {v3.mass_g:.2f} | {v3.max_disp_mm:.6f} | {v3.max_von_mises_mpa:.3f} | {v3.p95_von_mises_mpa:.3f} | {v3.safety_factor_yield:.2f} |

## 对比结论

- full V3 相对 official 的估算质量变化：{mass_change:+.2f}%。
- full V3 相对 official 的最大位移变化：{disp_change:+.2f}%。
- full V3 相对 official 的最大 von Mises 应力变化：{stress_change:+.2f}%。
- full V3 的 CAE 网格问题已初步解决：关键不是删除小孔，而是对小孔/沉孔区域采用更细的网格。
- 这一步把 Step 18 的“特征抑制版 V3”推进到“完整孔系 full V3 可网格化 FEA”。

## 输出文件

- 清理后的 full V3 STEP：`{OUT_DIR / "v3_full_cae_clean_5mm.step"}`
- 指标 JSON：`{metrics_json}`
- 指标 CSV：`{metrics_csv}`
- official VTK：`{OUT_DIR / "official_upper_arm_full_v3_clean_fea.vtu"}`
- full V3 VTK：`{OUT_DIR / "v3_full_upper_arm_full_v3_clean_fea.vtu"}`
- 应力图：`{OUT_DIR / "upper_arm_full_v3_clean_fea_von_mises.png"}`
- 位移图：`{OUT_DIR / "upper_arm_full_v3_clean_fea_displacement.png"}`

## 当前边界

- 当前解决的是 full V3 在 Gmsh 下的体网格可生成问题，还不是最终高保真接触仿真。
- 约束/载荷仍是区域式 screening；后续还应在 full V3 clean mesh 上继续做接口边界工况。
- 更细网格会增加计算量，但这是小孔和沉孔进入 CAE 的必要代价。
""",
        encoding="utf-8",
    )
    print(f"wrote {report}")
    print(f"wrote {metrics_csv}")


if __name__ == "__main__":
    run()
