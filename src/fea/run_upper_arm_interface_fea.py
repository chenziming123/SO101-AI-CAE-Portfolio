#!/usr/bin/env python3
"""Step 19: interface-boundary static FEA for SO-101 upper_arm_link.

This script reuses the Step 18 tetrahedral meshes and changes only the
boundary-condition selection:

- Step 18 fixed/loaded broad longitudinal bands.
- Step 19 fixes/loads narrow end-interface patches near the shoulder/elbow ends.

The goal is to make the FEA setup closer to a mechanical interface screening
case while keeping the same geometry, material, mesh, and force magnitude.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve

import run_upper_arm_static_fea as base


PROJECT_ROOT = base.PROJECT_ROOT
STEP18_DIR = PROJECT_ROOT / "07_fea_analysis" / "upper_arm_static_fea" / "results"
STEP19_DIR = (
    PROJECT_ROOT
    / "07_fea_analysis"
    / "upper_arm_static_fea"
    / "results_interface_boundary"
)
TMP_DIR = STEP18_DIR / "tmp"


@dataclass(frozen=True)
class InterfaceLoadCase:
    total_force_n: float = 10.0
    direction: tuple[float, float, float] = (0.0, 0.0, -1.0)
    min_interface_band_mm: float = 2.0
    max_interface_band_mm: float = 10.0
    min_boundary_nodes: int = 24
    description: str = (
        "Fix/load narrow end-interface patches instead of broad longitudinal bands"
    )


@dataclass
class InterfaceCaseResult:
    name: str
    mesh_path: str
    node_count: int
    tet_count: int
    span_axis: str
    bbox_extent_mm: str
    mesh_volume_mm3: float
    mass_g: float
    fixed_node_count: int
    loaded_node_count: int
    fixed_band_mm: float
    loaded_band_mm: float
    max_disp_mm: float
    mean_loaded_disp_mm: float
    max_von_mises_mpa: float
    p95_von_mises_mpa: float
    safety_factor_yield: float
    vtk_path: str


def ensure_step18_meshes() -> dict[str, Path]:
    mesh_paths = {
        "official": TMP_DIR / "official_upper_arm_tet.msh",
        "v3": TMP_DIR / "v3_upper_arm_tet.msh",
    }
    if not all(path.exists() for path in mesh_paths.values()):
        print("Step18 meshes missing; running Step18 FEA first.")
        base.run()
    missing = [str(path) for path in mesh_paths.values() if not path.exists()]
    if missing:
        raise FileNotFoundError(f"Missing Step18 mesh files after generation: {missing}")
    return mesh_paths


def select_interface_nodes(
    points: np.ndarray,
    span_axis: int,
    loadcase: InterfaceLoadCase,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    amin = float(points[:, span_axis].min())
    amax = float(points[:, span_axis].max())

    fixed_nodes: np.ndarray | None = None
    loaded_nodes: np.ndarray | None = None
    fixed_band = loadcase.min_interface_band_mm
    loaded_band = loadcase.min_interface_band_mm

    for band in np.linspace(
        loadcase.min_interface_band_mm, loadcase.max_interface_band_mm, 9
    ):
        candidates = np.flatnonzero(points[:, span_axis] <= amin + band)
        if candidates.size >= loadcase.min_boundary_nodes:
            fixed_nodes = candidates
            fixed_band = float(band)
            break
    for band in np.linspace(
        loadcase.min_interface_band_mm, loadcase.max_interface_band_mm, 9
    ):
        candidates = np.flatnonzero(points[:, span_axis] >= amax - band)
        if candidates.size >= loadcase.min_boundary_nodes:
            loaded_nodes = candidates
            loaded_band = float(band)
            break

    if fixed_nodes is None:
        fixed_nodes = np.flatnonzero(
            points[:, span_axis] <= amin + loadcase.max_interface_band_mm
        )
        fixed_band = loadcase.max_interface_band_mm
    if loaded_nodes is None:
        loaded_nodes = np.flatnonzero(
            points[:, span_axis] >= amax - loadcase.max_interface_band_mm
        )
        loaded_band = loadcase.max_interface_band_mm

    if fixed_nodes.size < 4 or loaded_nodes.size < 4:
        raise RuntimeError(
            "Interface boundary selection too small: "
            f"fixed={fixed_nodes.size}, loaded={loaded_nodes.size}"
        )
    return fixed_nodes, loaded_nodes, fixed_band, loaded_band


def solve_interface_case(
    name: str,
    points: np.ndarray,
    tetra: np.ndarray,
    material: base.Material,
    loadcase: InterfaceLoadCase,
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, int, float, float]:
    d = base.elasticity_matrix(material)
    ndof = points.shape[0] * 3
    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    data: list[np.ndarray] = []
    b_mats: list[np.ndarray] = []
    volumes = np.zeros(tetra.shape[0], dtype=float)

    for eid, tet in enumerate(tetra):
        coords = points[tet]
        b, volume = base.tet_b_matrix(coords)
        volumes[eid] = volume
        b_mats.append(b)
        ke = volume * (b.T @ d @ b)
        dofs = np.array([[3 * n, 3 * n + 1, 3 * n + 2] for n in tet]).ravel()
        rr, cc = np.meshgrid(dofs, dofs, indexing="ij")
        rows.append(rr.ravel())
        cols.append(cc.ravel())
        data.append(ke.ravel())

    k = coo_matrix(
        (np.concatenate(data), (np.concatenate(rows), np.concatenate(cols))),
        shape=(ndof, ndof),
    ).tocsr()

    span_axis = int(np.argmax(np.ptp(points, axis=0)))
    fixed_nodes, loaded_nodes, fixed_band, loaded_band = select_interface_nodes(
        points, span_axis, loadcase
    )

    f = np.zeros(ndof, dtype=float)
    direction = np.asarray(loadcase.direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    per_node_force = loadcase.total_force_n * direction / loaded_nodes.size
    for n in loaded_nodes:
        f[3 * n : 3 * n + 3] += per_node_force

    fixed_dofs = np.array([[3 * n, 3 * n + 1, 3 * n + 2] for n in fixed_nodes]).ravel()
    all_dofs = np.arange(ndof)
    free_dofs = np.setdiff1d(all_dofs, fixed_dofs, assume_unique=False)

    u = np.zeros(ndof, dtype=float)
    u[free_dofs] = spsolve(k[free_dofs][:, free_dofs], f[free_dofs])

    stresses = np.zeros((tetra.shape[0], 6), dtype=float)
    for eid, tet in enumerate(tetra):
        dofs = np.array([[3 * n, 3 * n + 1, 3 * n + 2] for n in tet]).ravel()
        strain = b_mats[eid] @ u[dofs]
        stresses[eid] = d @ strain

    return (
        u.reshape((-1, 3)),
        base.von_mises(stresses),
        float(volumes.sum()),
        fixed_nodes,
        loaded_nodes,
        span_axis,
        fixed_band,
        loaded_band,
    )


def run() -> None:
    material = base.Material()
    loadcase = InterfaceLoadCase()
    STEP19_DIR.mkdir(parents=True, exist_ok=True)
    mesh_paths = ensure_step18_meshes()

    results: list[InterfaceCaseResult] = []
    plot_payload: dict[str, dict] = {}

    for name, mesh_path in mesh_paths.items():
        print(f"running interface-boundary FEA: {name}")
        points, tetra = base.read_tet_mesh(mesh_path)
        (
            displacement,
            vm,
            volume_mm3,
            fixed_nodes,
            loaded_nodes,
            span_axis,
            fixed_band,
            loaded_band,
        ) = solve_interface_case(name, points, tetra, material, loadcase)

        disp_mag = np.linalg.norm(displacement, axis=1)
        vtk_path = STEP19_DIR / f"{name}_upper_arm_interface_fea.vtu"
        base.save_vtk(vtk_path, points, tetra, displacement, vm)
        faces, owners = base.boundary_faces(tetra)
        plot_payload[name] = {
            "points": points,
            "tetra": tetra,
            "faces": faces,
            "face_owners": owners,
            "span_axis": span_axis,
            "von_mises_mpa": vm,
            "disp_by_elem_mm": disp_mag[tetra].mean(axis=1),
        }

        extents = np.ptp(points, axis=0)
        results.append(
            InterfaceCaseResult(
                name=name,
                mesh_path=str(mesh_path),
                node_count=int(points.shape[0]),
                tet_count=int(tetra.shape[0]),
                span_axis="xyz"[span_axis],
                bbox_extent_mm=",".join(f"{v:.3f}" for v in extents),
                mesh_volume_mm3=volume_mm3,
                mass_g=float(volume_mm3 / 1000.0 * material.density_g_per_cm3),
                fixed_node_count=int(fixed_nodes.size),
                loaded_node_count=int(loaded_nodes.size),
                fixed_band_mm=fixed_band,
                loaded_band_mm=loaded_band,
                max_disp_mm=float(disp_mag.max()),
                mean_loaded_disp_mm=float(disp_mag[loaded_nodes].mean()),
                max_von_mises_mpa=float(vm.max()),
                p95_von_mises_mpa=float(np.percentile(vm, 95.0)),
                safety_factor_yield=float(material.yield_mpa / max(vm.max(), 1e-12)),
                vtk_path=str(vtk_path),
            )
        )

    metrics_json = STEP19_DIR / "upper_arm_interface_fea_metrics.json"
    metrics_csv = STEP19_DIR / "upper_arm_interface_fea_metrics.csv"
    with metrics_json.open("w", encoding="utf-8") as f:
        json.dump(
            {
                "material": material.__dict__,
                "loadcase": loadcase.__dict__,
                "results": [r.__dict__ for r in results],
            },
            f,
            indent=2,
            ensure_ascii=False,
        )
    with metrics_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].__dict__.keys()))
        writer.writeheader()
        for result in results:
            writer.writerow(result.__dict__)

    base.plot_case_surfaces(
        plot_payload,
        "von_mises_mpa",
        STEP19_DIR / "upper_arm_interface_fea_von_mises.png",
        "interface-boundary von Mises stress / MPa",
        "inferno",
    )
    base.plot_case_surfaces(
        plot_payload,
        "disp_by_elem_mm",
        STEP19_DIR / "upper_arm_interface_fea_displacement.png",
        "interface-boundary displacement magnitude / mm",
        "viridis",
    )

    official = next(r for r in results if r.name == "official")
    v3 = next(r for r in results if r.name == "v3")
    mass_change = (v3.mass_g / official.mass_g - 1.0) * 100.0
    disp_change = (v3.max_disp_mm / official.max_disp_mm - 1.0) * 100.0
    stress_change = (v3.max_von_mises_mpa / official.max_von_mises_mpa - 1.0) * 100.0

    report = STEP19_DIR / "upper_arm_interface_fea_report_zh.md"
    report.write_text(
        f"""# SO-101 upper_arm_link Official / V3 接口边界静力有限元对比

## 这一步在做什么

Step 19 在 Step 18 已经跑通的四面体网格基础上，只升级边界条件：从“固定/加载一大段长度区域”改为“固定/加载靠近 shoulder/elbow 端部接口的窄区域节点”。这样可以更接近机械接口受力，而不改变材料、网格和总载荷。

## 工况设定

- 几何和网格：复用 Step 18 的 official STEP 网格和 V3 可网格化 CAE 变体网格。
- 单位体系：mm、N、MPa。
- 材料：{material.name}，弹性模量 {material.young_mpa:.0f} MPa，泊松比 {material.poisson:.2f}，屈服强度按 {material.yield_mpa:.0f} MPa 做 screening。
- 约束：固定 shoulder 侧端部接口窄带节点。
- 载荷：在 elbow 侧端部接口窄带节点施加合力 {loadcase.total_force_n:.1f} N，方向为 Z 负方向。
- 边界节点选择：从 {loadcase.min_interface_band_mm:.1f} mm 端部窄带开始，如果节点太少，自动放宽到最多 {loadcase.max_interface_band_mm:.1f} mm。

## 核心结果

| 版本 | 主轴 | 固定节点 | 加载节点 | 固定带宽 mm | 加载带宽 mm | 质量 g | 最大位移 mm | 最大 von Mises MPa | p95 von Mises MPa | 屈服安全系数 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| official | {official.span_axis} | {official.fixed_node_count} | {official.loaded_node_count} | {official.fixed_band_mm:.1f} | {official.loaded_band_mm:.1f} | {official.mass_g:.2f} | {official.max_disp_mm:.6f} | {official.max_von_mises_mpa:.3f} | {official.p95_von_mises_mpa:.3f} | {official.safety_factor_yield:.2f} |
| V3 | {v3.span_axis} | {v3.fixed_node_count} | {v3.loaded_node_count} | {v3.fixed_band_mm:.1f} | {v3.loaded_band_mm:.1f} | {v3.mass_g:.2f} | {v3.max_disp_mm:.6f} | {v3.max_von_mises_mpa:.3f} | {v3.p95_von_mises_mpa:.3f} | {v3.safety_factor_yield:.2f} |

## 对比结论

- V3 相对 official 的估算质量变化：{mass_change:+.2f}%。
- V3 相对 official 的最大位移变化：{disp_change:+.2f}%。
- V3 相对 official 的最大 von Mises 应力变化：{stress_change:+.2f}%。
- 与 Step 18 相比，Step 19 的意义不是追求更好看的数值，而是让边界条件更像真实接口载荷；这会让应力集中更明显，也更适合指导下一版结构加强。

## 输出文件

- 指标 JSON：`{metrics_json}`
- 指标 CSV：`{metrics_csv}`
- official VTK：`{STEP19_DIR / "official_upper_arm_interface_fea.vtu"}`
- V3 VTK：`{STEP19_DIR / "v3_upper_arm_interface_fea.vtu"}`
- 应力图：`{STEP19_DIR / "upper_arm_interface_fea_von_mises.png"}`
- 位移图：`{STEP19_DIR / "upper_arm_interface_fea_displacement.png"}`

## 当前边界

- 当前仍是端部接口窄带节点选择，还不是从 STEP B-Rep 精确识别孔壁面、螺钉接触面或轴承接触面。
- V3 仍采用 Step 18 跑通的 `no_small_mount_holes` CAE 变体；full V3 的小装配孔和沉孔仍需要继续清理。
- 这一步适合用于“FEA 边界条件升级”和“V4 加强设计依据”，不作为最终生产强度认证。
""",
        encoding="utf-8",
    )
    print(f"wrote {report}")
    print(f"wrote {metrics_csv}")


if __name__ == "__main__":
    run()
