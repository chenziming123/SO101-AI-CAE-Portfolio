#!/usr/bin/env python3
"""Step 23: CalculiX smoke test for SO-101 upper_arm_link.

This script reuses the Step 20 full-V3-clean 5 mm tetrahedral meshes and writes
minimal CalculiX `.inp` jobs for official and full V3. The goal is to verify the
open-source solver chain:

    Gmsh mesh -> CalculiX input -> ccx run -> .dat/.frd output

It is a solver-chain smoke test, not the final high-fidelity contact model.
"""

from __future__ import annotations

import csv
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import run_upper_arm_static_fea as base


PROJECT_ROOT = base.PROJECT_ROOT
STEP20_DIR = (
    PROJECT_ROOT
    / "07_fea_analysis"
    / "upper_arm_static_fea"
    / "results_full_v3_clean"
)
STEP20_TMP = STEP20_DIR / "tmp"
OUT_DIR = (
    PROJECT_ROOT
    / "07_fea_analysis"
    / "upper_arm_static_fea"
    / "results_calculix_smoke"
)
CCX_BIN = (
    PROJECT_ROOT
    / "08_open_source_integration"
    / "envs"
    / "cae_ccx"
    / "bin"
    / "ccx"
)


@dataclass
class CcxSmokeResult:
    name: str
    mesh_path: str
    job_name: str
    node_count: int
    tet_count: int
    fixed_node_count: int
    loaded_node_count: int
    total_force_n: float
    per_loaded_node_force_n: float
    ccx_returncode: int
    ccx_status: str
    inp_path: str
    dat_path: str
    frd_path: str
    sta_path: str
    log_path: str
    dat_size_bytes: int
    frd_size_bytes: int
    max_loaded_disp_mm: float
    mean_loaded_disp_mm: float
    max_von_mises_mpa: float
    p95_von_mises_mpa: float
    safety_factor_yield: float


def ensure_step20_meshes() -> None:
    official = STEP20_TMP / "official_upper_arm_clean_5mm.msh"
    v3 = STEP20_TMP / "v3_full_upper_arm_clean_5mm.msh"
    if official.exists() and v3.exists():
        return
    import run_full_v3_clean_fea

    run_full_v3_clean_fea.run()


def comma_chunks(values: np.ndarray, per_line: int = 12) -> list[str]:
    lines: list[str] = []
    for i in range(0, values.size, per_line):
        chunk = values[i : i + per_line]
        lines.append(", ".join(str(int(v)) for v in chunk))
    return lines


def select_boundary_nodes(
    points: np.ndarray, loadcase: base.LoadCase
) -> tuple[np.ndarray, np.ndarray, int]:
    extents = np.ptp(points, axis=0)
    span_axis = int(np.argmax(extents))
    amin = float(points[:, span_axis].min())
    amax = float(points[:, span_axis].max())
    span = amax - amin
    fixed = np.flatnonzero(points[:, span_axis] <= amin + loadcase.fixed_x_fraction * span)
    loaded = np.flatnonzero(points[:, span_axis] >= amax - loadcase.loaded_x_fraction * span)
    if fixed.size < 4 or loaded.size < 4:
        raise RuntimeError(
            f"Boundary selection too small: fixed={fixed.size}, loaded={loaded.size}"
        )
    return fixed, loaded, span_axis


def write_inp(
    inp_path: Path,
    name: str,
    points: np.ndarray,
    tetra: np.ndarray,
    fixed_nodes0: np.ndarray,
    loaded_nodes0: np.ndarray,
    material: base.Material,
    loadcase: base.LoadCase,
) -> float:
    inp_path.parent.mkdir(parents=True, exist_ok=True)
    loaded_nodes1 = loaded_nodes0 + 1
    fixed_nodes1 = fixed_nodes0 + 1
    all_elements = np.arange(1, tetra.shape[0] + 1)

    direction = np.asarray(loadcase.direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    per_node_force = loadcase.total_force_n * direction / loaded_nodes0.size

    with inp_path.open("w", encoding="utf-8") as f:
        f.write(f"*HEADING\nSO-101 upper_arm_link CalculiX smoke test: {name}\n")
        f.write("*NODE\n")
        for idx, (x, y, z) in enumerate(points, start=1):
            f.write(f"{idx}, {x:.9g}, {y:.9g}, {z:.9g}\n")

        f.write("*ELEMENT, TYPE=C3D4, ELSET=EALL\n")
        for eid, tet in enumerate(tetra + 1, start=1):
            f.write(f"{eid}, {tet[0]}, {tet[1]}, {tet[2]}, {tet[3]}\n")

        f.write("*NSET, NSET=FIXED\n")
        f.write("\n".join(comma_chunks(fixed_nodes1)) + "\n")
        f.write("*NSET, NSET=LOADED\n")
        f.write("\n".join(comma_chunks(loaded_nodes1)) + "\n")
        f.write("*ELSET, ELSET=EALL\n")
        f.write("\n".join(comma_chunks(all_elements)) + "\n")

        f.write(f"*MATERIAL, NAME={material.name.replace(' ', '_')}\n")
        f.write("*ELASTIC\n")
        f.write(f"{material.young_mpa:.9g}, {material.poisson:.9g}\n")
        f.write("*SOLID SECTION, ELSET=EALL, MATERIAL=PLA_screening\n")
        f.write("*BOUNDARY\n")
        f.write("FIXED, 1, 3, 0.0\n")
        f.write("*STEP\n")
        f.write("*STATIC\n")
        f.write("*CLOAD\n")
        for nid in loaded_nodes1:
            for dof, force in enumerate(per_node_force, start=1):
                if abs(force) > 0.0:
                    f.write(f"{int(nid)}, {dof}, {force:.12g}\n")
        f.write("*NODE PRINT, NSET=LOADED\n")
        f.write("U\n")
        f.write("*EL PRINT, ELSET=EALL\n")
        f.write("S\n")
        f.write("*NODE FILE\n")
        f.write("U\n")
        f.write("*EL FILE\n")
        f.write("S\n")
        f.write("*END STEP\n")
    return float(np.linalg.norm(per_node_force))


def run_ccx(job_dir: Path, job_name: str) -> tuple[int, Path]:
    ccx = CCX_BIN if CCX_BIN.exists() else Path("ccx")
    log_path = job_dir / f"{job_name}_ccx.log"
    completed = subprocess.run(
        [str(ccx), "-i", job_name],
        cwd=job_dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    return int(completed.returncode), log_path


def parse_ccx_dat(dat_path: Path) -> tuple[float, float, float, float]:
    """Return max loaded displacement, mean loaded displacement, max VM, p95 VM."""
    if not dat_path.exists():
        return float("nan"), float("nan"), float("nan"), float("nan")

    displacements: list[float] = []
    vm_values: list[float] = []
    mode: str | None = None
    with dat_path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            low = line.lower()
            if low.startswith("displacements"):
                mode = "disp"
                continue
            if low.startswith("stresses"):
                mode = "stress"
                continue
            if mode == "disp":
                parts = line.split()
                if len(parts) != 4:
                    continue
                try:
                    ux, uy, uz = (float(parts[1]), float(parts[2]), float(parts[3]))
                except ValueError:
                    continue
                displacements.append(float(np.linalg.norm([ux, uy, uz])))
            elif mode == "stress":
                parts = line.split()
                if len(parts) != 8:
                    continue
                try:
                    sx, sy, sz = float(parts[2]), float(parts[3]), float(parts[4])
                    sxy, sxz, syz = float(parts[5]), float(parts[6]), float(parts[7])
                except ValueError:
                    continue
                vm = np.sqrt(
                    0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2)
                    + 3.0 * (sxy**2 + sxz**2 + syz**2)
                )
                vm_values.append(float(vm))

    disp_arr = np.asarray(displacements, dtype=float)
    vm_arr = np.asarray(vm_values, dtype=float)
    return (
        float(np.nanmax(disp_arr)) if disp_arr.size else float("nan"),
        float(np.nanmean(disp_arr)) if disp_arr.size else float("nan"),
        float(np.nanmax(vm_arr)) if vm_arr.size else float("nan"),
        float(np.nanpercentile(vm_arr, 95.0)) if vm_arr.size else float("nan"),
    )


def run_case(
    name: str,
    mesh_path: Path,
    material: base.Material,
    loadcase: base.LoadCase,
) -> CcxSmokeResult:
    points, tetra = base.read_tet_mesh(mesh_path)
    fixed_nodes, loaded_nodes, _ = select_boundary_nodes(points, loadcase)

    case_dir = OUT_DIR / name
    job_name = f"{name}_upper_arm_ccx_smoke"
    inp_path = case_dir / f"{job_name}.inp"
    per_node_force = write_inp(
        inp_path,
        name,
        points,
        tetra,
        fixed_nodes,
        loaded_nodes,
        material,
        loadcase,
    )
    returncode, log_path = run_ccx(case_dir, job_name)

    dat_path = case_dir / f"{job_name}.dat"
    frd_path = case_dir / f"{job_name}.frd"
    sta_path = case_dir / f"{job_name}.sta"
    ok = returncode == 0 and frd_path.exists() and frd_path.stat().st_size > 0
    status = "PASS" if ok else "FAIL"
    max_disp, mean_disp, max_vm, p95_vm = parse_ccx_dat(dat_path)

    return CcxSmokeResult(
        name=name,
        mesh_path=str(mesh_path),
        job_name=job_name,
        node_count=int(points.shape[0]),
        tet_count=int(tetra.shape[0]),
        fixed_node_count=int(fixed_nodes.size),
        loaded_node_count=int(loaded_nodes.size),
        total_force_n=float(loadcase.total_force_n),
        per_loaded_node_force_n=float(per_node_force),
        ccx_returncode=returncode,
        ccx_status=status,
        inp_path=str(inp_path),
        dat_path=str(dat_path),
        frd_path=str(frd_path),
        sta_path=str(sta_path),
        log_path=str(log_path),
        dat_size_bytes=dat_path.stat().st_size if dat_path.exists() else 0,
        frd_size_bytes=frd_path.stat().st_size if frd_path.exists() else 0,
        max_loaded_disp_mm=max_disp,
        mean_loaded_disp_mm=mean_disp,
        max_von_mises_mpa=max_vm,
        p95_von_mises_mpa=p95_vm,
        safety_factor_yield=float(material.yield_mpa / max(max_vm, 1e-12))
        if np.isfinite(max_vm)
        else float("nan"),
    )


def write_report(results: list[CcxSmokeResult], material: base.Material) -> None:
    metrics_json = OUT_DIR / "upper_arm_calculix_smoke_metrics.json"
    metrics_csv = OUT_DIR / "upper_arm_calculix_smoke_metrics.csv"
    report = OUT_DIR / "upper_arm_calculix_smoke_report_zh.md"
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    with metrics_json.open("w", encoding="utf-8") as f:
        json.dump(
            {
                "material": material.__dict__,
                "purpose": "CalculiX solver-chain smoke test",
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

    table_lines = [
        "| 版本 | 状态 | 节点 | 四面体 | 固定节点 | 加载节点 | 最大加载位移 mm | 最大 von Mises MPa | 安全系数 |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in results:
        table_lines.append(
            f"| {r.name} | {r.ccx_status} | {r.node_count} | {r.tet_count} | "
            f"{r.fixed_node_count} | {r.loaded_node_count} | "
            f"{r.max_loaded_disp_mm:.6f} | {r.max_von_mises_mpa:.3f} | "
            f"{r.safety_factor_yield:.2f} |"
        )

    report.write_text(
        f"""# SO-101 upper_arm_link CalculiX 求解链 smoke test 报告

## 这一步在做什么

Step 23 使用 Step 20 已经跑通的 official / full V3 clean 5 mm 四面体网格，生成 CalculiX `.inp` 文件，并调用 `ccx` 运行线性静力求解。

这一步的目标不是替代最终高保真 FEA，而是先证明开源 CAE 求解链已经打通：

```text
Gmsh 四面体网格 -> CalculiX .inp -> ccx 求解 -> .dat/.frd 结果文件
```

## 工具链

- CalculiX：`ccx 2.23`
- 输入网格：Step 20 full V3 clean 5 mm `.msh`
- 单元：一阶四面体 `C3D4`
- 材料：{material.name}，E={material.young_mpa:.0f} MPa，nu={material.poisson:.2f}
- 载荷：沿零件最长方向，shoulder 侧固定，elbow 侧施加总计 10 N 向下力

## 核心结果

{chr(10).join(table_lines)}

## 输出文件大小检查

| 版本 | ccx 返回码 | DAT bytes | FRD bytes |
|---|---:|---:|---:|
| {results[0].name} | {results[0].ccx_returncode} | {results[0].dat_size_bytes} | {results[0].frd_size_bytes} |
| {results[1].name} | {results[1].ccx_returncode} | {results[1].dat_size_bytes} | {results[1].frd_size_bytes} |

## 输出文件

- 指标 CSV：`{metrics_csv}`
- 指标 JSON：`{metrics_json}`

### official

- INP：`{results[0].inp_path}`
- DAT：`{results[0].dat_path}`
- FRD：`{results[0].frd_path}`
- LOG：`{results[0].log_path}`

### full V3

- INP：`{results[1].inp_path}`
- DAT：`{results[1].dat_path}`
- FRD：`{results[1].frd_path}`
- LOG：`{results[1].log_path}`

## 当前结论

- 如果两个 case 都为 `PASS`，说明 CalculiX 求解器链路已经可用于本项目。
- 下一步可以开始把 Python screening solver 的结果与 `ccx` 的位移/应力结果做同口径对比。
- 之后再考虑更复杂的边界条件、孔壁面载荷、螺钉接触、打印方向和材料各向异性。

## 当前边界

- 这是 smoke test，只验证求解链路，不作为最终强度结论。
- 当前仍使用节点集合近似固定和加载区域，没有做真实接触。
- 当前使用 C3D4 一阶四面体，后续可以评估 C3D10 或网格收敛。
""",
        encoding="utf-8",
    )
    print(f"wrote {report}")
    print(f"wrote {metrics_csv}")


def run() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ensure_step20_meshes()
    material = base.Material()
    loadcase = base.LoadCase(mesh_size_mm=5.0)
    cases = [
        ("official", STEP20_TMP / "official_upper_arm_clean_5mm.msh"),
        ("v3_full", STEP20_TMP / "v3_full_upper_arm_clean_5mm.msh"),
    ]
    results = [run_case(name, mesh_path, material, loadcase) for name, mesh_path in cases]
    write_report(results, material)
    for r in results:
        print(f"{r.name}: {r.ccx_status} frd={r.frd_size_bytes} dat={r.dat_size_bytes}")


if __name__ == "__main__":
    run()
