#!/usr/bin/env python3
"""Diagnose and clean V3 CAE geometry for tetrahedral meshing.

The normal V3 CAD/STEP is valid for visualization and assembly checks, but Gmsh
reported overlapping boundary facets when the full small-hole/counterbore set
was used for tetrahedral FEA. This script isolates the failing feature group and
tries cleaner boolean strategies.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import run_upper_arm_static_fea as base


PROJECT_ROOT = base.PROJECT_ROOT
OUT_DIR = (
    PROJECT_ROOT / "07_fea_analysis" / "upper_arm_static_fea" / "cae_geometry_cleanup"
)


@dataclass(frozen=True)
class GeometryAttempt:
    name: str
    include_cable_channel: bool = True
    include_lightening_holes: bool = True
    include_shoulder_clamp_holes: bool = False
    include_elbow_mount_holes: bool = False
    include_counterbores: bool = False
    cut_strategy: str = "fused"
    heal_after_cut: bool = False


def attempt_mesh(attempt: GeometryAttempt) -> dict[str, str | int | float]:
    msh_path = OUT_DIR / f"{attempt.name}.msh"
    step_path = OUT_DIR / f"{attempt.name}.step"
    for path in (msh_path, step_path):
        if path.exists():
            path.unlink()

    try:
        base.gmsh_mesh_v3_parametric(
            msh_path,
            step_path,
            mesh_size_mm=7.0,
            include_cable_channel=attempt.include_cable_channel,
            include_lightening_holes=attempt.include_lightening_holes,
            include_mount_holes=False,
            include_shoulder_clamp_holes=attempt.include_shoulder_clamp_holes,
            include_elbow_mount_holes=attempt.include_elbow_mount_holes,
            include_counterbores=attempt.include_counterbores,
            cut_strategy=attempt.cut_strategy,
            heal_after_cut=attempt.heal_after_cut,
        )
        points, tetra = base.read_tet_mesh(msh_path)
        return {
            "name": attempt.name,
            "status": "PASS",
            "error": "",
            "cut_strategy": attempt.cut_strategy,
            "heal_after_cut": str(attempt.heal_after_cut),
            "shoulder_clamp": str(attempt.include_shoulder_clamp_holes),
            "elbow_mount": str(attempt.include_elbow_mount_holes),
            "counterbores": str(attempt.include_counterbores),
            "nodes": int(points.shape[0]),
            "tets": int(tetra.shape[0]),
            "bbox_extent_mm": ",".join(f"{v:.3f}" for v in np.ptp(points, axis=0)),
            "msh_path": str(msh_path),
            "step_path": str(step_path),
        }
    except Exception as exc:  # noqa: BLE001 - this script is a diagnostic matrix.
        return {
            "name": attempt.name,
            "status": "FAIL",
            "error": str(exc),
            "cut_strategy": attempt.cut_strategy,
            "heal_after_cut": str(attempt.heal_after_cut),
            "shoulder_clamp": str(attempt.include_shoulder_clamp_holes),
            "elbow_mount": str(attempt.include_elbow_mount_holes),
            "counterbores": str(attempt.include_counterbores),
            "nodes": 0,
            "tets": 0,
            "bbox_extent_mm": "",
            "msh_path": str(msh_path),
            "step_path": str(step_path),
        }


def build_attempts() -> list[GeometryAttempt]:
    base_groups = [
        ("core", False, False, False),
        ("shoulder_clamp_only", True, False, False),
        ("elbow_through_only", False, True, False),
        ("elbow_with_counterbores", False, True, True),
        ("all_mount_no_counterbores", True, True, False),
        ("full_mount_features", True, True, True),
    ]
    attempts: list[GeometryAttempt] = []
    for cut_strategy in ("fused", "sequential"):
        for heal_after_cut in (False, True):
            for group_name, shoulder, elbow, counterbore in base_groups:
                attempts.append(
                    GeometryAttempt(
                        name=f"{group_name}_{cut_strategy}_{'heal' if heal_after_cut else 'noheal'}",
                        include_shoulder_clamp_holes=shoulder,
                        include_elbow_mount_holes=elbow,
                        include_counterbores=counterbore,
                        cut_strategy=cut_strategy,
                        heal_after_cut=heal_after_cut,
                    )
                )
    return attempts


def write_report(rows: list[dict[str, str | int | float]]) -> Path:
    report = OUT_DIR / "upper_arm_v3_cae_geometry_cleanup_report_zh.md"
    full_pass = [
        row
        for row in rows
        if row["status"] == "PASS" and str(row["name"]).startswith("full_mount_features")
    ]
    elbow_pass = [
        row
        for row in rows
        if row["status"] == "PASS" and row["elbow_mount"] == "True"
    ]
    failed_full = [
        row
        for row in rows
        if str(row["name"]).startswith("full_mount_features") and row["status"] == "FAIL"
    ]
    best = full_pass[0] if full_pass else (elbow_pass[0] if elbow_pass else None)

    table_lines = [
        "| 尝试 | 状态 | 肩部夹紧孔 | 肘部安装孔 | 沉孔 | 布尔策略 | healing | 节点 | 四面体 | 失败原因 |",
        "|---|---|---|---|---|---|---|---:|---:|---|",
    ]
    for row in rows:
        error = str(row["error"]).replace("|", "/")
        if len(error) > 80:
            error = error[:77] + "..."
        table_lines.append(
            "| {name} | {status} | {shoulder_clamp} | {elbow_mount} | {counterbores} | "
            "{cut_strategy} | {heal_after_cut} | {nodes} | {tets} | {error} |".format(
                **row
            )
        )

    if full_pass:
        conclusion = (
            "full V3 的小装配孔和沉孔已经找到可网格化组合，后续可直接用该 STEP/MESH 进入 FEA。"
        )
    else:
        conclusion = (
            "full V3 仍未完全可网格化；诊断结果用于定位失败孔系，并指导后续 CAE 专用孔系重建。"
        )

    best_text = (
        f"- 当前最完整可网格化结果：`{best['name']}`，节点 `{best['nodes']}`，四面体 `{best['tets']}`。"
        if best
        else "- 当前没有任何带肘部安装孔的组合通过，需要进一步重建孔系。"
    )
    failed_text = "\n".join(
        f"- `{row['name']}`：{row['error']}" for row in failed_full[:4]
    )

    report.write_text(
        f"""# upper_arm_v3 CAE 几何清理诊断报告

## 这一步在做什么

Step 20 不做 V4，而是先清理和诊断 V3 的 CAE 几何。目标是回答一个明确问题：为什么 full V3 带小装配孔和沉孔时不能稳定生成四面体网格。

## 诊断方法

- 保持 V3 主体、梁、肋板、减重孔和主关节孔不变。
- 将小特征拆成不同组合测试：肩部夹紧孔、肘部安装通孔、肘部沉孔、完整孔系。
- 对每个组合分别测试两种布尔策略：一次性融合 cutter 后切除、逐个 cutter 顺序切除。
- 每种策略再测试是否进行 OpenCASCADE healing。

## 结论

- {conclusion}
{best_text}

## full V3 失败摘要

{failed_text if failed_text else "- full V3 没有失败记录。"}

## 全部尝试结果

{chr(10).join(table_lines)}

## 输出文件

- 诊断 CSV：`{OUT_DIR / "upper_arm_v3_cae_geometry_cleanup_matrix.csv"}`
- 诊断目录：`{OUT_DIR}`

## 当前边界

- 这一步是 CAE 前处理诊断，不是最终强度结论。
- 如果 full V3 仍然失败，下一步应按失败定位结果重建孔系，而不是继续用展示用 STEP 硬跑 FEA。
""",
        encoding="utf-8",
    )
    return report


def run() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    for attempt in build_attempts():
        print(f"trying {attempt.name}")
        row = attempt_mesh(attempt)
        rows.append(row)
        print(f"  {row['status']}: {row['error']}")

    csv_path = OUT_DIR / "upper_arm_v3_cae_geometry_cleanup_matrix.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    report = write_report(rows)
    print(f"wrote {csv_path}")
    print(f"wrote {report}")


if __name__ == "__main__":
    run()
