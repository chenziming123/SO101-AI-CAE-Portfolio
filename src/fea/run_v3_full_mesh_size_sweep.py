#!/usr/bin/env python3
"""Sweep full V3 mesh size to test if small-hole failures are mesh-resolution related."""

from __future__ import annotations

from pathlib import Path

import run_upper_arm_static_fea as base


OUT_DIR = (
    base.PROJECT_ROOT
    / "07_fea_analysis"
    / "upper_arm_static_fea"
    / "cae_geometry_cleanup"
)


def run() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for size in [5.0, 4.0, 3.0, 2.5, 2.0, 1.5]:
        tag = str(size).replace(".", "p")
        msh = OUT_DIR / f"full_mount_features_mesh{tag}.msh"
        step = OUT_DIR / f"full_mount_features_mesh{tag}.step"
        print(f"trying full V3 mesh size {size} mm")
        try:
            base.gmsh_mesh_v3_parametric(
                msh,
                step,
                mesh_size_mm=size,
                include_cable_channel=True,
                include_lightening_holes=True,
                include_mount_holes=True,
                include_counterbores=True,
                cut_strategy="fused",
                heal_after_cut=False,
            )
            points, tetra = base.read_tet_mesh(msh)
            print(f"PASS size={size} nodes={points.shape[0]} tets={tetra.shape[0]}")
            return
        except Exception as exc:  # noqa: BLE001 - diagnostic sweep.
            print(f"FAIL size={size}: {exc}")
    raise RuntimeError("Full V3 failed for every tested mesh size")


if __name__ == "__main__":
    run()
