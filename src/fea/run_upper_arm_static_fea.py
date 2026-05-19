#!/usr/bin/env python3
"""Static FEA screening for SO-101 upper_arm_link.

This script compares the official upper arm STL and the AI-rebuilt V3 STL
under the same simple cantilever-style loading condition:

- units: mm, N, MPa
- material: PLA screening values
- fixed region: shoulder-side longitudinal band
- loaded region: elbow-side longitudinal band, downward Z force

It uses Gmsh for tetrahedral meshing and a small linear tetrahedral elasticity
solver implemented with NumPy/SciPy. The result is intended as an engineering
screening workflow, not as a certified production CAE result.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import meshio
import numpy as np
from matplotlib.collections import PolyCollection
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve


PROJECT_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = PROJECT_ROOT / "07_fea_analysis" / "upper_arm_static_fea" / "results"
TMP_DIR = OUT_DIR / "tmp"


@dataclass(frozen=True)
class Material:
    name: str = "PLA screening"
    young_mpa: float = 2500.0
    poisson: float = 0.35
    yield_mpa: float = 50.0
    density_g_per_cm3: float = 1.24


@dataclass(frozen=True)
class LoadCase:
    total_force_n: float = 10.0
    direction: tuple[float, float, float] = (0.0, 0.0, -1.0)
    fixed_x_fraction: float = 0.12
    loaded_x_fraction: float = 0.10
    mesh_size_mm: float = 7.0


@dataclass
class CaseResult:
    name: str
    source: str
    geometry_type: str
    scale_to_mm: float
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


def candidate_path(paths: Iterable[Path]) -> Path:
    for p in paths:
        if p.exists():
            return p
    raise FileNotFoundError("None of the candidate geometry paths exists")


def load_cases() -> dict[str, Path]:
    official = candidate_path(
        [
            PROJECT_ROOT / "00_source_snapshot/STEP_SO101/Upper_arm_SO101.step",
            PROJECT_ROOT / "00_source_snapshot/STEP_SO101/Upper_arm_SO101.stp",
            PROJECT_ROOT
            / "05_improved_design/upper_arm_v3/urdf_smoke/assets/upper_arm_so101_v1.stl",
            PROJECT_ROOT
            / "05_improved_design/upper_arm_v3/inertial_compare/assets/upper_arm_so101_v1.stl",
            PROJECT_ROOT
            / "05_improved_design/upper_arm_v2/urdf_smoke/assets/upper_arm_so101_v1.stl",
        ]
    )
    v3 = candidate_path(
        [
            PROJECT_ROOT / "05_improved_design/upper_arm_v3/upper_arm_v3_ai_rebuild.stl",
            PROJECT_ROOT / "05_improved_design/upper_arm_v3/upper_arm_v3_ai_rebuild.step",
        ]
    )
    return {"official": official, "v3": v3}


def preprocess_stl_to_mm(src: Path, dst: Path) -> float:
    import trimesh

    loaded = trimesh.load_mesh(src, force="mesh", process=True)
    if isinstance(loaded, trimesh.Scene):
        loaded = trimesh.util.concatenate(tuple(loaded.geometry.values()))

    mesh = loaded.copy()
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()
    try:
        mesh.update_faces(mesh.unique_faces())
        mesh.update_faces(mesh.nondegenerate_faces())
    except AttributeError:
        mesh.remove_duplicate_faces()
        mesh.remove_degenerate_faces()
    trimesh.repair.fix_normals(mesh)
    trimesh.repair.fix_inversion(mesh)
    trimesh.repair.fill_holes(mesh)

    points = np.asarray(mesh.vertices, dtype=float)
    extent = np.ptp(points, axis=0)
    max_extent = float(np.max(extent))
    scale = 1000.0 if max_extent < 2.0 else 1.0
    mesh.vertices = points * scale
    mesh.export(dst)
    return scale


def gmsh_mesh_geometry(geometry_path: Path, msh_path: Path, mesh_size_mm: float) -> None:
    import gmsh

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", mesh_size_mm * 0.55)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", mesh_size_mm)
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
        gmsh.option.setNumber("Geometry.OCCFixDegenerated", 1)
        gmsh.option.setNumber("Geometry.OCCFixSmallEdges", 1)
        gmsh.option.setNumber("Geometry.OCCFixSmallFaces", 1)
        gmsh.option.setNumber("Geometry.OCCSewFaces", 1)

        gmsh.model.add(geometry_path.stem)
        suffix = geometry_path.suffix.lower()
        if suffix in {".step", ".stp"}:
            gmsh.model.occ.importShapes(str(geometry_path))
            gmsh.model.occ.removeAllDuplicates()
            gmsh.model.occ.healShapes(
                [],
                tolerance=1e-4,
                fixDegenerated=True,
                fixSmallEdges=True,
                fixSmallFaces=True,
                sewFaces=True,
                makeSolids=True,
            )
            gmsh.model.occ.synchronize()
            volumes = gmsh.model.getEntities(3)
            if not volumes:
                raise RuntimeError(f"No solid volumes found in {geometry_path}")
        else:
            gmsh.merge(str(geometry_path))
            # Convert the STL shell into a discrete CAD-like volume.
            gmsh.model.mesh.classifySurfaces(
                math.radians(40.0), True, False, math.radians(180.0)
            )
            gmsh.model.mesh.createGeometry()
            surfaces = [tag for dim, tag in gmsh.model.getEntities(2)]
            if not surfaces:
                raise RuntimeError(f"No classified surfaces found in {geometry_path}")

            loop = gmsh.model.geo.addSurfaceLoop(surfaces)
            gmsh.model.geo.addVolume([loop])
            gmsh.model.geo.synchronize()

        gmsh.model.mesh.generate(3)
        gmsh.write(str(msh_path))
    finally:
        gmsh.finalize()


def gmsh_common_options(gmsh, mesh_size_mm: float) -> None:
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", mesh_size_mm * 0.55)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", mesh_size_mm)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
    gmsh.option.setNumber("Geometry.OCCFixDegenerated", 1)
    gmsh.option.setNumber("Geometry.OCCFixSmallEdges", 1)
    gmsh.option.setNumber("Geometry.OCCFixSmallFaces", 1)
    gmsh.option.setNumber("Geometry.OCCSewFaces", 1)


def gmsh_heal_and_mesh(
    gmsh,
    msh_path: Path,
    step_debug_path: Path | None = None,
    heal: bool = True,
) -> None:
    gmsh.model.occ.removeAllDuplicates()
    if heal:
        gmsh.model.occ.healShapes(
            [],
            tolerance=1e-4,
            fixDegenerated=True,
            fixSmallEdges=True,
            fixSmallFaces=True,
            sewFaces=True,
            makeSolids=True,
        )
    gmsh.model.occ.synchronize()
    volumes = gmsh.model.getEntities(3)
    if not volumes:
        raise RuntimeError("No solid volumes found before meshing")
    if step_debug_path is not None:
        gmsh.write(str(step_debug_path))
    gmsh.model.mesh.generate(3)
    gmsh.write(str(msh_path))


def gmsh_mesh_v3_parametric(
    msh_path: Path,
    step_debug_path: Path,
    mesh_size_mm: float,
    *,
    include_cable_channel: bool = True,
    include_lightening_holes: bool = True,
    include_mount_holes: bool = True,
    include_shoulder_clamp_holes: bool | None = None,
    include_elbow_mount_holes: bool | None = None,
    include_counterbores: bool = True,
    cut_strategy: str = "fused",
    heal_after_cut: bool = False,
) -> None:
    import gmsh

    if include_shoulder_clamp_holes is None:
        include_shoulder_clamp_holes = include_mount_holes
    if include_elbow_mount_holes is None:
        include_elbow_mount_holes = include_mount_holes

    # V3 dimensions mirror 05_improved_design/upper_arm_v3/upper_arm_v3_parameters.json.
    span = 116.0
    shoulder_boss_radius = 18.0
    elbow_boss_radius = 16.5
    joint_bore_radius = 4.2
    boss_width = 34.0
    rail_length = 98.0
    rail_center_x = span / 2.0
    rail_width_y = 15.0
    rail_thickness_z = 4.8
    rail_offset_z = 13.5
    web_length = 88.0
    web_thickness_y = 3.0
    web_height_z = 28.0
    rib_thickness_x = 5.5
    rib_width_y = 18.0
    rib_height_z = 36.0
    rib_x_positions = (26.0, 58.0, 90.0)
    cable_channel_length = 82.0
    cable_channel_width_y = 8.0
    cable_channel_wall = 1.8
    cable_channel_z = rail_offset_z + rail_thickness_z / 2.0 + 1.2
    lightening_hole_radius = 5.4
    lightening_hole_x_positions = (38.0, 58.0, 78.0)
    shoulder_pad_x = -5.0
    shoulder_pad_size = (22.0, 36.0, 30.0)
    shoulder_clamp_hole_x = -8.0
    shoulder_clamp_hole_z = 10.5
    shoulder_clamp_hole_radius = 2.0
    elbow_pad_x = 103.0
    elbow_pad_size = (30.0, 34.0, 36.0)
    elbow_servo_center_x = 103.0
    elbow_servo_spacing_x = 9.9
    elbow_servo_spacing_y = 9.9
    elbow_servo_hole_radius = 1.75
    elbow_servo_counterbore_radius = 2.95
    elbow_servo_counterbore_depth = 2.4

    def box(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float):
        return (
            3,
            gmsh.model.occ.addBox(cx - sx / 2.0, cy - sy / 2.0, cz - sz / 2.0, sx, sy, sz),
        )

    def cyl_y(cx: float, cy: float, cz: float, radius: float, length: float):
        return (
            3,
            gmsh.model.occ.addCylinder(cx, cy - length / 2.0, cz, 0.0, length, 0.0, radius),
        )

    def cyl_z(cx: float, cy: float, cz: float, radius: float, length: float):
        return (
            3,
            gmsh.model.occ.addCylinder(cx, cy, cz - length / 2.0, 0.0, 0.0, length, radius),
        )

    gmsh.initialize()
    try:
        gmsh_common_options(gmsh, mesh_size_mm)
        gmsh.model.add("upper_arm_v3_parametric_for_fea")

        solids = [
            box(rail_center_x, 0.0, rail_offset_z, rail_length, rail_width_y, rail_thickness_z),
            box(rail_center_x, 0.0, -rail_offset_z, rail_length, rail_width_y, rail_thickness_z),
            box(rail_center_x, 0.0, 0.0, web_length, web_thickness_y, web_height_z),
            box(shoulder_pad_x, 0.0, 0.0, *shoulder_pad_size),
            box(8.0, 0.0, 0.0, 14.0, 30.0, 28.0),
            box(elbow_pad_x, 0.0, 0.0, *elbow_pad_size),
            box(span - 15.0, 0.0, 0.0, 10.0, 24.0, 35.0),
            cyl_y(0.0, 0.0, 0.0, shoulder_boss_radius, boss_width),
            cyl_y(span, 0.0, 0.0, elbow_boss_radius, boss_width),
        ]
        if include_cable_channel:
            solids.extend(
                [
                    box(
                rail_center_x,
                0.0,
                cable_channel_z,
                cable_channel_length,
                cable_channel_width_y,
                cable_channel_wall,
                    ),
                    box(
                rail_center_x,
                cable_channel_width_y / 2.0,
                cable_channel_z + 2.1,
                cable_channel_length,
                cable_channel_wall,
                4.2,
                    ),
                    box(
                rail_center_x,
                -cable_channel_width_y / 2.0,
                cable_channel_z + 2.1,
                cable_channel_length,
                cable_channel_wall,
                4.2,
                    ),
                ]
            )
        solids.extend(
            box(x, 0.0, 0.0, rib_thickness_x, rib_width_y, rib_height_z)
            for x in rib_x_positions
        )

        fused, _ = gmsh.model.occ.fuse([solids[0]], solids[1:], removeObject=True, removeTool=True)

        cutters = [
            cyl_y(0.0, 0.0, 0.0, joint_bore_radius, 44.0),
            cyl_y(span, 0.0, 0.0, joint_bore_radius, 44.0),
        ]
        if include_shoulder_clamp_holes:
            cutters.extend(
                [
                    cyl_y(
                        shoulder_clamp_hole_x,
                        0.0,
                        -shoulder_clamp_hole_z,
                        shoulder_clamp_hole_radius,
                        44.0,
                    ),
                    cyl_y(
                        shoulder_clamp_hole_x,
                        0.0,
                        shoulder_clamp_hole_z,
                        shoulder_clamp_hole_radius,
                        44.0,
                    ),
                ]
            )
        if include_lightening_holes:
            cutters.extend(
                cyl_y(x, 0.0, 0.0, lightening_hole_radius, web_thickness_y + 4.0)
                for x in lightening_hole_x_positions
            )
        if include_elbow_mount_holes:
            half_x = elbow_servo_spacing_x / 2.0
            half_y = elbow_servo_spacing_y / 2.0
            top_z = elbow_pad_size[2] / 2.0
            bottom_z = -elbow_pad_size[2] / 2.0
            for x_offset in (-half_x, half_x):
                for y_offset in (-half_y, half_y):
                    x = elbow_servo_center_x + x_offset
                    y = y_offset
                    cutters.append(cyl_z(x, y, 0.0, elbow_servo_hole_radius, elbow_pad_size[2] + 8.0))
                    if include_counterbores:
                        cutters.append(
                            cyl_z(
                                x,
                                y,
                                top_z - elbow_servo_counterbore_depth / 2.0,
                                elbow_servo_counterbore_radius,
                                elbow_servo_counterbore_depth + 0.1,
                            )
                        )
                        cutters.append(
                            cyl_z(
                                x,
                                y,
                                bottom_z + elbow_servo_counterbore_depth / 2.0,
                                elbow_servo_counterbore_radius,
                                elbow_servo_counterbore_depth + 0.1,
                            )
                        )

        if cut_strategy == "sequential":
            current = fused
            for cutter in cutters:
                current, _ = gmsh.model.occ.cut(
                    current, [cutter], removeObject=True, removeTool=True
                )
            gmsh_heal_and_mesh(gmsh, msh_path, step_debug_path, heal=heal_after_cut)
        elif cut_strategy == "fused":
            fused_cutters, _ = gmsh.model.occ.fuse(
                [cutters[0]], cutters[1:], removeObject=True, removeTool=True
            )
            gmsh.model.occ.cut(fused, fused_cutters, removeObject=True, removeTool=True)
            gmsh_heal_and_mesh(gmsh, msh_path, step_debug_path, heal=heal_after_cut)
        else:
            raise ValueError(f"Unknown cut_strategy: {cut_strategy}")
    finally:
        gmsh.finalize()


def read_tet_mesh(msh_path: Path) -> tuple[np.ndarray, np.ndarray]:
    mesh = meshio.read(msh_path)
    tetra = None
    for block in mesh.cells:
        if block.type == "tetra":
            tetra = np.asarray(block.data, dtype=int)
            break
        if block.type == "tetra10":
            tetra = np.asarray(block.data[:, :4], dtype=int)
            break
    if tetra is None:
        raise RuntimeError(f"No tetrahedral cells found in {msh_path}")

    points = np.asarray(mesh.points[:, :3], dtype=float)
    used = np.unique(tetra.ravel())
    remap = np.full(points.shape[0], -1, dtype=int)
    remap[used] = np.arange(used.size)
    return points[used], remap[tetra]


def elasticity_matrix(material: Material) -> np.ndarray:
    e = material.young_mpa
    nu = material.poisson
    c = e / ((1.0 + nu) * (1.0 - 2.0 * nu))
    d = np.array(
        [
            [1.0 - nu, nu, nu, 0.0, 0.0, 0.0],
            [nu, 1.0 - nu, nu, 0.0, 0.0, 0.0],
            [nu, nu, 1.0 - nu, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, (1.0 - 2.0 * nu) / 2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, (1.0 - 2.0 * nu) / 2.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, (1.0 - 2.0 * nu) / 2.0],
        ],
        dtype=float,
    )
    return c * d


def tet_b_matrix(coords: np.ndarray) -> tuple[np.ndarray, float]:
    m = np.column_stack([np.ones(4), coords])
    det = float(np.linalg.det(m))
    volume = abs(det) / 6.0
    if volume <= 1e-10:
        raise ValueError("Degenerate tetrahedron")
    inv_m = np.linalg.inv(m)
    grads = inv_m[1:, :].T
    b = np.zeros((6, 12), dtype=float)
    for i, (dn_dx, dn_dy, dn_dz) in enumerate(grads):
        j = 3 * i
        b[0, j] = dn_dx
        b[1, j + 1] = dn_dy
        b[2, j + 2] = dn_dz
        b[3, j] = dn_dy
        b[3, j + 1] = dn_dx
        b[4, j + 1] = dn_dz
        b[4, j + 2] = dn_dy
        b[5, j] = dn_dz
        b[5, j + 2] = dn_dx
    return b, volume


def von_mises(stress: np.ndarray) -> np.ndarray:
    sx, sy, sz, txy, tyz, txz = stress.T
    return np.sqrt(
        0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2)
        + 3.0 * (txy**2 + tyz**2 + txz**2)
    )


def solve_static_case(
    name: str,
    points: np.ndarray,
    tetra: np.ndarray,
    material: Material,
    loadcase: LoadCase,
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, int]:
    d = elasticity_matrix(material)
    ndof = points.shape[0] * 3
    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    data: list[np.ndarray] = []
    b_mats: list[np.ndarray] = []
    volumes = np.zeros(tetra.shape[0], dtype=float)

    for eid, tet in enumerate(tetra):
        coords = points[tet]
        b, volume = tet_b_matrix(coords)
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

    extents = np.ptp(points, axis=0)
    span_axis = int(np.argmax(extents))
    amin, amax = float(points[:, span_axis].min()), float(points[:, span_axis].max())
    span = amax - amin
    fixed_nodes = np.flatnonzero(
        points[:, span_axis] <= amin + loadcase.fixed_x_fraction * span
    )
    loaded_nodes = np.flatnonzero(
        points[:, span_axis] >= amax - loadcase.loaded_x_fraction * span
    )
    if fixed_nodes.size < 4 or loaded_nodes.size < 4:
        raise RuntimeError(
            f"{name}: boundary selection too small, fixed={fixed_nodes.size}, "
            f"loaded={loaded_nodes.size}"
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
        von_mises(stresses),
        volumes.sum(),
        fixed_nodes,
        loaded_nodes,
        span_axis,
    )


def boundary_faces(tetra: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    local_faces = np.array(
        [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        dtype=int,
    )
    faces = []
    owners = []
    for eid, tet in enumerate(tetra):
        for face in local_faces:
            faces.append(tet[face])
            owners.append(eid)
    faces_arr = np.asarray(faces, dtype=int)
    owners_arr = np.asarray(owners, dtype=int)
    keys = np.sort(faces_arr, axis=1)
    _, idx, counts = np.unique(keys, axis=0, return_index=True, return_counts=True)
    boundary_idx = idx[counts == 1]
    return faces_arr[boundary_idx], owners_arr[boundary_idx]


def save_vtk(
    path: Path,
    points: np.ndarray,
    tetra: np.ndarray,
    displacement: np.ndarray,
    von_mises_mpa: np.ndarray,
) -> None:
    mesh = meshio.Mesh(
        points=points,
        cells=[("tetra", tetra)],
        point_data={
            "displacement_mm": displacement,
            "displacement_magnitude_mm": np.linalg.norm(displacement, axis=1),
        },
        cell_data={"von_mises_mpa": [von_mises_mpa]},
    )
    meshio.write(path, mesh)


def plot_case_surfaces(
    results: dict[str, dict],
    value_key: str,
    output: Path,
    title: str,
    cmap_name: str,
) -> None:
    all_values = np.concatenate([case[value_key] for case in results.values()])
    vmax = float(np.percentile(all_values, 99.0))
    vmin = 0.0
    cmap = plt.get_cmap(cmap_name)
    ncols = len(results)
    fig = plt.figure(figsize=(7.2 * ncols + 1.0, 5.4), constrained_layout=True)
    gs = fig.add_gridspec(1, ncols + 1, width_ratios=[1.0] * ncols + [0.045])
    axes = [fig.add_subplot(gs[0, i]) for i in range(ncols)]
    cax = fig.add_subplot(gs[0, -1])
    for ax, (name, case) in zip(axes, results.items()):
        points = case["points"]
        faces = case["faces"]
        owners = case["face_owners"]
        values = case[value_key][owners]

        # Orthographic longitudinal-z projection keeps the plot readable on a headless server.
        horizontal_axis = int(case["span_axis"])
        vertical_axis = 2 if horizontal_axis != 2 else 1
        polys = [points[face][:, [horizontal_axis, vertical_axis]] for face in faces]
        colors = cmap(np.clip((values - vmin) / max(vmax - vmin, 1e-12), 0.0, 1.0))
        collection = PolyCollection(polys, facecolors=colors, edgecolors="none", alpha=0.95)
        ax.add_collection(collection)
        ax.autoscale_view()
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(name)
        ax.set_xlabel(f"axis {horizontal_axis} / mm")
        ax.set_ylabel(f"axis {vertical_axis} / mm")
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=vmin, vmax=vmax))
    sm.set_array([])
    fig.colorbar(sm, cax=cax, label=title)
    fig.suptitle(title)
    fig.savefig(output, dpi=220)
    plt.close(fig)


def run() -> None:
    material = Material()
    loadcase = LoadCase()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    TMP_DIR.mkdir(parents=True, exist_ok=True)

    case_paths = load_cases()
    results: list[CaseResult] = []
    plot_payload: dict[str, dict] = {}

    for name, src in case_paths.items():
        print(f"running {name}: {src}")
        geometry_for_meshing = src
        scale = 1.0
        geometry_type = src.suffix.lower().lstrip(".")
        if name == "v3":
            msh_path = TMP_DIR / f"{name}_upper_arm_tet.msh"
            variants = [
                (
                    "full",
                    dict(
                        include_cable_channel=True,
                        include_lightening_holes=True,
                        include_mount_holes=True,
                        include_counterbores=True,
                    ),
                ),
                (
                    "no_counterbores",
                    dict(
                        include_cable_channel=True,
                        include_lightening_holes=True,
                        include_mount_holes=True,
                        include_counterbores=False,
                    ),
                ),
                (
                    "no_small_mount_holes",
                    dict(
                        include_cable_channel=True,
                        include_lightening_holes=True,
                        include_mount_holes=False,
                        include_counterbores=False,
                    ),
                ),
                (
                    "structural_core",
                    dict(
                        include_cable_channel=False,
                        include_lightening_holes=False,
                        include_mount_holes=False,
                        include_counterbores=False,
                    ),
                ),
            ]
            last_error = None
            for variant_name, variant_kwargs in variants:
                try:
                    print(f"  trying v3 FEA geometry variant: {variant_name}")
                    gmsh_mesh_v3_parametric(
                        msh_path,
                        OUT_DIR / f"v3_parametric_for_fea_{variant_name}.step",
                        loadcase.mesh_size_mm,
                        **variant_kwargs,
                    )
                    geometry_type = f"gmsh_occ_parametric_{variant_name}"
                    break
                except Exception as exc:  # noqa: BLE001 - keep trying simpler CAE variants.
                    last_error = exc
                    print(f"  variant failed: {variant_name}: {exc}")
                    if msh_path.exists():
                        msh_path.unlink()
            else:
                raise RuntimeError("All V3 FEA geometry variants failed") from last_error
        elif src.suffix.lower() not in {".step", ".stp"}:
            scaled_stl = TMP_DIR / f"{name}_upper_arm_mm.stl"
            scale = preprocess_stl_to_mm(src, scaled_stl)
            geometry_for_meshing = scaled_stl
            msh_path = TMP_DIR / f"{name}_upper_arm_tet.msh"
            gmsh_mesh_geometry(geometry_for_meshing, msh_path, loadcase.mesh_size_mm)
        else:
            msh_path = TMP_DIR / f"{name}_upper_arm_tet.msh"
            gmsh_mesh_geometry(geometry_for_meshing, msh_path, loadcase.mesh_size_mm)
        points, tetra = read_tet_mesh(msh_path)
        (
            displacement,
            vm,
            volume_mm3,
            fixed_nodes,
            loaded_nodes,
            span_axis,
        ) = solve_static_case(name, points, tetra, material, loadcase)
        disp_mag = np.linalg.norm(displacement, axis=1)
        vtk_path = OUT_DIR / f"{name}_upper_arm_static_fea.vtu"
        save_vtk(vtk_path, points, tetra, displacement, vm)
        faces, owners = boundary_faces(tetra)
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
            CaseResult(
                name=name,
                source=str(src),
                geometry_type=geometry_type,
                scale_to_mm=scale,
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
        )

    metrics_json = OUT_DIR / "upper_arm_static_fea_metrics.json"
    metrics_csv = OUT_DIR / "upper_arm_static_fea_metrics.csv"
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
        for r in results:
            writer.writerow(r.__dict__)

    plot_case_surfaces(
        plot_payload,
        "von_mises_mpa",
        OUT_DIR / "upper_arm_static_fea_von_mises.png",
        "von Mises stress / MPa",
        "inferno",
    )
    plot_case_surfaces(
        plot_payload,
        "disp_by_elem_mm",
        OUT_DIR / "upper_arm_static_fea_displacement.png",
        "displacement magnitude / mm",
        "viridis",
    )

    report = OUT_DIR / "upper_arm_static_fea_report_zh.md"
    official = next(r for r in results if r.name == "official")
    v3 = next(r for r in results if r.name == "v3")
    stress_change = (v3.max_von_mises_mpa / official.max_von_mises_mpa - 1.0) * 100.0
    disp_change = (v3.max_disp_mm / official.max_disp_mm - 1.0) * 100.0
    mass_change = (v3.mass_g / official.mass_g - 1.0) * 100.0
    report.write_text(
        f"""# SO-101 upper_arm_link Official / V3 静力有限元初步对比

## 这一步在做什么

Step 18 对官方 upper arm 和 AI 改进后的 V3 upper arm 建立相同的线性静力有限元工况，用同一套材料、网格尺寸、固定区域和载荷区域对比结构性能。

## 工况设定

- 几何对象：`official` 使用官方 STEP；`V3` 使用根据 V3 参数在 Gmsh/OpenCASCADE 中重建的 CAE 几何。
- 单位体系：mm、N、MPa。
- 材料：{material.name}，弹性模量 {material.young_mpa:.0f} MPa，泊松比 {material.poisson:.2f}，屈服强度按 {material.yield_mpa:.0f} MPa 做 screening。
- 约束：固定 shoulder 侧沿零件最长方向前 {loadcase.fixed_x_fraction:.0%} 区域的全部平动自由度。
- 载荷：在 elbow 侧沿零件最长方向后 {loadcase.loaded_x_fraction:.0%} 区域施加合力 {loadcase.total_force_n:.1f} N，方向为 Z 负方向。
- 网格：Gmsh 四面体网格，目标尺寸 {loadcase.mesh_size_mm:.1f} mm。

## 核心结果

| 版本 | 几何 | 主轴 | 节点数 | 四面体数 | 网格体积 mm^3 | 估算质量 g | 最大位移 mm | 最大 von Mises MPa | p95 von Mises MPa | 屈服安全系数 |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| official | {official.geometry_type} | {official.span_axis} | {official.node_count} | {official.tet_count} | {official.mesh_volume_mm3:.2f} | {official.mass_g:.2f} | {official.max_disp_mm:.6f} | {official.max_von_mises_mpa:.3f} | {official.p95_von_mises_mpa:.3f} | {official.safety_factor_yield:.2f} |
| V3 | {v3.geometry_type} | {v3.span_axis} | {v3.node_count} | {v3.tet_count} | {v3.mesh_volume_mm3:.2f} | {v3.mass_g:.2f} | {v3.max_disp_mm:.6f} | {v3.max_von_mises_mpa:.3f} | {v3.p95_von_mises_mpa:.3f} | {v3.safety_factor_yield:.2f} |

## 对比结论

- V3 相对 official 的网格体积/估算质量变化：{mass_change:+.2f}%。
- V3 相对 official 的最大位移变化：{disp_change:+.2f}%。
- V3 相对 official 的最大 von Mises 应力变化：{stress_change:+.2f}%。
- V3 当前可网格化 CAE 变体为 `{v3.geometry_type}`。full V3 和去沉孔版本在体网格阶段暴露出重叠边界问题，说明后续需要继续做 CAD/CAE 几何清理；本轮先保留主体、减重孔和主关节孔，暂时去掉小装配孔来完成第一版强度 screening。
- 这一步把前面的 CAD 改进从“几何与运动学验证”推进到“结构强度 screening 验证”。

## 输出文件

- 指标 JSON：`{metrics_json}`
- 指标 CSV：`{metrics_csv}`
- official VTK：`{OUT_DIR / "official_upper_arm_static_fea.vtu"}`
- V3 VTK：`{OUT_DIR / "v3_upper_arm_static_fea.vtu"}`
- 应力图：`{OUT_DIR / "upper_arm_static_fea_von_mises.png"}`
- 位移图：`{OUT_DIR / "upper_arm_static_fea_displacement.png"}`

## 当前边界

- 这是第一版开源工具链 screening FEA，边界条件采用 x 方向区域选择，还不是基于真实螺钉接触、轴承接触或舵机装配接触的高保真模型。
- STL 到四面体网格会带来离散误差，后续可用 STEP/B-Rep 特征进一步定义孔壁、接触面和载荷面。
- 结果适合用于方案筛选、作品集方法展示和下一轮设计判断，不作为最终生产强度认证。
""",
        encoding="utf-8",
    )
    print(f"wrote {report}")
    print(f"wrote {metrics_csv}")


if __name__ == "__main__":
    run()
