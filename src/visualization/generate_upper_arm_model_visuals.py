from __future__ import annotations

import json
import math
import struct
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


COLORS = {
    "official": "#7a7a7a",
    "V1": "#2f6fed",
    "V2": "#2e9d57",
    "V3": "#d97706",
}


def read_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = path.read_bytes()
    if len(data) >= 84:
        face_count = struct.unpack("<I", data[80:84])[0]
        expected = 84 + face_count * 50
        if expected <= len(data):
            vertices = np.empty((face_count * 3, 3), dtype=float)
            faces = np.empty((face_count, 3), dtype=int)
            offset = 84
            for face_idx in range(face_count):
                offset += 12
                coords = struct.unpack("<9f", data[offset : offset + 36])
                vertices[face_idx * 3 : face_idx * 3 + 3] = np.asarray(coords, dtype=float).reshape(3, 3)
                faces[face_idx] = [face_idx * 3, face_idx * 3 + 1, face_idx * 3 + 2]
                offset += 38
            return vertices, faces

    vertices_list: list[list[float]] = []
    faces_list: list[list[int]] = []
    current: list[int] = []
    for line in path.read_text(errors="ignore").splitlines():
        stripped = line.strip()
        if stripped.startswith("vertex"):
            _, x, y, z = stripped.split()
            vertices_list.append([float(x), float(y), float(z)])
            current.append(len(vertices_list) - 1)
            if len(current) == 3:
                faces_list.append(current)
                current = []
    return np.asarray(vertices_list, dtype=float), np.asarray(faces_list, dtype=int)


def mesh_triangles(vertices: np.ndarray, faces: np.ndarray, max_faces: int = 6500) -> np.ndarray:
    if len(faces) > max_faces:
        idx = np.linspace(0, len(faces) - 1, max_faces).astype(int)
        faces = faces[idx]
    return vertices[faces]


def face_shaded_colors(triangles: np.ndarray, base_hex: str) -> np.ndarray:
    rgb = np.asarray(matplotlib.colors.to_rgb(base_hex))
    v1 = triangles[:, 1] - triangles[:, 0]
    v2 = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(v1, v2)
    denom = np.linalg.norm(normals, axis=1)
    denom[denom == 0] = 1.0
    normals = normals / denom[:, None]
    light = np.asarray([0.4, -0.6, 0.7])
    light = light / np.linalg.norm(light)
    shade = np.clip(0.50 + 0.50 * (normals @ light), 0.34, 1.0)
    colors = np.empty((len(triangles), 4), dtype=float)
    colors[:, :3] = rgb[None, :] * shade[:, None]
    colors[:, 3] = 1.0
    return colors


def set_equal_axes(ax, vertices: np.ndarray, pad: float = 0.08) -> None:
    mins = vertices.min(axis=0)
    maxs = vertices.max(axis=0)
    center = (mins + maxs) / 2.0
    span = float(np.max(maxs - mins))
    span = span * (1.0 + pad)
    half = span / 2.0
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)


def draw_mesh(ax, vertices: np.ndarray, faces: np.ndarray, color: str, title: str, elev: float, azim: float) -> None:
    triangles = mesh_triangles(vertices, faces)
    collection = Poly3DCollection(
        triangles,
        facecolors=face_shaded_colors(triangles, color),
        linewidths=0.02,
        edgecolors=(0.08, 0.08, 0.08, 0.08),
    )
    ax.add_collection3d(collection)
    set_equal_axes(ax, vertices)
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, fontsize=12, pad=8)
    ax.set_axis_off()


def draw_four_version_isometric(out_path: Path, meshes: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    fig = plt.figure(figsize=(16, 5))
    for idx, (name, (vertices, faces)) in enumerate(meshes.items(), start=1):
        ax = fig.add_subplot(1, 4, idx, projection="3d")
        draw_mesh(ax, vertices, faces, COLORS[name], name, elev=24, azim=-58)
        extents = vertices.max(axis=0) - vertices.min(axis=0)
        ax.text2D(
            0.03,
            0.03,
            f"bbox {extents[0]:.1f} x {extents[1]:.1f} x {extents[2]:.1f} mm",
            transform=ax.transAxes,
            fontsize=9,
        )
    fig.suptitle("SO-101 upper_arm_link: official / V1 / V2 / V3 model comparison", fontsize=16)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def draw_three_views(out_path: Path, meshes: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    view_specs = [
        ("isometric", 24, -58),
        ("top / XY", 90, -90),
        ("side / XZ", 0, -90),
    ]
    variants = ["V1", "V2", "V3"]
    fig = plt.figure(figsize=(14, 12))
    for row, (view_name, elev, azim) in enumerate(view_specs):
        for col, variant in enumerate(variants):
            ax = fig.add_subplot(3, 3, row * 3 + col + 1, projection="3d")
            vertices, faces = meshes[variant]
            draw_mesh(ax, vertices, faces, COLORS[variant], f"{variant} - {view_name}", elev=elev, azim=azim)
    fig.suptitle("V1 / V2 / V3 structural evolution views", fontsize=16)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def draw_elbow_hole_zoom(out_path: Path, v2_params: dict, v3_params: dict) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), sharex=True, sharey=True)
    for ax, label, params, color in [
        (axes[0], "V2: interface added, clearance still tight", v2_params, COLORS["V2"]),
        (axes[1], "V3: hole pattern shifted and widened", v3_params, COLORS["V3"]),
    ]:
        center_x = float(params["elbow_servo_mount_center_x_mm"])
        spacing_x = float(params["elbow_servo_mount_spacing_x_mm"])
        spacing_y = float(params["elbow_servo_mount_spacing_y_mm"])
        hole_radius = float(params["elbow_servo_hole_radius_mm"])
        counter_radius = float(params["elbow_servo_counterbore_radius_mm"])
        pad_x = float(params.get("elbow_pad_x_mm", center_x))
        pad_sx = float(params.get("elbow_pad_size_x_mm", 28.0 if label.startswith("V2") else 30.0))
        pad_sy = float(params.get("elbow_pad_size_y_mm", 32.0 if label.startswith("V2") else 34.0))
        joint_bore = float(params["joint_bore_radius_mm"])
        span = float(params["span_shoulder_to_elbow_mm"])

        rect = plt.Rectangle(
            (pad_x - pad_sx / 2.0, -pad_sy / 2.0),
            pad_sx,
            pad_sy,
            fill=False,
            linewidth=2.0,
            color=color,
            label="elbow pad",
        )
        ax.add_patch(rect)
        ax.add_patch(plt.Circle((span, 0), joint_bore, fill=False, linewidth=2.5, color="#b91c1c", label="elbow main bore"))
        ax.add_patch(plt.Circle((span, 0), 16.5, fill=False, linewidth=1.0, linestyle=":", color="#b91c1c"))

        for dx in (-spacing_x / 2.0, spacing_x / 2.0):
            for dy in (-spacing_y / 2.0, spacing_y / 2.0):
                x = center_x + dx
                y = dy
                ax.add_patch(plt.Circle((x, y), counter_radius, fill=False, linewidth=1.4, linestyle="--", color="#f59e0b"))
                ax.add_patch(plt.Circle((x, y), hole_radius, fill=False, linewidth=1.8, color=color))

        nearest_x = center_x + spacing_x / 2.0
        material_counter = abs(span - nearest_x) - joint_bore - counter_radius
        material_hole = abs(span - nearest_x) - joint_bore - hole_radius
        ax.annotate(
            f"center x={center_x:.1f} mm\nhole r={hole_radius:.2f}\ncounter r={counter_radius:.2f}\nthrough bridge={material_hole:.2f} mm\ncounter bridge={material_counter:.2f} mm",
            xy=(nearest_x, spacing_y / 2.0),
            xytext=(88.2, 13.2),
            arrowprops={"arrowstyle": "->", "linewidth": 1.2, "color": "#111827"},
            fontsize=9.5,
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#d1d5db"},
        )
        ax.set_title(label)
        ax.set_xlabel("X / mm")
        ax.grid(True, alpha=0.25)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(86, 123)
        ax.set_ylim(-20, 20)
        ax.text(116, -18, f"through bridge {material_hole:.2f} mm", ha="center", fontsize=9)
    axes[0].set_ylabel("Y / mm")
    axes[0].legend(loc="lower left", fontsize=9)
    fig.suptitle("Elbow-side mounting hole difference: V2 -> V3", fontsize=15)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def write_markdown(path: Path, image_paths: dict[str, Path]) -> None:
    rel = {key: image_paths[key].name for key in image_paths}
    lines = [
        "# upper_arm_link 模型图对比说明",
        "",
        "## 目的",
        "",
        "把 official、V1、V2、V3 的模型差异可视化，方便直接查看结构变化，而不是只看体积和力矩数字。",
        "",
        "## 图片文件",
        "",
        f"- 四版本整体等轴视图：`{rel['isometric']}`",
        f"- V1/V2/V3 三视图演化：`{rel['three_views']}`",
        f"- V2/V3 肘部孔系局部对比：`{rel['elbow_zoom']}`",
        "",
        "## 主要观察点",
        "",
        "- V1：主要体现轻量化骨架，主关节中心距和主孔轴线保留，但装配孔系不足。",
        "- V2：增加肘部 2x2 安装孔、沉孔和肩部夹紧/定位孔，装配表达更完整。",
        "- V3：在 V2 基础上移动并放宽肘部孔系，局部 pad 加厚，用少量质量换取标准件装配余量。",
        "- official：作为开源 baseline，不要求和 AI 重建件外形完全一致，主要用于对比体积、质量和接口约束。",
        "",
        "## 当前限制",
        "",
        "- 这些图是 STL/参数的可视化渲染，不是生产图纸。",
        "- official STL 与 V1/V2/V3 的本地坐标系不完全一致，因此 official 主要作为外观和体积参照，不建议直接和 V1/V2/V3 叠加判断孔位。",
        "",
        f"![四版本整体等轴视图]({rel['isometric']})",
        "",
        f"![V1/V2/V3 三视图演化]({rel['three_views']})",
        "",
        f"![V2/V3 肘部孔系局部对比]({rel['elbow_zoom']})",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    out_dir = root / "06_portfolio_summary" / "model_visuals"
    out_dir.mkdir(parents=True, exist_ok=True)

    official_candidates = [
        root / "00_source_snapshot" / "STL_SO101" / "Individual" / "Upper_arm_SO101.stl",
        root / "05_improved_design" / "upper_arm_v3" / "urdf_smoke" / "assets" / "upper_arm_so101_v1.stl",
        root / "05_improved_design" / "upper_arm_v2" / "urdf_smoke" / "assets" / "upper_arm_so101_v1.stl",
    ]
    official_stl = next((path for path in official_candidates if path.exists()), None)
    if official_stl is None:
        raise FileNotFoundError("No official upper arm STL candidate found")

    stls = {
        "official": official_stl,
        "V1": root / "05_improved_design" / "upper_arm_v1" / "upper_arm_v1_ai_rebuild.stl",
        "V2": root / "05_improved_design" / "upper_arm_v2" / "upper_arm_v2_ai_rebuild.stl",
        "V3": root / "05_improved_design" / "upper_arm_v3" / "upper_arm_v3_ai_rebuild.stl",
    }
    meshes = {}
    for name, path in stls.items():
        vertices, faces = read_stl(path)
        extents = vertices.max(axis=0) - vertices.min(axis=0)
        if name == "official" and float(np.max(extents)) < 2.0:
            vertices = vertices * 1000.0
        meshes[name] = (vertices, faces)

    image_paths = {
        "isometric": out_dir / "upper_arm_official_v1_v2_v3_isometric.png",
        "three_views": out_dir / "upper_arm_v1_v2_v3_three_views.png",
        "elbow_zoom": out_dir / "upper_arm_v2_v3_elbow_hole_zoom.png",
    }
    draw_four_version_isometric(image_paths["isometric"], meshes)
    draw_three_views(image_paths["three_views"], meshes)

    v2_params = json.loads((root / "05_improved_design" / "upper_arm_v2" / "upper_arm_v2_parameters.json").read_text(encoding="utf-8"))
    v3_params = json.loads((root / "05_improved_design" / "upper_arm_v3" / "upper_arm_v3_parameters.json").read_text(encoding="utf-8"))
    v2_params.update({"elbow_pad_x_mm": 105.0, "elbow_pad_size_x_mm": 28.0, "elbow_pad_size_y_mm": 32.0})
    draw_elbow_hole_zoom(image_paths["elbow_zoom"], v2_params, v3_params)

    readme = out_dir / "upper_arm_model_visual_comparison_zh.md"
    write_markdown(readme, image_paths)

    print(readme)
    for path in image_paths.values():
        print(path)


if __name__ == "__main__":
    main()
