#!/usr/bin/env python3
"""Generate visual evidence package for SO-101 simulation and FEA.

The figures are portfolio-oriented: they combine CAD evolution, robot
simulation, FEA boundary modeling, solver workflow, and result plots into a
small set of high-signal images with Chinese captions.
"""

from __future__ import annotations

import csv
import math
import textwrap
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "06_portfolio_summary" / "simulation_fea_visual_package"


COLORS = {
    "ink": "#17212b",
    "muted": "#53616f",
    "line": "#d8dee6",
    "panel": "#ffffff",
    "bg": "#f5f7fa",
    "blue": "#246bfe",
    "red": "#d33f49",
    "green": "#0b8f62",
    "amber": "#b56a00",
    "purple": "#6f4cc3",
    "gray": "#6b7280",
}


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        "/System/Library/Fonts/PingFang.ttc",
        "/System/Library/Fonts/STHeiti Light.ttc",
        "/System/Library/Fonts/STHeiti Medium.ttc",
        "/Library/Fonts/Arial Unicode.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]
    for path in candidates:
        p = Path(path)
        if p.exists():
            try:
                return ImageFont.truetype(str(p), size=size, index=0)
            except OSError:
                continue
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


FONT_H1 = font(58, bold=True)
FONT_H2 = font(36, bold=True)
FONT_H3 = font(28, bold=True)
FONT_BODY = font(24)
FONT_SMALL = font(20)


def draw_wrapped(
    draw: ImageDraw.ImageDraw,
    text: str,
    xy: tuple[int, int],
    max_width: int,
    fnt: ImageFont.ImageFont,
    fill: str = COLORS["ink"],
    line_gap: int = 8,
) -> int:
    x, y = xy
    current = ""
    lines: list[str] = []
    for ch in text:
        candidate = current + ch
        if draw.textlength(candidate, font=fnt) <= max_width or not current:
            current = candidate
        else:
            lines.append(current)
            current = ch
    if current:
        lines.append(current)
    line_h = int(fnt.getbbox("Hg")[3] - fnt.getbbox("Hg")[1]) + line_gap
    for line in lines:
        draw.text((x, y), line, font=fnt, fill=fill)
        y += line_h
    return y


def contain(img: Image.Image, size: tuple[int, int], bg: str = "#ffffff") -> Image.Image:
    img = img.convert("RGB")
    w, h = img.size
    tw, th = size
    scale = min(tw / w, th / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = img.resize((nw, nh), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", size, bg)
    canvas.paste(resized, ((tw - nw) // 2, (th - nh) // 2))
    return canvas


def cover(img: Image.Image, size: tuple[int, int]) -> Image.Image:
    img = img.convert("RGB")
    w, h = img.size
    tw, th = size
    scale = max(tw / w, th / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = img.resize((nw, nh), Image.Resampling.LANCZOS)
    left = (nw - tw) // 2
    top = (nh - th) // 2
    return resized.crop((left, top, left + tw, top + th))


def draw_panel(
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    image_path: Path,
    title: str,
    caption: str,
    mode: str = "contain",
) -> None:
    draw = ImageDraw.Draw(canvas)
    x0, y0, x1, y1 = box
    draw.rounded_rectangle(box, radius=22, fill=COLORS["panel"], outline=COLORS["line"], width=2)
    draw.text((x0 + 28, y0 + 22), title, font=FONT_H3, fill=COLORS["ink"])
    caption_y = y1 - 120
    image_box = (x0 + 24, y0 + 76, x1 - 24, caption_y - 18)
    img = Image.open(image_path)
    fitted = cover(img, (image_box[2] - image_box[0], image_box[3] - image_box[1])) if mode == "cover" else contain(
        img, (image_box[2] - image_box[0], image_box[3] - image_box[1])
    )
    canvas.paste(fitted, (image_box[0], image_box[1]))
    draw.line((x0 + 24, caption_y - 4, x1 - 24, caption_y - 4), fill=COLORS["line"], width=1)
    draw_wrapped(draw, caption, (x0 + 28, caption_y + 14), x1 - x0 - 56, FONT_SMALL, COLORS["muted"], 6)


def parse_metrics(path: Path) -> dict[str, dict[str, float | str]]:
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    out: dict[str, dict[str, float | str]] = {}
    for row in rows:
        name = row["name"]
        out[name] = {}
        for k, v in row.items():
            try:
                out[name][k] = float(v)
            except (TypeError, ValueError):
                out[name][k] = v
    return out


def parse_msh_nodes(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8", errors="replace") as f:
        lines = iter(f)
        for line in lines:
            if line.strip() == "$Nodes":
                count = int(next(lines).strip())
                pts = np.zeros((count, 3), dtype=float)
                for i in range(count):
                    parts = next(lines).split()
                    pts[i] = [float(parts[1]), float(parts[2]), float(parts[3])]
                return pts
    raise ValueError(f"No $Nodes section found in {path}")


def boundary_nodes(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    extents = np.ptp(points, axis=0)
    span_axis = int(np.argmax(extents))
    amin = float(points[:, span_axis].min())
    amax = float(points[:, span_axis].max())
    span = amax - amin
    fixed = np.flatnonzero(points[:, span_axis] <= amin + 0.12 * span)
    loaded = np.flatnonzero(points[:, span_axis] >= amax - 0.10 * span)
    return fixed, loaded, span_axis


def set_equal_3d(ax: plt.Axes, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = (mins + maxs) / 2
    radius = float((maxs - mins).max() / 2)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def make_boundary_mesh_figure() -> Path:
    cases = [
        (
            "Official upper_arm",
            ROOT / "07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/tmp/official_upper_arm_clean_5mm.msh",
        ),
        (
            "Full V3 upper_arm",
            ROOT / "07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/tmp/v3_full_upper_arm_clean_5mm.msh",
        ),
    ]
    fig = plt.figure(figsize=(15, 8), dpi=180)
    for idx, (title, path) in enumerate(cases, start=1):
        pts = parse_msh_nodes(path)
        fixed, loaded, _ = boundary_nodes(pts)
        ax = fig.add_subplot(1, 2, idx, projection="3d")
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=1.5, c="#a9b0ba", alpha=0.32, depthshade=False)
        ax.scatter(pts[fixed, 0], pts[fixed, 1], pts[fixed, 2], s=8, c=COLORS["blue"], alpha=0.95, depthshade=False)
        ax.scatter(pts[loaded, 0], pts[loaded, 1], pts[loaded, 2], s=11, c=COLORS["red"], alpha=0.95, depthshade=False)
        center = pts[loaded].mean(axis=0)
        ax.quiver(center[0], center[1], center[2] + 18, 0, 0, -22, color=COLORS["red"], linewidth=2.5, arrow_length_ratio=0.25)
        ax.set_title(f"{title}\nfixed={len(fixed)}  loaded={len(loaded)}", fontsize=12, pad=12)
        ax.set_xlabel("X / mm")
        ax.set_ylabel("Y / mm")
        ax.set_zlabel("Z / mm")
        ax.view_init(elev=24, azim=-58)
        set_equal_3d(ax, pts)
        ax.grid(True, alpha=0.2)
    fig.patch.set_facecolor("white")
    plt.tight_layout()
    out = OUT_DIR / "fig03a_boundary_node_sets.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def header(canvas: Image.Image, title: str, subtitle: str) -> int:
    draw = ImageDraw.Draw(canvas)
    draw.text((70, 50), title, font=FONT_H1, fill=COLORS["ink"])
    draw_wrapped(draw, subtitle, (72, 128), canvas.size[0] - 144, FONT_BODY, COLORS["muted"], 8)
    return 190


def make_overview_board() -> Path:
    w, h = 3200, 2240
    canvas = Image.new("RGB", (w, h), COLORS["bg"])
    top = header(
        canvas,
        "SO-101 仿真与有限元证据总览",
        "从官方模型复现、工作空间/力矩分析、AI 辅助结构迭代，到 full V3 clean mesh 的 FEA 结果，形成可讲述的工程闭环。",
    )
    margin, gap = 58, 34
    panel_w = (w - 2 * margin - 2 * gap) // 3
    panel_h = 930
    y1 = top + 30
    y2 = y1 + panel_h + gap
    items = [
        (
            "官方 URDF/PyBullet baseline",
            ROOT / "02_baseline_validation/baseline_preview.png",
            "证明官方 SO-101 不是纸面模型，而是已经能加载进物理仿真环境。",
        ),
        (
            "工作空间可达性",
            ROOT / "04_structural_analysis/baseline_workspace_plot.png",
            "用末端采样点云解释机械臂能覆盖哪些桌面任务区域。",
        ),
        (
            "静载关节力矩",
            ROOT / "04_structural_analysis/baseline_static_torque_plot.png",
            "定位 shoulder_lift、elbow_flex 等主要受力瓶颈，为结构优化提供依据。",
        ),
        (
            "结构瓶颈筛查",
            ROOT / "04_structural_analysis/baseline_structural_bottleneck_plot.png",
            "把力矩、杆长和简化梁模型结合，确定第一轮改进对象为 upper_arm_link。",
        ),
        (
            "V3 接回 URDF/PyBullet",
            ROOT / "05_improved_design/upper_arm_v3/urdf_smoke/upper_arm_v3_urdf_smoke_plot.png",
            "验证改进件接回机器人模型后没有破坏关节链和末端运动学。",
        ),
        (
            "Official / V1 / V2 / V3 结构演化",
            ROOT / "06_portfolio_summary/model_visuals/upper_arm_official_v1_v2_v3_isometric.png",
            "展示 AI 辅助 CAD 不是一次生成，而是经过装配孔、余量、FEA 问题逐步迭代。",
        ),
    ]
    for i, (title, img, cap) in enumerate(items):
        row, col = divmod(i, 3)
        x0 = margin + col * (panel_w + gap)
        y0 = y1 if row == 0 else y2
        draw_panel(canvas, (x0, y0, x0 + panel_w, y0 + panel_h), img, title, cap)
    out = OUT_DIR / "fig01_simulation_fea_evidence_overview.png"
    canvas.save(out, quality=95)
    return out


def make_cad_evolution_board() -> Path:
    w, h = 3200, 2360
    canvas = Image.new("RGB", (w, h), COLORS["bg"])
    top = header(
        canvas,
        "AI 辅助 CAD 建模与结构迭代图",
        "这张图用于说明建模工作量：从官方 upper_arm 到 V1/V2/V3，逐步加入轻量化、装配孔、局部余量修正和标准件校核。",
    )
    draw = ImageDraw.Draw(canvas)
    blocks = [
        (
            "四版本等轴视图",
            ROOT / "06_portfolio_summary/model_visuals/upper_arm_official_v1_v2_v3_isometric.png",
            "先看整体形态和质量变化。",
        ),
        (
            "V1 / V2 / V3 三视图",
            ROOT / "06_portfolio_summary/model_visuals/upper_arm_v1_v2_v3_three_views.png",
            "再看孔系、梁、肋板和外形边界的结构演化。",
        ),
        (
            "V2 / V3 肘部孔系局部对比",
            ROOT / "06_portfolio_summary/model_visuals/upper_arm_v2_v3_elbow_hole_zoom.png",
            "V3 的明确改进点：解决 V2 肘部 M3 孔与主孔之间材料余量过小的问题。",
        ),
    ]
    y = top + 24
    heights = [470, 1010, 560]
    for (title, img, cap), bh in zip(blocks, heights, strict=True):
        box = (60, y, w - 60, y + bh)
        draw.rounded_rectangle(box, radius=22, fill=COLORS["panel"], outline=COLORS["line"], width=2)
        draw.text((90, y + 20), title, font=FONT_H3, fill=COLORS["ink"])
        draw_wrapped(draw, cap, (90, y + 62), 760, FONT_SMALL, COLORS["muted"], 6)
        image_area = (900, y + 22, w - 90, y + bh - 22)
        fitted = contain(Image.open(img), (image_area[2] - image_area[0], image_area[3] - image_area[1]))
        canvas.paste(fitted, (image_area[0], image_area[1]))
        y += bh + 28
    out = OUT_DIR / "fig02_cad_modeling_evolution.png"
    canvas.save(out, quality=95)
    return out


def make_fea_modeling_board(boundary_img: Path) -> Path:
    metrics = parse_metrics(ROOT / "07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/upper_arm_calculix_smoke_metrics.csv")
    w, h = 3200, 1880
    canvas = Image.new("RGB", (w, h), COLORS["bg"])
    top = header(
        canvas,
        "有限元前处理建模图：网格、边界条件、求解器输入",
        "这张图回答“有限元怎么建模”：使用 5 mm 四面体网格，shoulder 侧节点集合固定，elbow 侧节点集合施加 10 N 向下合力，再导出 CalculiX .inp 进行求解。",
    )
    draw = ImageDraw.Draw(canvas)
    left = (62, top + 28, 2080, h - 72)
    draw.rounded_rectangle(left, radius=22, fill=COLORS["panel"], outline=COLORS["line"], width=2)
    draw.text((92, top + 52), "边界条件节点集可视化", font=FONT_H3, fill=COLORS["ink"])
    fitted = contain(Image.open(boundary_img), (left[2] - left[0] - 64, left[3] - left[1] - 150))
    canvas.paste(fitted, (left[0] + 32, left[1] + 106))
    draw_wrapped(
        draw,
        "蓝色为固定端节点集，红色为加载端节点集，红色箭头表示 Z 负方向 10 N 合力。这个图比单独写公式更直观，能说明边界条件不是随便设的。",
        (left[0] + 34, left[3] - 104),
        left[2] - left[0] - 68,
        FONT_SMALL,
        COLORS["muted"],
    )

    right = (2120, top + 28, w - 62, h - 72)
    draw.rounded_rectangle(right, radius=22, fill=COLORS["panel"], outline=COLORS["line"], width=2)
    draw.text((right[0] + 34, right[1] + 30), "建模参数与求解结果", font=FONT_H3, fill=COLORS["ink"])
    bullets = [
        "几何对象：official upper_arm 与 full V3 upper_arm",
        "网格：Gmsh 5 mm 四面体，C3D4 一阶四面体单元",
        "材料：PLA screening，E=2500 MPa，nu=0.35",
        "约束：shoulder 侧节点集合 X/Y/Z 三向固定",
        "载荷：elbow 侧节点集合均布 10 N 向下力",
        "求解链：Gmsh .msh -> CalculiX .inp -> ccx -> .dat/.frd",
    ]
    y = right[1] + 96
    for b in bullets:
        draw.ellipse((right[0] + 40, y + 9, right[0] + 50, y + 19), fill=COLORS["blue"])
        y = draw_wrapped(draw, b, (right[0] + 66, y), right[2] - right[0] - 100, FONT_BODY, COLORS["ink"], 8) + 8

    y += 24
    headers = ["版本", "节点/四面体", "最大位移", "最大应力", "安全系数"]
    rows = []
    for key, label in [("official", "Official"), ("v3_full", "Full V3")]:
        m = metrics[key]
        rows.append(
            [
                label,
                f"{int(m['node_count'])}/{int(m['tet_count'])}",
                f"{m['max_loaded_disp_mm']:.6f} mm",
                f"{m['max_von_mises_mpa']:.3f} MPa",
                f"{m['safety_factor_yield']:.2f}",
            ]
        )
    table_x = right[0] + 34
    table_y = y
    col_w = [142, 190, 180, 170, 132]
    row_h = 58
    draw.rounded_rectangle((table_x, table_y, right[2] - 34, table_y + row_h * 3), radius=14, fill="#fbfcfe", outline=COLORS["line"])
    x = table_x
    for i, hd in enumerate(headers):
        draw.text((x + 12, table_y + 15), hd, font=FONT_SMALL, fill=COLORS["ink"])
        x += col_w[i]
    draw.line((table_x, table_y + row_h, right[2] - 34, table_y + row_h), fill=COLORS["line"], width=2)
    for r, row in enumerate(rows):
        x = table_x
        yy = table_y + row_h * (r + 1)
        for i, val in enumerate(row):
            fill = COLORS["red"] if r == 1 and i in (2, 3) else COLORS["ink"]
            draw.text((x + 12, yy + 15), val, font=FONT_SMALL, fill=fill)
            x += col_w[i]
    y = table_y + row_h * 3 + 44
    draw_wrapped(
        draw,
        "结论：V3 虽然更轻，但在当前工况下位移和应力都高于 official，因此后续 V4 不能继续盲目减重，应围绕中部梁、上下梁连接和肋板布局做刚度回补。",
        (right[0] + 34, y),
        right[2] - right[0] - 68,
        FONT_BODY,
        COLORS["amber"],
        8,
    )
    out = OUT_DIR / "fig03_fea_modeling_boundary_conditions.png"
    canvas.save(out, quality=95)
    return out


def make_fea_results_board() -> Path:
    w, h = 3200, 1880
    canvas = Image.new("RGB", (w, h), COLORS["bg"])
    header(
        canvas,
        "有限元结果图：应力云图与位移云图",
        "这张图用于对外展示 FEA 结果本身：同样材料、同样 10 N 工况、同样 5 mm 网格口径下，比较 official 与 full V3 的应力和位移响应。",
    )
    draw = ImageDraw.Draw(canvas)
    top = 230
    panel_h = 720
    items = [
        (
            "von Mises 应力对比",
            ROOT / "07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_von_mises.png",
            "V3 最大应力 0.770 MPa，高于 official 的 0.327 MPa。虽然安全系数仍较高，但说明轻量化带来局部应力上升。",
        ),
        (
            "总位移对比",
            ROOT / "07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_displacement.png",
            "V3 最大位移 0.071577 mm，高于 official 的 0.024591 mm。后续优化重点是提高刚度，而不是继续减重。",
        ),
    ]
    for i, (title, img, cap) in enumerate(items):
        y = top + i * (panel_h + 42)
        draw_panel(canvas, (62, y, w - 62, y + panel_h), img, title, cap)
    out = OUT_DIR / "fig04_fea_results_stress_displacement.png"
    canvas.save(out, quality=95)
    return out


def arrow(draw: ImageDraw.ImageDraw, p0: tuple[int, int], p1: tuple[int, int], color: str) -> None:
    draw.line((p0, p1), fill=color, width=5)
    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    for da in (2.55, -2.55):
        q = (p1[0] - 22 * math.cos(ang + da), p1[1] - 22 * math.sin(ang + da))
        draw.line((p1, q), fill=color, width=5)


def make_workflow_diagram() -> Path:
    w, h = 3200, 1120
    canvas = Image.new("RGB", (w, h), COLORS["bg"])
    top = header(
        canvas,
        "开源仿真与有限元工作流",
        "这张图用于讲述完整方法论：不是单独跑一个软件，而是把开源 CAD、机器人仿真、网格、求解器和结果分析串成闭环。",
    )
    draw = ImageDraw.Draw(canvas)
    labels = [
        ("官方 SO-101\nSTEP/STL/URDF", "复现 baseline"),
        ("AI 参数化 CAD\nV1/V2/V3", "结构迭代"),
        ("URDF/PyBullet", "运动学 smoke test"),
        ("Gmsh 5 mm mesh", "CAE 前处理"),
        ("CalculiX ccx", ".inp 求解"),
        (".dat/.frd/CSV/图", "结果解析"),
        ("V4 结构加强", "下一轮优化"),
    ]
    box_w, box_h = 360, 210
    gap = 70
    x = 85
    y = top + 150
    centers = []
    for i, (main, sub) in enumerate(labels):
        color = [COLORS["blue"], COLORS["purple"], COLORS["green"], COLORS["amber"], COLORS["red"], COLORS["gray"], COLORS["blue"]][i]
        box = (x, y, x + box_w, y + box_h)
        draw.rounded_rectangle(box, radius=24, fill=COLORS["panel"], outline=color, width=4)
        lines = main.split("\n")
        ty = y + 42
        for line in lines:
            tw = draw.textlength(line, font=FONT_H3)
            draw.text((x + (box_w - tw) / 2, ty), line, font=FONT_H3, fill=COLORS["ink"])
            ty += 42
        tw = draw.textlength(sub, font=FONT_SMALL)
        draw.text((x + (box_w - tw) / 2, y + box_h - 48), sub, font=FONT_SMALL, fill=COLORS["muted"])
        centers.append((x + box_w, y + box_h // 2))
        if i > 0:
            prev = (x - gap, y + box_h // 2)
            arrow(draw, prev, (x - 12, y + box_h // 2), COLORS["line"])
        x += box_w + gap
    note = (
        "工程验证重点：先复现官方开源机械臂，再用 AI 辅助进行结构重建和改进；每一版都要经过 CAD 校核、"
        "URDF/PyBullet 回归、Gmsh 网格、CalculiX 求解和结果对比，最后用结果反推下一版结构。"
    )
    draw_wrapped(draw, note, (100, h - 210), w - 200, FONT_BODY, COLORS["ink"], 10)
    out = OUT_DIR / "fig05_open_source_simulation_cae_workflow.png"
    canvas.save(out, quality=95)
    return out


def write_report(figs: dict[str, Path]) -> Path:
    report = OUT_DIR / "simulation_fea_visual_package_zh.md"
    rel = {k: v.name for k, v in figs.items()}
    report.write_text(
        f"""# SO-101 仿真与有限元分析图包

## 这一步在做什么

本步骤把前面已经完成的 CAD 建模、机器人仿真、工作空间分析、静载力矩分析、FEA 前处理、CalculiX 求解和结果云图整理成一组工程说明图片。

目标不是新增一个单独数值，而是增强项目说服力：让别人一眼看到“模型从哪里来、怎么改、怎么接回仿真、有限元怎么建模、结果如何指导下一轮设计”。

## 生成图片

### 1. 仿真与有限元证据总览

![SO-101 仿真与有限元证据总览]({rel['overview']})

用途：总览图，适合放在项目汇报第一页，说明项目覆盖了 baseline 复现、运动仿真、结构分析、CAD 迭代和 FEA。

### 2. AI 辅助 CAD 建模与结构迭代

![AI 辅助 CAD 建模与结构迭代]({rel['cad']})

用途：说明 upper_arm 不是一次性生成，而是经历 V1 轻量化、V2 装配孔补全、V3 孔系余量修正。

### 3. 有限元前处理建模图

![有限元前处理建模图]({rel['fea_model']})

用途：回答“有限元怎么建模”。图中明确展示了网格、固定端、加载端、10 N 载荷方向、材料和求解器链路。

### 4. 有限元结果图

![有限元结果图]({rel['fea_result']})

用途：展示应力云图和位移云图。核心结论是 V3 轻量化有效，但刚度下降明显，V4 应该做局部加强。

### 5. 开源仿真与 CAE 工作流

![开源仿真与 CAE 工作流]({rel['workflow']})

用途：说明项目不是孤立脚本，而是开源 CAD/URDF/PyBullet/Gmsh/CalculiX 的组合工程流程。

## 工程流程说明

流程先复现官方 SO-101 的 URDF 和 PyBullet 仿真，确认关节链和末端运动正常；然后用工作空间和静载力矩分析定位结构瓶颈，确定 upper_arm 是第一轮改进对象。之后使用 AI 辅助参数化 CAD 重建，经历 V1、V2、V3 三轮迭代，并把改进件接回 URDF 做运动学回归。最后将 official 和 full V3 的 STEP 几何转成 Gmsh 四面体网格，再导出 CalculiX 输入文件做静力有限元，得到应力和位移对比。结果显示 V3 质量降低，但刚度下降，所以下一版 V4 的目标不是继续减重，而是加强中部梁、上下梁连接和肋板布局。

## 当前边界

- 当前 FEA 是线性静力 screening，不是最终强度认证。
- 当前边界条件为节点集合固定/加载，还没有做真实螺钉接触、轴承接触和预紧。
- 当前材料为 PLA screening 参数，后续可以加入打印方向、层间强度和材料各向异性。
- 当前图包已经足够用于阶段性作品集，但后续 V4 应增加“改进前后 FEA 云图对比”和“网格收敛图”。
""",
        encoding="utf-8",
    )
    return report


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    boundary = make_boundary_mesh_figure()
    figs = {
        "overview": make_overview_board(),
        "cad": make_cad_evolution_board(),
        "fea_model": make_fea_modeling_board(boundary),
        "fea_result": make_fea_results_board(),
        "workflow": make_workflow_diagram(),
    }
    figs["boundary_raw"] = boundary
    report = write_report(figs)
    print(f"wrote {report}")
    for fig in figs.values():
        print(f"wrote {fig}")


if __name__ == "__main__":
    main()
