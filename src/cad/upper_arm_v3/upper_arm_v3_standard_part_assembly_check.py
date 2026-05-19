from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

from build123d import Box, Compound, Cylinder, Pos, export_step, export_stl


SCRIPT_DIR = Path(__file__).resolve().parent
V2_DIR = SCRIPT_DIR.parents[0] / "upper_arm_v2"
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(V2_DIR))

import upper_arm_v3_cad as cad  # noqa: E402


# Simplified standard-part assumptions, in millimeters.
MAIN_SHAFT_RADIUS_MM = 4.0
MAIN_SHAFT_LENGTH_MM = 44.0

M3_SHANK_RADIUS_MM = 1.5
M3_HEAD_RADIUS_MM = 2.6
M3_HEAD_HEIGHT_MM = 2.0

SHOULDER_CLAMP_HEAD_RADIUS_MM = 3.0
SHOULDER_CLAMP_HEAD_HEIGHT_MM = 3.0

SERVO_REFERENCE_PLATE_X_MM = 22.0
SERVO_REFERENCE_PLATE_Y_MM = 22.0
SERVO_REFERENCE_PLATE_Z_MM = 8.0

FUNCTIONAL_CLEARANCE_MIN_MM = 0.05
PRINT_CLEARANCE_RECOMMENDED_MM = 0.20
MIN_MATERIAL_RECOMMENDED_MM = 2.0
NEAR_BORE_RECOMMENDED_MM = 0.8


def y_cylinder(radius: float, length: float):
    return Cylinder(radius=radius, height=length, rotation=(90, 0, 0))


def z_cylinder(radius: float, length: float):
    return Cylinder(radius=radius, height=length)


def read_params() -> dict[str, Any]:
    return json.loads((SCRIPT_DIR / "upper_arm_v3_parameters.json").read_text(encoding="utf-8"))


def p(params: dict[str, Any], key: str, fallback: float) -> float:
    return float(params.get(key, fallback))


def elbow_screw_centers(params: dict[str, Any]) -> list[tuple[float, float]]:
    center_x = float(params["elbow_servo_mount_center_x_mm"])
    half_x = float(params["elbow_servo_mount_spacing_x_mm"]) / 2.0
    half_y = float(params["elbow_servo_mount_spacing_y_mm"]) / 2.0
    return [(center_x + dx, dy) for dx in (-half_x, half_x) for dy in (-half_y, half_y)]


def simplified_standard_parts(params: dict[str, Any]):
    parts = []

    span = float(params["span_shoulder_to_elbow_mm"])
    parts.append(Pos(0, 0, 0) * y_cylinder(MAIN_SHAFT_RADIUS_MM, MAIN_SHAFT_LENGTH_MM))
    parts.append(Pos(span, 0, 0) * y_cylinder(MAIN_SHAFT_RADIUS_MM, MAIN_SHAFT_LENGTH_MM))

    elbow_pad_z = p(params, "elbow_pad_size_z_mm", cad.base.ELBOW_PAD_SIZE_Z_MM)
    top_z = elbow_pad_z / 2.0
    bottom_z = -elbow_pad_z / 2.0
    through_length = elbow_pad_z + 8.0
    for x_pos, y_pos in elbow_screw_centers(params):
        parts.append(Pos(x_pos, y_pos, 0) * z_cylinder(M3_SHANK_RADIUS_MM, through_length))
        parts.append(Pos(x_pos, y_pos, top_z - M3_HEAD_HEIGHT_MM / 2.0) * z_cylinder(M3_HEAD_RADIUS_MM, M3_HEAD_HEIGHT_MM))
        parts.append(Pos(x_pos, y_pos, bottom_z + M3_HEAD_HEIGHT_MM / 2.0) * z_cylinder(M3_HEAD_RADIUS_MM, M3_HEAD_HEIGHT_MM))

    shoulder_x = float(params["shoulder_clamp_hole_x_mm"])
    shoulder_z = float(params["shoulder_clamp_hole_z_mm"])
    shoulder_pad_y = cad.base.SHOULDER_PAD_SIZE_Y_MM
    for z_pos in (-shoulder_z, shoulder_z):
        parts.append(Pos(shoulder_x, 0, z_pos) * y_cylinder(M3_SHANK_RADIUS_MM, shoulder_pad_y + 8.0))
        parts.append(
            Pos(shoulder_x, shoulder_pad_y / 2.0 + SHOULDER_CLAMP_HEAD_HEIGHT_MM / 2.0, z_pos)
            * y_cylinder(SHOULDER_CLAMP_HEAD_RADIUS_MM, SHOULDER_CLAMP_HEAD_HEIGHT_MM)
        )
        parts.append(
            Pos(shoulder_x, -shoulder_pad_y / 2.0 - SHOULDER_CLAMP_HEAD_HEIGHT_MM / 2.0, z_pos)
            * y_cylinder(SHOULDER_CLAMP_HEAD_RADIUS_MM, SHOULDER_CLAMP_HEAD_HEIGHT_MM)
        )

    plate_z_center = top_z + SERVO_REFERENCE_PLATE_Z_MM / 2.0
    parts.append(
        Pos(float(params["elbow_servo_mount_center_x_mm"]), 0, plate_z_center)
        * Box(SERVO_REFERENCE_PLATE_X_MM, SERVO_REFERENCE_PLATE_Y_MM, SERVO_REFERENCE_PLATE_Z_MM)
    )
    return parts


def check(name: str, requirement: str, measured: str, value: float | None, status: str, note: str) -> dict[str, Any]:
    return {
        "name": name,
        "requirement": requirement,
        "measured": measured,
        "value": value,
        "status": status,
        "note": note,
    }


def clearance_status(clearance: float, functional_min: float = FUNCTIONAL_CLEARANCE_MIN_MM) -> str:
    if clearance < functional_min:
        return "FAIL"
    if clearance < PRINT_CLEARANCE_RECOMMENDED_MM:
        return "WARN"
    return "PASS"


def material_status(material: float, recommended: float = MIN_MATERIAL_RECOMMENDED_MM) -> str:
    return "PASS" if material >= recommended else "WARN"


def build_checks(params: dict[str, Any]) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []

    joint_bore = float(params["joint_bore_radius_mm"])
    main_shaft_clearance = joint_bore - MAIN_SHAFT_RADIUS_MM
    checks.append(
        check(
            "主关节 D8 轴/轴套径向余量",
            f"孔半径 {joint_bore:.2f} mm，应大于标准件半径 {MAIN_SHAFT_RADIUS_MM:.2f} mm",
            f"{main_shaft_clearance:.3f} mm",
            main_shaft_clearance,
            clearance_status(main_shaft_clearance),
            "用于判断 shoulder/elbow 主轴或轴套是否能装入。",
        )
    )

    elbow_hole_radius = float(params["elbow_servo_hole_radius_mm"])
    elbow_shank_clearance = elbow_hole_radius - M3_SHANK_RADIUS_MM
    checks.append(
        check(
            "肘部 M3 螺钉通孔径向余量",
            f"孔半径 {elbow_hole_radius:.2f} mm，应大于 M3 螺钉半径 {M3_SHANK_RADIUS_MM:.2f} mm",
            f"{elbow_shank_clearance:.3f} mm",
            elbow_shank_clearance,
            clearance_status(elbow_shank_clearance),
            "V3 已将 V2 的偏紧通孔放宽，给 3D 打印误差留出更稳定余量。",
        )
    )

    counterbore_radius = float(params["elbow_servo_counterbore_radius_mm"])
    counterbore_depth = float(params["elbow_servo_counterbore_depth_mm"])
    head_radial_clearance = counterbore_radius - M3_HEAD_RADIUS_MM
    head_depth_clearance = counterbore_depth - M3_HEAD_HEIGHT_MM
    checks.append(
        check(
            "肘部 M3 螺钉头沉孔径向余量",
            f"沉孔半径 {counterbore_radius:.2f} mm，应大于头部半径 {M3_HEAD_RADIUS_MM:.2f} mm",
            f"{head_radial_clearance:.3f} mm",
            head_radial_clearance,
            clearance_status(head_radial_clearance),
            "V3 已放宽沉孔半径，降低实际螺钉头装不进去的风险。",
        )
    )
    checks.append(
        check(
            "肘部 M3 螺钉头沉孔深度余量",
            f"沉孔深度 {counterbore_depth:.2f} mm，应大于头部高度 {M3_HEAD_HEIGHT_MM:.2f} mm",
            f"{head_depth_clearance:.3f} mm",
            head_depth_clearance,
            clearance_status(head_depth_clearance),
            "沉孔深度可容纳简化螺钉头。",
        )
    )

    elbow_pad_center_x = p(params, "elbow_pad_x_mm", cad.base.ELBOW_PAD_X_MM)
    elbow_pad_size_x = p(params, "elbow_pad_size_x_mm", cad.base.ELBOW_PAD_SIZE_X_MM)
    elbow_pad_size_y = p(params, "elbow_pad_size_y_mm", cad.base.ELBOW_PAD_SIZE_Y_MM)
    pad_min_x = elbow_pad_center_x - elbow_pad_size_x / 2.0
    pad_max_x = elbow_pad_center_x + elbow_pad_size_x / 2.0
    pad_min_y = -elbow_pad_size_y / 2.0
    pad_max_y = elbow_pad_size_y / 2.0
    min_shank_edge = min(
        min(x - pad_min_x, pad_max_x - x, y - pad_min_y, pad_max_y - y) - elbow_hole_radius
        for x, y in elbow_screw_centers(params)
    )
    min_head_edge = min(
        min(x - pad_min_x, pad_max_x - x, y - pad_min_y, pad_max_y - y) - counterbore_radius
        for x, y in elbow_screw_centers(params)
    )
    checks.append(
        check(
            "肘部安装孔到 pad 边缘最小材料",
            "建议 >= 2.0 mm",
            f"通孔边缘 {min_shank_edge:.3f} mm，沉孔边缘 {min_head_edge:.3f} mm",
            min(min_shank_edge, min_head_edge),
            material_status(min(min_shank_edge, min_head_edge)),
            "V3 同时加大 elbow pad，使放宽孔径后仍保留边缘材料。",
        )
    )

    spacing_x = float(params["elbow_servo_mount_spacing_x_mm"])
    spacing_y = float(params["elbow_servo_mount_spacing_y_mm"])
    min_spacing = min(spacing_x, spacing_y)
    ligament_shank = min_spacing - 2.0 * elbow_hole_radius
    ligament_head = min_spacing - 2.0 * counterbore_radius
    checks.append(
        check(
            "肘部 2x2 孔间最小材料",
            "建议通孔/沉孔之间保留 >= 2.0 mm",
            f"通孔间 {ligament_shank:.3f} mm，沉孔间 {ligament_head:.3f} mm",
            min(ligament_shank, ligament_head),
            material_status(min(ligament_shank, ligament_head)),
            "孔系自身材料余量仍可接受。",
        )
    )

    span = float(params["span_shoulder_to_elbow_mm"])
    nearest_elbow_x = max(x for x, _ in elbow_screw_centers(params))
    elbow_mount_to_main_bore_shank = abs(span - nearest_elbow_x) - joint_bore - elbow_hole_radius
    elbow_mount_to_main_bore_head = abs(span - nearest_elbow_x) - joint_bore - counterbore_radius
    checks.append(
        check(
            "肘部安装孔与 elbow 主孔最小孔间材料",
            f"保守按沉孔外缘建议 >= {NEAR_BORE_RECOMMENDED_MM:.1f} mm",
            f"通孔外缘 {elbow_mount_to_main_bore_shank:.3f} mm，沉孔外缘 {elbow_mount_to_main_bore_head:.3f} mm",
            elbow_mount_to_main_bore_head,
            "PASS" if elbow_mount_to_main_bore_head >= NEAR_BORE_RECOMMENDED_MM else "WARN",
            "这是 Step 14 的主要风险项。V3 将孔系向近端移动后，最近 M3 孔与 elbow 主孔之间的材料桥明显增加。",
        )
    )

    shoulder_hole_radius = float(params["shoulder_clamp_hole_radius_mm"])
    shoulder_clearance = shoulder_hole_radius - M3_SHANK_RADIUS_MM
    checks.append(
        check(
            "肩部 M3 夹紧/定位孔径向余量",
            f"孔半径 {shoulder_hole_radius:.2f} mm，应大于 M3 螺钉半径 {M3_SHANK_RADIUS_MM:.2f} mm",
            f"{shoulder_clearance:.3f} mm",
            shoulder_clearance,
            clearance_status(shoulder_clearance),
            "肩部侧向孔给 M3 螺钉留有较宽松余量。",
        )
    )

    shoulder_x = float(params["shoulder_clamp_hole_x_mm"])
    shoulder_z = float(params["shoulder_clamp_hole_z_mm"])
    shoulder_min_x = cad.base.SHOULDER_PAD_X_MM - cad.base.SHOULDER_PAD_SIZE_X_MM / 2.0
    shoulder_max_x = cad.base.SHOULDER_PAD_X_MM + cad.base.SHOULDER_PAD_SIZE_X_MM / 2.0
    shoulder_min_z = -cad.base.SHOULDER_PAD_SIZE_Z_MM / 2.0
    shoulder_max_z = cad.base.SHOULDER_PAD_SIZE_Z_MM / 2.0
    shoulder_edge = min(
        shoulder_x - shoulder_min_x,
        shoulder_max_x - shoulder_x,
        shoulder_z - shoulder_min_z,
        shoulder_max_z - shoulder_z,
    ) - shoulder_hole_radius
    checks.append(
        check(
            "肩部夹紧孔到 pad 边缘最小材料",
            "建议 >= 2.0 mm",
            f"{shoulder_edge:.3f} mm",
            shoulder_edge,
            material_status(shoulder_edge),
            "V3 没有改肩部接口，该项继承 V2 的可接受结果。",
        )
    )

    shoulder_to_main = math.sqrt(shoulder_x**2 + shoulder_z**2) - joint_bore - shoulder_hole_radius
    checks.append(
        check(
            "肩部夹紧孔与 shoulder 主孔最小孔间材料",
            "建议 >= 2.0 mm",
            f"{shoulder_to_main:.3f} mm",
            shoulder_to_main,
            material_status(shoulder_to_main),
            "肩部夹紧孔与主孔之间有较充足材料。",
        )
    )

    plate_min_x = elbow_pad_center_x - SERVO_REFERENCE_PLATE_X_MM / 2.0
    plate_max_x = elbow_pad_center_x + SERVO_REFERENCE_PLATE_X_MM / 2.0
    plate_min_y = -SERVO_REFERENCE_PLATE_Y_MM / 2.0
    plate_max_y = SERVO_REFERENCE_PLATE_Y_MM / 2.0
    plate_margin = min(
        plate_min_x - pad_min_x,
        pad_max_x - plate_max_x,
        plate_min_y - pad_min_y,
        pad_max_y - plate_max_y,
    )
    checks.append(
        check(
            "简化舵机/连接板 footprint 是否落在 elbow pad 内",
            "参考板投影应在 pad 内",
            f"最小边缘余量 {plate_margin:.3f} mm",
            plate_margin,
            "PASS" if plate_margin >= 0.0 else "FAIL",
            "用于检查简化 mating plate 不会明显悬空。",
        )
    )

    return checks


def overall_status(checks: list[dict[str, Any]]) -> str:
    statuses = {item["status"] for item in checks}
    if "FAIL" in statuses:
        return "FAIL"
    if "WARN" in statuses:
        return "PASS_WITH_WARNINGS"
    return "PASS"


def write_checks_csv(path: Path, checks: list[dict[str, Any]]) -> None:
    fields = ["name", "requirement", "measured", "value", "status", "note"]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(checks)


def maybe_write_plot(path: Path, params: dict[str, Any], checks: list[dict[str, Any]]) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    fig, (ax_elbow, ax_shoulder) = plt.subplots(1, 2, figsize=(12, 5))

    elbow_pad_center_x = p(params, "elbow_pad_x_mm", cad.base.ELBOW_PAD_X_MM)
    elbow_pad_size_x = p(params, "elbow_pad_size_x_mm", cad.base.ELBOW_PAD_SIZE_X_MM)
    elbow_pad_size_y = p(params, "elbow_pad_size_y_mm", cad.base.ELBOW_PAD_SIZE_Y_MM)
    ax_elbow.add_patch(
        plt.Rectangle(
            (elbow_pad_center_x - elbow_pad_size_x / 2.0, -elbow_pad_size_y / 2.0),
            elbow_pad_size_x,
            elbow_pad_size_y,
            fill=False,
            linewidth=2,
            label="elbow pad",
        )
    )
    for x_pos, y_pos in elbow_screw_centers(params):
        ax_elbow.add_patch(
            plt.Circle((x_pos, y_pos), float(params["elbow_servo_hole_radius_mm"]), fill=False, color="tab:blue")
        )
        ax_elbow.add_patch(
            plt.Circle(
                (x_pos, y_pos),
                float(params["elbow_servo_counterbore_radius_mm"]),
                fill=False,
                color="tab:orange",
                linestyle="--",
            )
        )
    ax_elbow.add_patch(
        plt.Circle((float(params["span_shoulder_to_elbow_mm"]), 0), float(params["joint_bore_radius_mm"]), fill=False, color="tab:red")
    )
    ax_elbow.set_title("V3 elbow mounting pattern, XY view")
    ax_elbow.set_xlabel("x / mm")
    ax_elbow.set_ylabel("y / mm")
    ax_elbow.axis("equal")
    ax_elbow.grid(True, alpha=0.3)

    ax_shoulder.add_patch(
        plt.Rectangle(
            (
                cad.base.SHOULDER_PAD_X_MM - cad.base.SHOULDER_PAD_SIZE_X_MM / 2.0,
                -cad.base.SHOULDER_PAD_SIZE_Z_MM / 2.0,
            ),
            cad.base.SHOULDER_PAD_SIZE_X_MM,
            cad.base.SHOULDER_PAD_SIZE_Z_MM,
            fill=False,
            linewidth=2,
            label="shoulder pad",
        )
    )
    shoulder_x = float(params["shoulder_clamp_hole_x_mm"])
    shoulder_z = float(params["shoulder_clamp_hole_z_mm"])
    for z_pos in (-shoulder_z, shoulder_z):
        ax_shoulder.add_patch(
            plt.Circle((shoulder_x, z_pos), float(params["shoulder_clamp_hole_radius_mm"]), fill=False, color="tab:blue")
        )
    ax_shoulder.add_patch(plt.Circle((0, 0), float(params["joint_bore_radius_mm"]), fill=False, color="tab:red"))
    ax_shoulder.set_title("V3 shoulder clamp holes, XZ view")
    ax_shoulder.set_xlabel("x / mm")
    ax_shoulder.set_ylabel("z / mm")
    ax_shoulder.axis("equal")
    ax_shoulder.grid(True, alpha=0.3)

    fig.suptitle(f"upper_arm_v3 simplified standard-part assembly check: {overall_status(checks)}")
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return True


def write_report(
    path: Path,
    params: dict[str, Any],
    checks: list[dict[str, Any]],
    result_json: Path,
    checks_csv: Path,
    assembly_step: Path,
    assembly_stl: Path,
    plot_path: Path,
    plot_written: bool,
) -> None:
    status = overall_status(checks)
    warnings = [item for item in checks if item["status"] == "WARN"]
    failures = [item for item in checks if item["status"] == "FAIL"]
    lines = [
        "# upper_arm_v3 标准件装配与简化干涉检查报告",
        "",
        "## 这一步在做什么",
        "",
        "Step 15 针对 Step 14 在 V2 中发现的肘部孔系余量问题，生成 V3 的简化标准件装配检查。",
        "",
        "本步骤用简化 D8 主轴、M3 螺钉、M3 螺钉头和简化舵机连接板，检查 V3 的孔径、沉孔、孔间材料和 pad 边缘材料是否可接受。",
        "",
        "## V3 相比 V2 的修改",
        "",
        "- 肘部 2x2 安装孔中心从 x=105.0 mm 移到 x=103.0 mm，让孔系远离 elbow 主孔。",
        "- 肘部 M3 通孔半径从 1.60 mm 放宽到 1.75 mm，提高打印后螺钉装入成功率。",
        "- 肘部 M3 沉孔半径从 2.70 mm 放宽到 2.95 mm，提高螺钉头装配余量。",
        "- 肘部 pad 从 28 x 32 x 35 mm 增加到 30 x 34 x 36 mm，用局部加厚抵消放宽孔径带来的材料削弱。",
        "",
        "## 标准件简化假设",
        "",
        f"- 主关节轴/轴套：半径 {MAIN_SHAFT_RADIUS_MM:.2f} mm，对应约 D8 标准件包络。",
        f"- 肘部安装螺钉：M3 简化螺钉，杆部半径 {M3_SHANK_RADIUS_MM:.2f} mm，头部半径 {M3_HEAD_RADIUS_MM:.2f} mm，高度 {M3_HEAD_HEIGHT_MM:.2f} mm。",
        f"- 肩部夹紧/定位螺钉：M3 简化螺钉，杆部半径 {M3_SHANK_RADIUS_MM:.2f} mm，外侧头部半径 {SHOULDER_CLAMP_HEAD_RADIUS_MM:.2f} mm。",
        f"- 肘部简化舵机/连接板 footprint：{SERVO_REFERENCE_PLATE_X_MM:.1f} x {SERVO_REFERENCE_PLATE_Y_MM:.1f} mm。",
        "",
        "## 总体结论",
        "",
        f"- 装配检查总体状态：`{status}`。",
        f"- FAIL 项数量：{len(failures)}。",
        f"- WARN 项数量：{len(warnings)}。",
    ]
    if warnings:
        lines.extend(["", "仍需注意的 WARN 项："])
        for item in warnings:
            lines.append(f"- `{item['name']}`：{item['measured']}。{item['note']}")
    else:
        lines.append("- Step 14 中 V2 的 3 个装配余量 WARN 项在 V3 中已消除。")

    lines.extend(
        [
            "",
            "## 检查明细",
            "",
            "| 检查项 | 要求 | 实测/计算 | 状态 | 说明 |",
            "|---|---|---|---|---|",
        ]
    )
    for item in checks:
        lines.append(
            f"| {item['name']} | {item['requirement']} | {item['measured']} | {item['status']} | {item['note']} |"
        )

    lines.extend(
        [
            "",
            "## 工程解释",
            "",
            "- V3 的主关节孔、肩部夹紧孔和肘部 M3 孔在简化标准件尺寸下都能装入。",
            "- V3 用少量局部材料换取更好的装配余量，这比继续追求极限轻量化更适合作品集阶段的工程表达。",
            "- 本检查仍然是设计阶段筛查，不等同于真实打印、螺钉锁紧、疲劳和冲击载荷验证。",
            "",
            "## 下一步",
            "",
            "Step 16 应将 V3 接回 URDF/PyBullet，复查关节链、末端运动学和惯量更新后的静载力矩，确认 V3 的局部加厚没有明显破坏前面建立的仿真 baseline。",
            "",
            "## 输出文件",
            "",
            f"- 简化标准件装配 STEP：`{assembly_step}`",
            f"- 简化标准件装配 STL：`{assembly_stl}`",
            f"- 结构化结果 JSON：`{result_json}`",
            f"- 检查明细 CSV：`{checks_csv}`",
        ]
    )
    if plot_written:
        lines.append(f"- 孔系检查图：`{plot_path}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    out_dir = SCRIPT_DIR / "assembly_check"
    out_dir.mkdir(parents=True, exist_ok=True)
    params = read_params()
    checks = build_checks(params)
    status = overall_status(checks)

    assembly = Compound(children=(cad.upper_arm_v3(), *simplified_standard_parts(params)))

    assembly_step = out_dir / "upper_arm_v3_simplified_standard_parts_assembly.step"
    assembly_stl = out_dir / "upper_arm_v3_simplified_standard_parts_assembly.stl"
    export_step(assembly, assembly_step)
    export_stl(assembly, assembly_stl)

    checks_csv = out_dir / "upper_arm_v3_standard_part_checks.csv"
    result_json = out_dir / "upper_arm_v3_standard_part_check.json"
    report = out_dir / "upper_arm_v3_standard_part_check_report_zh.md"
    plot_path = out_dir / "upper_arm_v3_standard_part_layout.png"
    plot_written = maybe_write_plot(plot_path, params, checks)

    write_checks_csv(checks_csv, checks)
    result = {
        "overall_status": status,
        "standard_part_assumptions": {
            "main_shaft_radius_mm": MAIN_SHAFT_RADIUS_MM,
            "m3_shank_radius_mm": M3_SHANK_RADIUS_MM,
            "m3_head_radius_mm": M3_HEAD_RADIUS_MM,
            "m3_head_height_mm": M3_HEAD_HEIGHT_MM,
            "shoulder_clamp_head_radius_mm": SHOULDER_CLAMP_HEAD_RADIUS_MM,
            "servo_reference_plate_xyz_mm": [
                SERVO_REFERENCE_PLATE_X_MM,
                SERVO_REFERENCE_PLATE_Y_MM,
                SERVO_REFERENCE_PLATE_Z_MM,
            ],
        },
        "checks": checks,
        "outputs": {
            "assembly_step": str(assembly_step),
            "assembly_stl": str(assembly_stl),
            "checks_csv": str(checks_csv),
            "report": str(report),
            "plot": str(plot_path) if plot_written else "",
        },
    }
    result_json.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_report(report, params, checks, result_json, checks_csv, assembly_step, assembly_stl, plot_path, plot_written)

    print(report)
    print(result_json)
    print(checks_csv)
    print(assembly_step)
    print(assembly_stl)
    if plot_written:
        print(plot_path)
    print(json.dumps({"overall_status": status}, ensure_ascii=False))


if __name__ == "__main__":
    main()
