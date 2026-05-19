from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

from build123d import Box, Compound, Cylinder, Pos, export_step, export_stl


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from upper_arm_v2_cad import upper_arm_v2  # noqa: E402


# Simplified standard-part assumptions, in millimeters.
MAIN_SHAFT_RADIUS_MM = 4.0  # nominal D8 shaft / bushing envelope
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
    return json.loads((SCRIPT_DIR / "upper_arm_v2_parameters.json").read_text(encoding="utf-8"))


def elbow_screw_centers(params: dict[str, Any]) -> list[tuple[float, float]]:
    center_x = float(params["elbow_servo_mount_center_x_mm"])
    half_x = float(params["elbow_servo_mount_spacing_x_mm"]) / 2.0
    half_y = float(params["elbow_servo_mount_spacing_y_mm"]) / 2.0
    return [(center_x + dx, dy) for dx in (-half_x, half_x) for dy in (-half_y, half_y)]


def simplified_standard_parts(params: dict[str, Any]):
    parts = []

    # Main shoulder and elbow shafts / bushings.
    span = float(params["span_shoulder_to_elbow_mm"])
    parts.append(Pos(0, 0, 0) * y_cylinder(MAIN_SHAFT_RADIUS_MM, MAIN_SHAFT_LENGTH_MM))
    parts.append(Pos(span, 0, 0) * y_cylinder(MAIN_SHAFT_RADIUS_MM, MAIN_SHAFT_LENGTH_MM))

    # Elbow M3 screw set through Z with simplified socket/cap heads in the
    # counterbore pockets.
    pad_z = 35.0
    top_z = pad_z / 2.0
    bottom_z = -pad_z / 2.0
    through_length = pad_z + 8.0
    for x_pos, y_pos in elbow_screw_centers(params):
        parts.append(Pos(x_pos, y_pos, 0) * z_cylinder(M3_SHANK_RADIUS_MM, through_length))
        parts.append(Pos(x_pos, y_pos, top_z - M3_HEAD_HEIGHT_MM / 2.0) * z_cylinder(M3_HEAD_RADIUS_MM, M3_HEAD_HEIGHT_MM))
        parts.append(Pos(x_pos, y_pos, bottom_z + M3_HEAD_HEIGHT_MM / 2.0) * z_cylinder(M3_HEAD_RADIUS_MM, M3_HEAD_HEIGHT_MM))

    # Shoulder clamp / locating M3 screws through Y with external heads.
    shoulder_x = float(params["shoulder_clamp_hole_x_mm"])
    shoulder_z = float(params["shoulder_clamp_hole_z_mm"])
    shoulder_pad_y = 36.0
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

    # A simplified mating plate to represent the elbow-side servo/connector.
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
            "用于判断 shoulder/elbow 主轴是否能装入。0.20 mm 对 3D 打印属于可用但偏紧的余量。",
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
            "功能上可装入，但 0.10 mm 对 FDM 打印偏紧，建议 V3 放宽到半径 1.70-1.80 mm。",
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
            "螺钉头可进入沉孔，但径向余量偏紧。",
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

    elbow_pad_center_x = 105.0
    elbow_pad_size_x = 28.0
    elbow_pad_size_y = 32.0
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
            "边缘余量充足。",
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
            "2x2 孔系自身材料余量可接受。",
        )
    )

    span = float(params["span_shoulder_to_elbow_mm"])
    nearest_elbow_x = max(x for x, _ in elbow_screw_centers(params))
    elbow_mount_to_main_bore = abs(span - nearest_elbow_x) - joint_bore - elbow_hole_radius
    checks.append(
        check(
            "肘部安装孔与 elbow 主孔最小孔间材料",
            f"建议 >= {NEAR_BORE_RECOMMENDED_MM:.1f} mm",
            f"{elbow_mount_to_main_bore:.3f} mm",
            elbow_mount_to_main_bore,
            "PASS" if elbow_mount_to_main_bore >= NEAR_BORE_RECOMMENDED_MM else "WARN",
            "这是当前最需要关注的风险：最近的 M3 通孔离 elbow 主孔较近，真实打印和受力下建议 V3 把孔系向近端移动 1-2 mm 或增大局部材料。",
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

    shoulder_pad_center_x = -5.0
    shoulder_pad_size_x = 22.0
    shoulder_pad_size_z = 30.0
    shoulder_x = float(params["shoulder_clamp_hole_x_mm"])
    shoulder_z = float(params["shoulder_clamp_hole_z_mm"])
    shoulder_min_x = shoulder_pad_center_x - shoulder_pad_size_x / 2.0
    shoulder_max_x = shoulder_pad_center_x + shoulder_pad_size_x / 2.0
    shoulder_min_z = -shoulder_pad_size_z / 2.0
    shoulder_max_z = shoulder_pad_size_z / 2.0
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
            "边缘材料余量可接受，但已接近小型打印件下限。",
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

    elbow_pad_center_x = 105.0
    elbow_pad_size_x = 28.0
    elbow_pad_size_y = 32.0
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
        ax_elbow.add_patch(plt.Circle((x_pos, y_pos), float(params["elbow_servo_hole_radius_mm"]), fill=False, color="tab:blue"))
        ax_elbow.add_patch(plt.Circle((x_pos, y_pos), float(params["elbow_servo_counterbore_radius_mm"]), fill=False, color="tab:orange", linestyle="--"))
    ax_elbow.add_patch(plt.Circle((float(params["span_shoulder_to_elbow_mm"]), 0), float(params["joint_bore_radius_mm"]), fill=False, color="tab:red"))
    ax_elbow.set_title("Elbow mounting pattern, XY view")
    ax_elbow.set_xlabel("x / mm")
    ax_elbow.set_ylabel("y / mm")
    ax_elbow.axis("equal")
    ax_elbow.grid(True, alpha=0.3)

    shoulder_pad_center_x = -5.0
    shoulder_pad_size_x = 22.0
    shoulder_pad_size_z = 30.0
    ax_shoulder.add_patch(
        plt.Rectangle(
            (shoulder_pad_center_x - shoulder_pad_size_x / 2.0, -shoulder_pad_size_z / 2.0),
            shoulder_pad_size_x,
            shoulder_pad_size_z,
            fill=False,
            linewidth=2,
            label="shoulder pad",
        )
    )
    shoulder_x = float(params["shoulder_clamp_hole_x_mm"])
    shoulder_z = float(params["shoulder_clamp_hole_z_mm"])
    for z_pos in (-shoulder_z, shoulder_z):
        ax_shoulder.add_patch(plt.Circle((shoulder_x, z_pos), float(params["shoulder_clamp_hole_radius_mm"]), fill=False, color="tab:blue"))
    ax_shoulder.add_patch(plt.Circle((0, 0), float(params["joint_bore_radius_mm"]), fill=False, color="tab:red"))
    ax_shoulder.set_title("Shoulder clamp holes, XZ view")
    ax_shoulder.set_xlabel("x / mm")
    ax_shoulder.set_ylabel("z / mm")
    ax_shoulder.axis("equal")
    ax_shoulder.grid(True, alpha=0.3)

    status = overall_status(checks)
    fig.suptitle(f"upper_arm_v2 simplified standard-part assembly check: {status}")
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
        "# upper_arm_v2 标准件装配与简化干涉检查报告",
        "",
        "## 这一步在做什么",
        "",
        "Step 14 在 V2 二级孔位校核的基础上，引入简化标准件模型，检查主轴/轴套、M3 螺钉、沉孔和简化舵机连接板是否能与 V2 的孔系匹配。",
        "",
        "本步骤不是生产级装配认证，而是作品集阶段的标准件适配性和明显干涉风险筛查。",
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
        lines.extend(["", "需要注意的 WARN 项："])
        for item in warnings:
            lines.append(f"- `{item['name']}`：{item['measured']}。{item['note']}")

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
            "- V2 的主轴、肘部 M3 螺钉、肩部夹紧螺钉在简化尺寸下都能装入对应孔系。",
            "- 但肘部 M3 通孔半径 1.60 mm 对 M3 螺钉的径向余量只有 0.10 mm，真实 FDM/树脂打印时偏紧。",
            "- 最近的肘部 M3 安装孔与 elbow 主孔之间理论材料余量约 0.25 mm，这是当前主要风险点。",
            "- 因此 Step 14 的结论不是“生产级完全通过”，而是“标准件可装入，但 V3 应优化肘部孔系余量”。",
            "",
            "## V3 改进建议",
            "",
            "- 将肘部 2x2 安装孔中心整体向近端移动 1-2 mm，增大其与 elbow 主孔之间的材料余量。",
            "- 将肘部 M3 通孔半径从 1.60 mm 放宽到 1.70-1.80 mm，提升打印后装配成功率。",
            "- 将 M3 螺钉头沉孔半径从 2.70 mm 放宽到 2.90-3.00 mm，给实际螺钉头和打印误差留余量。",
            "- 在肘部主孔和最近 M3 孔之间增加局部 boss 或加厚 pad，避免薄弱材料桥。",
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

    assembly = Compound(children=(upper_arm_v2(), *simplified_standard_parts(params)))

    assembly_step = out_dir / "upper_arm_v2_simplified_standard_parts_assembly.step"
    assembly_stl = out_dir / "upper_arm_v2_simplified_standard_parts_assembly.stl"
    export_step(assembly, assembly_step)
    export_stl(assembly, assembly_stl)

    checks_csv = out_dir / "upper_arm_v2_standard_part_checks.csv"
    result_json = out_dir / "upper_arm_v2_standard_part_check.json"
    report = out_dir / "upper_arm_v2_standard_part_check_report_zh.md"
    plot_path = out_dir / "upper_arm_v2_standard_part_layout.png"
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
