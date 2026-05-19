from __future__ import annotations

import csv
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any


V2_DIR = Path(__file__).resolve().parents[1] / "upper_arm_v2"
sys.path.insert(0, str(V2_DIR))

import upper_arm_v2_mounting_check as base  # noqa: E402


def elbow_hole_targets(params: dict[str, Any]) -> list[dict[str, float]]:
    center_x = float(params["elbow_servo_mount_center_x_mm"])
    half_x = float(params["elbow_servo_mount_spacing_x_mm"]) / 2.0
    half_y = float(params["elbow_servo_mount_spacing_y_mm"]) / 2.0
    radius = float(params["elbow_servo_hole_radius_mm"])
    return [{"x_mm": center_x + dx, "y_mm": dy, "radius_mm": radius} for dx in (-half_x, half_x) for dy in (-half_y, half_y)]


def shoulder_hole_targets(params: dict[str, Any]) -> list[dict[str, float]]:
    x_mm = float(params["shoulder_clamp_hole_x_mm"])
    z = float(params["shoulder_clamp_hole_z_mm"])
    radius = float(params["shoulder_clamp_hole_radius_mm"])
    return [
        {"x_mm": x_mm, "z_mm": -z, "radius_mm": radius},
        {"x_mm": x_mm, "z_mm": z, "radius_mm": radius},
    ]


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "model",
        "face_index",
        "radius_mm",
        "loc_x_mm",
        "loc_y_mm",
        "loc_z_mm",
        "axis_x",
        "axis_y",
        "axis_z",
        "axis_label",
        "axis_error_to_y_deg",
        "duplicate_count",
    ]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def main() -> None:
    project_root = Path(__file__).resolve().parents[2]
    out_dir = Path(__file__).resolve().parent
    check_dir = out_dir / "mounting_check"
    check_dir.mkdir(parents=True, exist_ok=True)

    official_step = project_root / "00_source_snapshot" / "STEP_SO101" / "Upper_arm_SO101.step"
    v2_step = project_root / "05_improved_design" / "upper_arm_v2" / "upper_arm_v2_ai_rebuild.step"
    v3_step = out_dir / "upper_arm_v3_ai_rebuild.step"
    params_path = out_dir / "upper_arm_v3_parameters.json"
    params = json.loads(params_path.read_text(encoding="utf-8"))

    official_unique = base.dedupe_cylinders(base.extract_cylinders(official_step, "official_upper_arm_step"))
    v2_unique = base.dedupe_cylinders(base.extract_cylinders(v2_step, "upper_arm_v2_step"))
    v3_unique = base.dedupe_cylinders(base.extract_cylinders(v3_step, "upper_arm_v3_step"))

    expected_span = float(params["span_shoulder_to_elbow_mm"])
    expected_bore = float(params["joint_bore_radius_mm"])
    expected_shoulder_boss = float(params["shoulder_boss_radius_mm"])
    expected_elbow_boss = float(params["elbow_boss_radius_mm"])

    shoulder_bore = base.find_y_feature(v3_unique, radius_mm=expected_bore, x_mm=0.0)
    elbow_bore = base.find_y_feature(v3_unique, radius_mm=expected_bore, x_mm=expected_span)
    shoulder_boss = base.find_y_feature(v3_unique, radius_mm=expected_shoulder_boss, x_mm=0.0)
    elbow_boss = base.find_y_feature(v3_unique, radius_mm=expected_elbow_boss, x_mm=expected_span)
    bore_span = None
    if shoulder_bore and elbow_bore:
        bore_span = float(elbow_bore["loc_x_mm"]) - float(shoulder_bore["loc_x_mm"])

    primary_checks = [
        base.check_line(
            "肩部主孔半径",
            f"{expected_bore:.3f} mm",
            base.fmt_feature(shoulder_bore),
            shoulder_bore is not None and base.close(float(shoulder_bore["radius_mm"]), expected_bore, base.RADIUS_TOL),
            "保持 shoulder_lift 轴线接口。",
        ),
        base.check_line(
            "肘部主孔半径",
            f"{expected_bore:.3f} mm",
            base.fmt_feature(elbow_bore),
            elbow_bore is not None and base.close(float(elbow_bore["radius_mm"]), expected_bore, base.RADIUS_TOL),
            "保持 elbow_flex 轴线接口。",
        ),
        base.check_line(
            "肩部到肘部主孔中心距",
            f"{expected_span:.3f} mm",
            base.fmt(bore_span),
            bore_span is not None and base.close(bore_span, expected_span, base.MM_TOL),
            "保证 V3 不破坏原运动链长度。",
        ),
        base.check_line(
            "主孔轴线方向",
            f"沿 Y 轴，误差 <= {base.AXIS_TOL_DEG:.1f} deg",
            (
                f"shoulder={base.fmt(None if shoulder_bore is None else base.axis_error_deg(shoulder_bore['axis_x'], shoulder_bore['axis_y'], shoulder_bore['axis_z'], 'Y'))} deg, "
                f"elbow={base.fmt(None if elbow_bore is None else base.axis_error_deg(elbow_bore['axis_x'], elbow_bore['axis_y'], elbow_bore['axis_z'], 'Y'))} deg"
            ),
            shoulder_bore is not None
            and elbow_bore is not None
            and base.axis_error_deg(shoulder_bore["axis_x"], shoulder_bore["axis_y"], shoulder_bore["axis_z"], "Y")
            <= base.AXIS_TOL_DEG
            and base.axis_error_deg(elbow_bore["axis_x"], elbow_bore["axis_y"], elbow_bore["axis_z"], "Y")
            <= base.AXIS_TOL_DEG,
            "主孔轴线必须与转轴方向一致。",
        ),
        base.check_line(
            "肩部外凸台半径",
            f"{expected_shoulder_boss:.3f} mm",
            base.fmt_feature(shoulder_boss),
            shoulder_boss is not None
            and base.close(float(shoulder_boss["radius_mm"]), expected_shoulder_boss, base.RADIUS_TOL),
            "保留肩部承载区域。",
        ),
        base.check_line(
            "肘部外凸台半径",
            f"{expected_elbow_boss:.3f} mm",
            base.fmt_feature(elbow_boss),
            elbow_boss is not None and base.close(float(elbow_boss["radius_mm"]), expected_elbow_boss, base.RADIUS_TOL),
            "保留肘部承载区域。",
        ),
    ]

    elbow_checks = []
    for idx, target in enumerate(elbow_hole_targets(params), start=1):
        feature = base.find_z_feature(
            v3_unique,
            radius_mm=target["radius_mm"],
            x_mm=target["x_mm"],
            y_mm=target["y_mm"],
        )
        elbow_checks.append(
            base.check_line(
                f"肘部 2x2 舵机安装孔 {idx}",
                f"Z 轴通孔 r={target['radius_mm']:.3f} mm, x={target['x_mm']:.3f}, y={target['y_mm']:.3f}",
                base.fmt_feature(feature),
                feature is not None,
                "对应 V3 优化后的舵机/连接件安装孔。",
            )
        )

    shoulder_checks = []
    for idx, target in enumerate(shoulder_hole_targets(params), start=1):
        feature = base.find_y_secondary_feature(
            v3_unique,
            radius_mm=target["radius_mm"],
            x_mm=target["x_mm"],
            z_mm=target["z_mm"],
        )
        shoulder_checks.append(
            base.check_line(
                f"肩部侧向夹紧/定位孔 {idx}",
                f"Y 轴通孔 r={target['radius_mm']:.3f} mm, x={target['x_mm']:.3f}, z={target['z_mm']:.3f}",
                base.fmt_feature(feature),
                feature is not None,
                "对应肩部定位或夹紧接口。",
            )
        )

    official_axis_counter = Counter(row["axis_label"] for row in official_unique)
    v2_axis_counter = Counter(row["axis_label"] for row in v2_unique)
    v3_axis_counter = Counter(row["axis_label"] for row in v3_unique)
    primary_passed = all(item["passed"] for item in primary_checks)
    secondary_passed = all(item["passed"] for item in elbow_checks + shoulder_checks)
    all_passed = primary_passed and secondary_passed

    v3_mounting_features = sorted(
        [
            row
            for row in v3_unique
            if (row["axis_label"] == "Z" and row["radius_mm"] >= 1.4)
            or (row["axis_label"] == "Y" and row["radius_mm"] >= 1.5)
        ],
        key=lambda row: (row["axis_label"], -row["radius_mm"], row["loc_x_mm"], row["loc_y_mm"], row["loc_z_mm"]),
    )

    csv_path = check_dir / "upper_arm_v3_mounting_cylinders.csv"
    json_path = check_dir / "upper_arm_v3_mounting_check.json"
    report_path = check_dir / "upper_arm_v3_mounting_check_report_zh.md"
    all_unique = sorted(
        official_unique + v2_unique + v3_unique,
        key=lambda row: (row["model"], row["axis_label"], -row["radius_mm"], row["loc_x_mm"]),
    )
    write_csv(csv_path, all_unique)

    result = {
        "inputs": {
            "official_step": str(official_step),
            "upper_arm_v2_step": str(v2_step),
            "upper_arm_v3_step": str(v3_step),
            "parameters": str(params_path),
        },
        "counts": {
            "official_unique_cylinders": len(official_unique),
            "upper_arm_v2_unique_cylinders": len(v2_unique),
            "upper_arm_v3_unique_cylinders": len(v3_unique),
            "official_axis_counter": dict(official_axis_counter),
            "upper_arm_v2_axis_counter": dict(v2_axis_counter),
            "upper_arm_v3_axis_counter": dict(v3_axis_counter),
        },
        "primary_checks": primary_checks,
        "elbow_servo_mount_checks": elbow_checks,
        "shoulder_clamp_checks": shoulder_checks,
        "v3_mounting_features": v3_mounting_features,
        "conclusion": {
            "v3_primary_interface_check": "PASS" if primary_passed else "FAIL",
            "v3_secondary_mounting_feature_check": "PASS" if secondary_passed else "FAIL",
            "v3_overall_mounting_check": "PASS" if all_passed else "FAIL",
            "note": "V3 preserves the V2 primary interfaces while moving and widening the elbow-side mounting pattern.",
        },
    }
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    lines = [
        "# upper_arm_v3 CAD 孔位复核报告",
        "",
        "## 这一步在做什么",
        "",
        "检查 V3 在修复肘部孔系余量后，主关节接口和新增装配孔系是否仍然能从 STEP 圆柱特征中被检测到。",
        "",
        "## 圆柱特征统计",
        "",
        "| 模型 | 去重后圆柱特征数 | X 轴 | Y 轴 | Z 轴 | 斜轴 |",
        "|---|---:|---:|---:|---:|---:|",
        (
            f"| official_upper_arm_step | {len(official_unique)} | {official_axis_counter.get('X', 0)} | "
            f"{official_axis_counter.get('Y', 0)} | {official_axis_counter.get('Z', 0)} | {official_axis_counter.get('oblique', 0)} |"
        ),
        (
            f"| upper_arm_v2_step | {len(v2_unique)} | {v2_axis_counter.get('X', 0)} | "
            f"{v2_axis_counter.get('Y', 0)} | {v2_axis_counter.get('Z', 0)} | {v2_axis_counter.get('oblique', 0)} |"
        ),
        (
            f"| upper_arm_v3_step | {len(v3_unique)} | {v3_axis_counter.get('X', 0)} | "
            f"{v3_axis_counter.get('Y', 0)} | {v3_axis_counter.get('Z', 0)} | {v3_axis_counter.get('oblique', 0)} |"
        ),
        "",
        "## 主关节接口复核",
        "",
        "| 检查项 | 设计要求 | 实测/检测结果 | 结论 | 作用 |",
        "|---|---|---|---|---|",
    ]
    for item in primary_checks:
        lines.append(
            f"| {item['name']} | {item['expected']} | {item['measured']} | "
            f"{'通过' if item['passed'] else '未通过'} | {item['note']} |"
        )

    lines.extend(
        [
            "",
            "## V3 装配孔系复核",
            "",
            "| 检查项 | 设计要求 | 实测/检测结果 | 结论 | 作用 |",
            "|---|---|---|---|---|",
        ]
    )
    for item in elbow_checks + shoulder_checks:
        lines.append(
            f"| {item['name']} | {item['expected']} | {item['measured']} | "
            f"{'通过' if item['passed'] else '未通过'} | {item['note']} |"
        )

    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"- V3 主关节接口复核：`{'PASS' if primary_passed else 'FAIL'}`。",
            f"- V3 装配孔系复核：`{'PASS' if secondary_passed else 'FAIL'}`。",
            f"- V3 CAD 孔位复核总体：`{'PASS' if all_passed else 'FAIL'}`。",
            "- 本报告只确认 STEP 几何中的孔/轴/凸台特征存在；是否解决装配余量风险，需要看标准件装配检查报告。",
            "",
            "## 输出文件",
            "",
            f"- 圆柱特征 CSV：`{csv_path}`",
            f"- 结构化校核 JSON：`{json_path}`",
            f"- 中文报告：`{report_path}`",
        ]
    )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(report_path)
    print(json_path)
    print(csv_path)
    print(json.dumps(result["conclusion"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
