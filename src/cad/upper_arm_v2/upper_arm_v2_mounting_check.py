from __future__ import annotations

import csv
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any


V1_DIR = Path(__file__).resolve().parents[1] / "upper_arm_v1"
sys.path.insert(0, str(V1_DIR))

from upper_arm_v1_mounting_check import (  # noqa: E402
    AXIS_TOL_DEG,
    MM_TOL,
    RADIUS_TOL,
    axis_error_deg,
    check_line,
    close,
    dedupe_cylinders,
    extract_cylinders,
    feature_table_rows,
    find_y_feature,
    fmt,
    fmt_feature,
)


SECONDARY_MM_TOL = 0.30
SECONDARY_RADIUS_TOL = 0.10


def z_axis(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if row["axis_label"] == "Z"]


def find_z_feature(
    rows: list[dict[str, Any]],
    *,
    radius_mm: float,
    x_mm: float,
    y_mm: float,
    radius_tol: float = SECONDARY_RADIUS_TOL,
    xy_tol: float = SECONDARY_MM_TOL,
) -> dict[str, Any] | None:
    candidates = [
        row
        for row in z_axis(rows)
        if close(row["radius_mm"], radius_mm, radius_tol)
        and close(row["loc_x_mm"], x_mm, xy_tol)
        and close(row["loc_y_mm"], y_mm, xy_tol)
    ]
    if not candidates:
        return None
    return sorted(candidates, key=lambda row: (abs(row["loc_z_mm"]), row["face_index"]))[0]


def find_y_secondary_feature(
    rows: list[dict[str, Any]],
    *,
    radius_mm: float,
    x_mm: float,
    z_mm: float,
    radius_tol: float = SECONDARY_RADIUS_TOL,
    pos_tol: float = SECONDARY_MM_TOL,
) -> dict[str, Any] | None:
    candidates = [
        row
        for row in rows
        if row["axis_label"] == "Y"
        and close(row["radius_mm"], radius_mm, radius_tol)
        and close(row["loc_x_mm"], x_mm, pos_tol)
        and close(row["loc_z_mm"], z_mm, pos_tol)
    ]
    if not candidates:
        return None
    return sorted(candidates, key=lambda row: (abs(row["loc_y_mm"]), row["face_index"]))[-1]


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


def elbow_hole_targets(params: dict[str, Any]) -> list[dict[str, float]]:
    center_x = float(params["elbow_servo_mount_center_x_mm"])
    half_x = float(params["elbow_servo_mount_spacing_x_mm"]) / 2.0
    half_y = float(params["elbow_servo_mount_spacing_y_mm"]) / 2.0
    radius = float(params["elbow_servo_hole_radius_mm"])
    targets = []
    for x_offset in (-half_x, half_x):
        for y_offset in (-half_y, half_y):
            targets.append({"x_mm": center_x + x_offset, "y_mm": y_offset, "radius_mm": radius})
    return targets


def shoulder_hole_targets(params: dict[str, Any]) -> list[dict[str, float]]:
    x_mm = float(params["shoulder_clamp_hole_x_mm"])
    z = float(params["shoulder_clamp_hole_z_mm"])
    radius = float(params["shoulder_clamp_hole_radius_mm"])
    return [
        {"x_mm": x_mm, "z_mm": -z, "radius_mm": radius},
        {"x_mm": x_mm, "z_mm": z, "radius_mm": radius},
    ]


def main() -> None:
    project_root = Path(__file__).resolve().parents[2]
    out_dir = Path(__file__).resolve().parent
    check_dir = out_dir / "mounting_check"
    check_dir.mkdir(parents=True, exist_ok=True)

    official_step = project_root / "00_source_snapshot" / "STEP_SO101" / "Upper_arm_SO101.step"
    v1_step = project_root / "05_improved_design" / "upper_arm_v1" / "upper_arm_v1_ai_rebuild.step"
    v2_step = out_dir / "upper_arm_v2_ai_rebuild.step"
    params_path = out_dir / "upper_arm_v2_parameters.json"
    params = json.loads(params_path.read_text(encoding="utf-8"))

    official_unique = dedupe_cylinders(extract_cylinders(official_step, "official_upper_arm_step"))
    v1_unique = dedupe_cylinders(extract_cylinders(v1_step, "upper_arm_v1_step"))
    v2_unique = dedupe_cylinders(extract_cylinders(v2_step, "upper_arm_v2_step"))

    expected_span = float(params["span_shoulder_to_elbow_mm"])
    expected_bore = float(params["joint_bore_radius_mm"])
    expected_shoulder_boss = float(params["shoulder_boss_radius_mm"])
    expected_elbow_boss = float(params["elbow_boss_radius_mm"])

    shoulder_bore = find_y_feature(v2_unique, radius_mm=expected_bore, x_mm=0.0)
    elbow_bore = find_y_feature(v2_unique, radius_mm=expected_bore, x_mm=expected_span)
    shoulder_boss = find_y_feature(v2_unique, radius_mm=expected_shoulder_boss, x_mm=0.0)
    elbow_boss = find_y_feature(v2_unique, radius_mm=expected_elbow_boss, x_mm=expected_span)
    bore_span = None
    if shoulder_bore and elbow_bore:
        bore_span = float(elbow_bore["loc_x_mm"]) - float(shoulder_bore["loc_x_mm"])

    primary_checks = [
        check_line(
            "肩部主孔半径",
            f"{expected_bore:.3f} mm",
            fmt_feature(shoulder_bore),
            shoulder_bore is not None and close(float(shoulder_bore["radius_mm"]), expected_bore, RADIUS_TOL),
            "保持 shoulder_lift 轴线接口。",
        ),
        check_line(
            "肘部主孔半径",
            f"{expected_bore:.3f} mm",
            fmt_feature(elbow_bore),
            elbow_bore is not None and close(float(elbow_bore["radius_mm"]), expected_bore, RADIUS_TOL),
            "保持 elbow_flex 轴线接口。",
        ),
        check_line(
            "肩部到肘部主孔中心距",
            f"{expected_span:.3f} mm",
            fmt(bore_span),
            bore_span is not None and close(bore_span, expected_span, MM_TOL),
            "保证 V2 不破坏原运动链长度。",
        ),
        check_line(
            "主孔轴线方向",
            f"沿 Y 轴，误差 <= {AXIS_TOL_DEG:.1f} deg",
            (
                f"shoulder={fmt(None if shoulder_bore is None else axis_error_deg(shoulder_bore['axis_x'], shoulder_bore['axis_y'], shoulder_bore['axis_z'], 'Y'))} deg, "
                f"elbow={fmt(None if elbow_bore is None else axis_error_deg(elbow_bore['axis_x'], elbow_bore['axis_y'], elbow_bore['axis_z'], 'Y'))} deg"
            ),
            shoulder_bore is not None
            and elbow_bore is not None
            and axis_error_deg(shoulder_bore["axis_x"], shoulder_bore["axis_y"], shoulder_bore["axis_z"], "Y")
            <= AXIS_TOL_DEG
            and axis_error_deg(elbow_bore["axis_x"], elbow_bore["axis_y"], elbow_bore["axis_z"], "Y")
            <= AXIS_TOL_DEG,
            "主孔轴线必须与转轴方向一致。",
        ),
        check_line(
            "肩部外凸台半径",
            f"{expected_shoulder_boss:.3f} mm",
            fmt_feature(shoulder_boss),
            shoulder_boss is not None
            and close(float(shoulder_boss["radius_mm"]), expected_shoulder_boss, RADIUS_TOL),
            "保留肩部承载区域。",
        ),
        check_line(
            "肘部外凸台半径",
            f"{expected_elbow_boss:.3f} mm",
            fmt_feature(elbow_boss),
            elbow_boss is not None and close(float(elbow_boss["radius_mm"]), expected_elbow_boss, RADIUS_TOL),
            "保留肘部承载区域。",
        ),
    ]

    elbow_checks = []
    elbow_detected = []
    for idx, target in enumerate(elbow_hole_targets(params), start=1):
        feature = find_z_feature(
            v2_unique,
            radius_mm=target["radius_mm"],
            x_mm=target["x_mm"],
            y_mm=target["y_mm"],
        )
        elbow_detected.append(feature)
        elbow_checks.append(
            check_line(
                f"肘部 2x2 舵机安装孔 {idx}",
                f"Z 轴通孔 r={target['radius_mm']:.3f} mm, x={target['x_mm']:.3f}, y={target['y_mm']:.3f}",
                fmt_feature(feature),
                feature is not None,
                "对应 V2 新增的舵机/连接件安装孔。",
            )
        )

    shoulder_checks = []
    shoulder_detected = []
    for idx, target in enumerate(shoulder_hole_targets(params), start=1):
        feature = find_y_secondary_feature(
            v2_unique,
            radius_mm=target["radius_mm"],
            x_mm=target["x_mm"],
            z_mm=target["z_mm"],
        )
        shoulder_detected.append(feature)
        shoulder_checks.append(
            check_line(
                f"肩部侧向夹紧/定位孔 {idx}",
                f"Y 轴通孔 r={target['radius_mm']:.3f} mm, x={target['x_mm']:.3f}, z={target['z_mm']:.3f}",
                fmt_feature(feature),
                feature is not None,
                "对应 V2 新增的肩部定位或夹紧接口。",
            )
        )

    official_axis_counter = Counter(row["axis_label"] for row in official_unique)
    v1_axis_counter = Counter(row["axis_label"] for row in v1_unique)
    v2_axis_counter = Counter(row["axis_label"] for row in v2_unique)
    primary_passed = all(item["passed"] for item in primary_checks)
    secondary_passed = all(item["passed"] for item in elbow_checks + shoulder_checks)
    all_passed = primary_passed and secondary_passed

    official_reference = sorted(
        [
            row
            for row in official_unique
            if (row["axis_label"] == "Z" and 1.4 <= row["radius_mm"] <= 3.1)
            or (row["axis_label"] == "Y" and row["radius_mm"] >= 1.5)
        ],
        key=lambda row: (row["axis_label"], -row["radius_mm"], row["loc_x_mm"], row["loc_y_mm"], row["loc_z_mm"]),
    )
    v2_mounting_features = sorted(
        [
            row
            for row in v2_unique
            if (row["axis_label"] == "Z" and row["radius_mm"] >= 1.4)
            or (row["axis_label"] == "Y" and row["radius_mm"] >= 1.5)
        ],
        key=lambda row: (row["axis_label"], -row["radius_mm"], row["loc_x_mm"], row["loc_y_mm"], row["loc_z_mm"]),
    )

    all_unique = sorted(
        official_unique + v1_unique + v2_unique,
        key=lambda row: (row["model"], row["axis_label"], -row["radius_mm"], row["loc_x_mm"]),
    )
    csv_path = check_dir / "upper_arm_v2_mounting_cylinders.csv"
    json_path = check_dir / "upper_arm_v2_mounting_check.json"
    report_path = check_dir / "upper_arm_v2_mounting_check_report_zh.md"
    write_csv(csv_path, all_unique)

    result = {
        "inputs": {
            "official_step": str(official_step),
            "upper_arm_v1_step": str(v1_step),
            "upper_arm_v2_step": str(v2_step),
            "parameters": str(params_path),
        },
        "counts": {
            "official_unique_cylinders": len(official_unique),
            "upper_arm_v1_unique_cylinders": len(v1_unique),
            "upper_arm_v2_unique_cylinders": len(v2_unique),
            "official_axis_counter": dict(official_axis_counter),
            "upper_arm_v1_axis_counter": dict(v1_axis_counter),
            "upper_arm_v2_axis_counter": dict(v2_axis_counter),
        },
        "primary_checks": primary_checks,
        "elbow_servo_mount_checks": elbow_checks,
        "shoulder_clamp_checks": shoulder_checks,
        "v2_mounting_features": v2_mounting_features,
        "official_reference_features": official_reference,
        "conclusion": {
            "v2_primary_interface_check": "PASS" if primary_passed else "FAIL",
            "v2_secondary_mounting_feature_check": "PASS" if secondary_passed else "FAIL",
            "v2_overall_mounting_check": "PASS" if all_passed else "FAIL",
            "official_exact_mounting_pattern_reproduced": "PARTIAL",
            "note": (
                "V2 adds explicit secondary mounting features and keeps primary joint interfaces. "
                "Because official and V2 datum frames are not fully mated, this is a feature-level "
                "replication/check, not final production assembly certification."
            ),
        },
    }
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    lines = [
        "# upper_arm_v2 CAD 二级装配孔位校核报告",
        "",
        "## 这一步在做什么",
        "",
        "在 V1 主关节接口校核通过的基础上，检查 V2 新增的装配孔系是否真实写入 STEP：包括肘部 2x2 舵机安装孔和肩部侧向夹紧/定位孔。",
        "",
        "## 输入文件",
        "",
        f"- 官方 STEP：`{official_step}`",
        f"- V1 STEP：`{v1_step}`",
        f"- V2 STEP：`{v2_step}`",
        f"- V2 参数：`{params_path}`",
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
            f"| upper_arm_v1_step | {len(v1_unique)} | {v1_axis_counter.get('X', 0)} | "
            f"{v1_axis_counter.get('Y', 0)} | {v1_axis_counter.get('Z', 0)} | {v1_axis_counter.get('oblique', 0)} |"
        ),
        (
            f"| upper_arm_v2_step | {len(v2_unique)} | {v2_axis_counter.get('X', 0)} | "
            f"{v2_axis_counter.get('Y', 0)} | {v2_axis_counter.get('Z', 0)} | {v2_axis_counter.get('oblique', 0)} |"
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
            "## V2 新增装配孔系校核",
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
            "## V2 检测到的装配相关圆柱特征",
            "",
            *feature_table_rows(v2_mounting_features, max_rows=32),
            "",
            "## 官方 STEP 中可作为参考的孔/台阶特征",
            "",
            "下表不是一一坐标对齐结论，而是说明官方件确实存在大量 Z 轴螺丝孔、Y 轴定位/主孔等装配特征。V2 当前做的是结构化补齐，而不是完全复制官方所有曲面与倒角。",
            "",
            *feature_table_rows(official_reference, max_rows=32),
            "",
            "## 结论",
            "",
            f"- V2 主关节接口复核：`{'PASS' if primary_passed else 'FAIL'}`。",
            f"- V2 新增装配孔系校核：`{'PASS' if secondary_passed else 'FAIL'}`。",
            f"- V2 二级装配孔位校核总体：`{'PASS' if all_passed else 'FAIL'}`。",
            "- V2 相比 V1 的核心进步是：从只有主孔/轻量化孔，推进到包含明确的舵机安装孔和肩部定位/夹紧孔。",
            "- 仍需说明：由于没有完整官方装配 mating transform 和舵机标准件模型，V2 还不是生产级装配认证；它是作品集阶段的 CAD 特征级改进版。",
            "",
            "## 这一步的意义",
            "",
            "- V1 证明 AI 可以生成可仿真的轻量化结构。",
            "- V2 证明你开始按机械装配逻辑补孔系、定位和可制造接口，不再停留在外观建模。",
            "- 这一步可以作为实习作品集中“AI 辅助结构迭代 + CAD 特征校核”的关键证据。",
            "",
            "## 下一步",
            "",
            "建议做 Step 12：将 V2 接回 URDF/PyBullet 做 visual smoke test，并重新做几何/质量/力矩对比，形成 V1 与 V2 的 before-after 表。",
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
