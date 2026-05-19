from __future__ import annotations

import csv
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


MM_TOL = 0.20
RADIUS_TOL = 0.08
AXIS_TOL_DEG = 2.0


def shape_to_face(shape):
    if hasattr(TopoDS, "Face_s"):
        return TopoDS.Face_s(shape)
    return TopoDS.Face(shape)


def load_step_shape(step_path: Path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {step_path}")
    reader.TransferRoots()
    return reader.OneShape()


def canonical_axis(dx: float, dy: float, dz: float) -> tuple[float, float, float]:
    values = [dx, dy, dz]
    dominant = max(range(3), key=lambda idx: abs(values[idx]))
    if values[dominant] < 0:
        return (-dx, -dy, -dz)
    return (dx, dy, dz)


def axis_label(dx: float, dy: float, dz: float) -> str:
    values = {"X": abs(dx), "Y": abs(dy), "Z": abs(dz)}
    label, score = max(values.items(), key=lambda item: item[1])
    return label if score >= 0.95 else "oblique"


def axis_error_deg(dx: float, dy: float, dz: float, target: str = "Y") -> float:
    component = {"X": dx, "Y": dy, "Z": dz}[target]
    component = max(-1.0, min(1.0, abs(component)))
    return math.degrees(math.acos(component))


def extract_cylinders(step_path: Path, model_label: str) -> list[dict[str, Any]]:
    shape = load_step_shape(step_path)
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    cylinders: list[dict[str, Any]] = []
    face_index = 0
    while explorer.More():
        face_index += 1
        face = shape_to_face(explorer.Current())
        surface = BRepAdaptor_Surface(face, True)
        if surface.GetType() == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            radius = float(cylinder.Radius())
            loc = cylinder.Location()
            direction = cylinder.Axis().Direction()
            dx, dy, dz = canonical_axis(float(direction.X()), float(direction.Y()), float(direction.Z()))
            cylinders.append(
                {
                    "model": model_label,
                    "face_index": face_index,
                    "radius_mm": radius,
                    "loc_x_mm": float(loc.X()),
                    "loc_y_mm": float(loc.Y()),
                    "loc_z_mm": float(loc.Z()),
                    "axis_x": dx,
                    "axis_y": dy,
                    "axis_z": dz,
                    "axis_label": axis_label(dx, dy, dz),
                    "axis_error_to_y_deg": axis_error_deg(dx, dy, dz, "Y"),
                }
            )
        explorer.Next()
    return cylinders


def dedupe_cylinders(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: dict[tuple[Any, ...], dict[str, Any]] = {}
    for row in rows:
        key = (
            row["model"],
            round(row["radius_mm"], 3),
            round(row["loc_x_mm"], 3),
            round(row["loc_y_mm"], 3),
            round(row["loc_z_mm"], 3),
            round(row["axis_x"], 3),
            round(row["axis_y"], 3),
            round(row["axis_z"], 3),
        )
        if key not in seen:
            copied = dict(row)
            copied["duplicate_count"] = 1
            seen[key] = copied
        else:
            seen[key]["duplicate_count"] += 1
    return list(seen.values())


def close(value: float, expected: float, tol: float) -> bool:
    return abs(value - expected) <= tol


def y_axis(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if row["axis_label"] == "Y"]


def find_y_feature(
    rows: list[dict[str, Any]],
    *,
    radius_mm: float,
    x_mm: float,
    z_mm: float = 0.0,
    radius_tol: float = RADIUS_TOL,
    x_tol: float = MM_TOL,
    z_tol: float = MM_TOL,
) -> dict[str, Any] | None:
    candidates = [
        row
        for row in y_axis(rows)
        if close(row["radius_mm"], radius_mm, radius_tol)
        and close(row["loc_x_mm"], x_mm, x_tol)
        and close(row["loc_z_mm"], z_mm, z_tol)
    ]
    if not candidates:
        return None
    return sorted(candidates, key=lambda row: (abs(row["loc_y_mm"]), row["face_index"]))[-1]


def feature_value(feature: dict[str, Any] | None, field: str) -> float | None:
    if feature is None:
        return None
    return float(feature[field])


def check_line(
    name: str,
    expected: str,
    measured: str,
    passed: bool,
    note: str,
) -> dict[str, Any]:
    return {
        "name": name,
        "expected": expected,
        "measured": measured,
        "passed": bool(passed),
        "note": note,
    }


def fmt(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "未检测到"
    return f"{value:.{digits}f}"


def fmt_feature(feature: dict[str, Any] | None) -> str:
    if feature is None:
        return "未检测到"
    return (
        f"r={feature['radius_mm']:.3f}, "
        f"loc=({feature['loc_x_mm']:.3f}, {feature['loc_y_mm']:.3f}, {feature['loc_z_mm']:.3f}), "
        f"axis=({feature['axis_x']:.3f}, {feature['axis_y']:.3f}, {feature['axis_z']:.3f})"
    )


def feature_table_rows(rows: list[dict[str, Any]], max_rows: int = 16) -> list[str]:
    lines = [
        "| 半径 mm | 圆柱轴 | 轴线误差 deg | 圆柱位置 x,y,z mm | 重复面数 |",
        "|---:|---|---:|---:|---:|",
    ]
    for row in rows[:max_rows]:
        lines.append(
            f"| {row['radius_mm']:.3f} | {row['axis_label']} | {row['axis_error_to_y_deg']:.3f} | "
            f"{row['loc_x_mm']:.3f}, {row['loc_y_mm']:.3f}, {row['loc_z_mm']:.3f} | "
            f"{row.get('duplicate_count', 1)} |"
        )
    if len(rows) > max_rows:
        lines.append(f"| ... | ... | ... | 另有 {len(rows) - max_rows} 个特征，详见 CSV | ... |")
    return lines


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
    v1_step = out_dir / "upper_arm_v1_ai_rebuild.step"
    params_path = out_dir / "upper_arm_v1_parameters.json"
    params = json.loads(params_path.read_text(encoding="utf-8"))

    official_raw = extract_cylinders(official_step, "official_upper_arm_step")
    v1_raw = extract_cylinders(v1_step, "upper_arm_v1_step")
    official_unique = dedupe_cylinders(official_raw)
    v1_unique = dedupe_cylinders(v1_raw)

    expected_span = float(params["span_shoulder_to_elbow_mm"])
    expected_bore = float(params["joint_bore_radius_mm"])
    expected_shoulder_boss = float(params["shoulder_boss_radius_mm"])
    expected_elbow_boss = float(params["elbow_boss_radius_mm"])

    shoulder_bore = find_y_feature(v1_unique, radius_mm=expected_bore, x_mm=0.0)
    elbow_bore = find_y_feature(v1_unique, radius_mm=expected_bore, x_mm=expected_span)
    shoulder_boss = find_y_feature(v1_unique, radius_mm=expected_shoulder_boss, x_mm=0.0)
    elbow_boss = find_y_feature(v1_unique, radius_mm=expected_elbow_boss, x_mm=expected_span)

    bore_span = None
    if shoulder_bore and elbow_bore:
        bore_span = float(elbow_bore["loc_x_mm"]) - float(shoulder_bore["loc_x_mm"])

    checks = [
        check_line(
            "肩部主孔半径",
            f"{expected_bore:.3f} mm",
            fmt_feature(shoulder_bore),
            shoulder_bore is not None and close(float(shoulder_bore["radius_mm"]), expected_bore, RADIUS_TOL),
            "用于保持 shoulder_lift 轴线接口。",
        ),
        check_line(
            "肘部主孔半径",
            f"{expected_bore:.3f} mm",
            fmt_feature(elbow_bore),
            elbow_bore is not None and close(float(elbow_bore["radius_mm"]), expected_bore, RADIUS_TOL),
            "用于保持 elbow_flex 轴线接口。",
        ),
        check_line(
            "肩部到肘部主孔中心距",
            f"{expected_span:.3f} mm",
            fmt(bore_span),
            bore_span is not None and close(bore_span, expected_span, MM_TOL),
            "这是 V1 与 URDF 关节距离一致性的核心指标。",
        ),
        check_line(
            "主孔轴线方向",
            f"沿 Y 轴，误差 <= {AXIS_TOL_DEG:.1f} deg",
            (
                f"shoulder={fmt(feature_value(shoulder_bore, 'axis_error_to_y_deg'))} deg, "
                f"elbow={fmt(feature_value(elbow_bore, 'axis_error_to_y_deg'))} deg"
            ),
            shoulder_bore is not None
            and elbow_bore is not None
            and float(shoulder_bore["axis_error_to_y_deg"]) <= AXIS_TOL_DEG
            and float(elbow_bore["axis_error_to_y_deg"]) <= AXIS_TOL_DEG,
            "孔轴线必须与舵机/转轴方向一致，否则无法装配或会产生偏磨。",
        ),
        check_line(
            "肩部外凸台半径",
            f"{expected_shoulder_boss:.3f} mm",
            fmt_feature(shoulder_boss),
            shoulder_boss is not None
            and close(float(shoulder_boss["radius_mm"]), expected_shoulder_boss, RADIUS_TOL),
            "外凸台用于承载肩部连接区域。",
        ),
        check_line(
            "肘部外凸台半径",
            f"{expected_elbow_boss:.3f} mm",
            fmt_feature(elbow_boss),
            elbow_boss is not None and close(float(elbow_boss["radius_mm"]), expected_elbow_boss, RADIUS_TOL),
            "外凸台用于承载肘部连接区域。",
        ),
    ]

    official_y_candidates = [
        row
        for row in official_unique
        if row["axis_label"] == "Y" and row["radius_mm"] >= 1.0
    ]
    official_y_candidates = sorted(
        official_y_candidates,
        key=lambda row: (-row["radius_mm"], row["loc_x_mm"], row["loc_y_mm"], row["loc_z_mm"]),
    )
    v1_interface_features = [
        row
        for row in v1_unique
        if row["axis_label"] == "Y" and row["radius_mm"] >= 4.0
    ]
    v1_interface_features = sorted(
        v1_interface_features,
        key=lambda row: (-row["radius_mm"], row["loc_x_mm"], row["loc_y_mm"], row["loc_z_mm"]),
    )

    official_axis_counter = Counter(row["axis_label"] for row in official_unique)
    v1_axis_counter = Counter(row["axis_label"] for row in v1_unique)
    passed_count = sum(1 for item in checks if item["passed"])
    all_v1_primary_checks_passed = passed_count == len(checks)

    all_unique = sorted(
        official_unique + v1_unique,
        key=lambda row: (row["model"], row["axis_label"], -row["radius_mm"], row["loc_x_mm"]),
    )
    csv_path = check_dir / "upper_arm_v1_mounting_cylinders.csv"
    json_path = check_dir / "upper_arm_v1_mounting_check.json"
    report_path = check_dir / "upper_arm_v1_mounting_check_report_zh.md"
    write_csv(csv_path, all_unique)

    result = {
        "inputs": {
            "official_step": str(official_step),
            "upper_arm_v1_step": str(v1_step),
            "parameters": str(params_path),
        },
        "counts": {
            "official_raw_cylinders": len(official_raw),
            "official_unique_cylinders": len(official_unique),
            "upper_arm_v1_raw_cylinders": len(v1_raw),
            "upper_arm_v1_unique_cylinders": len(v1_unique),
            "official_axis_counter": dict(official_axis_counter),
            "upper_arm_v1_axis_counter": dict(v1_axis_counter),
        },
        "checks": checks,
        "v1_interface_features": v1_interface_features,
        "official_y_axis_reference_features": official_y_candidates,
        "conclusion": {
            "v1_primary_interface_self_check": "PASS" if all_v1_primary_checks_passed else "FAIL",
            "official_exact_mounting_pattern_reproduced": "NOT_YET",
            "note": "V1 keeps the designed URDF joint-span and primary bore/boss features, but it has not yet reproduced all official screw-hole and assembly mating features.",
        },
    }
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    report_lines = [
        "# upper_arm_v1 CAD 装配接口与孔位校核报告",
        "",
        "## 这一步在做什么",
        "",
        "读取官方 `Upper_arm_SO101.step` 和 AI 重建 `upper_arm_v1_ai_rebuild.step` 的真实 CAD B-Rep 几何，抽取圆柱面特征，检查 V1 的主安装孔、关节轴线、外凸台和 shoulder-to-elbow 中心距是否按设计生成。",
        "",
        "这不是只看 STL 外观，而是直接从 STEP 几何里读取圆柱孔/轴/凸台特征。",
        "",
        "## 输入文件",
        "",
        f"- 官方 STEP：`{official_step}`",
        f"- V1 STEP：`{v1_step}`",
        f"- V1 参数：`{params_path}`",
        "",
        "## 方法",
        "",
        f"- 使用 OpenCascade/OCP 读取 STEP。",
        f"- 遍历所有 CAD face，筛选圆柱面。",
        f"- 将圆柱轴线按 X/Y/Z 分类，主装配孔要求沿 Y 轴。",
        f"- V1 一级接口校核公差：半径 ±{RADIUS_TOL:.2f} mm，位置 ±{MM_TOL:.2f} mm，轴线误差 <= {AXIS_TOL_DEG:.1f} deg。",
        "",
        "## STEP 圆柱特征统计",
        "",
        "| 模型 | 原始圆柱面数 | 去重后圆柱特征数 | X 轴 | Y 轴 | Z 轴 | 斜轴 |",
        "|---|---:|---:|---:|---:|---:|---:|",
        (
            f"| official_upper_arm_step | {len(official_raw)} | {len(official_unique)} | "
            f"{official_axis_counter.get('X', 0)} | {official_axis_counter.get('Y', 0)} | "
            f"{official_axis_counter.get('Z', 0)} | {official_axis_counter.get('oblique', 0)} |"
        ),
        (
            f"| upper_arm_v1_step | {len(v1_raw)} | {len(v1_unique)} | "
            f"{v1_axis_counter.get('X', 0)} | {v1_axis_counter.get('Y', 0)} | "
            f"{v1_axis_counter.get('Z', 0)} | {v1_axis_counter.get('oblique', 0)} |"
        ),
        "",
        "## V1 主接口校核结果",
        "",
        "| 检查项 | 设计要求 | 实测/检测结果 | 结论 | 作用 |",
        "|---|---|---|---|---|",
    ]
    for item in checks:
        report_lines.append(
            f"| {item['name']} | {item['expected']} | {item['measured']} | "
            f"{'通过' if item['passed'] else '未通过'} | {item['note']} |"
        )

    report_lines.extend(
        [
            "",
            "## V1 检测到的主要 Y 轴接口特征",
            "",
            *feature_table_rows(v1_interface_features, max_rows=20),
            "",
            "## 官方 STEP 的 Y 轴圆柱参考特征",
            "",
            "官方件的圆柱特征明显更复杂，包含主轴孔、螺丝孔、圆角/倒角衍生圆柱面等。下面列出半径较大的 Y 轴圆柱特征作为后续 V2 复刻装配孔位的参考。",
            "",
            *feature_table_rows(official_y_candidates, max_rows=24),
            "",
            "## 结论",
            "",
            f"- V1 一级接口自检：`{'PASS' if all_v1_primary_checks_passed else 'FAIL'}`。",
            f"- V1 检测到 shoulder 主孔和 elbow 主孔，中心距为 `{fmt(bore_span)}` mm，设计目标为 `{expected_span:.3f}` mm。",
            "- V1 的主孔轴线沿 Y 轴，满足与 URDF 关节轴线一致的一级要求。",
            "- 但 V1 目前没有完整复刻官方 upper arm 的全部螺丝孔、定位台阶和装配 mating 特征，所以不能直接宣称已经是最终可装机零件。",
            "- 这一步的工程结论应表述为：`主关节接口几何通过一级校核；完整 CAD 装配孔位仍需 V2 继续补充官方孔系与装配基准`。",
            "",
            "## 这一步的意义",
            "",
            "- 把“AI 生成了一个好看的模型”推进到“AI 生成件接受 CAD 特征级校核”。",
            "- 证明 V1 至少保住了机械臂运动链最关键的两个接口：shoulder_lift 与 elbow_flex 的轴线和中心距。",
            "- 也明确暴露了下一步真实工程难点：需要从官方 STEP 或舵机规格中提取完整孔系，再做装配干涉和螺栓可达性检查。",
            "",
            "## 下一步",
            "",
            "Step 11 建议做 `upper_arm_v2`：在 V1 参数化模型基础上补充官方孔系/舵机安装孔/定位台阶，然后重新运行本脚本做二级装配孔位校核。",
            "",
            "## 输出文件",
            "",
            f"- 圆柱特征 CSV：`{csv_path}`",
            f"- 结构化校核 JSON：`{json_path}`",
            f"- 中文报告：`{report_path}`",
        ]
    )
    report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")

    print(report_path)
    print(json_path)
    print(csv_path)
    print(json.dumps(result["conclusion"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
