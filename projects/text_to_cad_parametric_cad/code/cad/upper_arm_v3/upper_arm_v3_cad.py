from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


V2_DIR = Path(__file__).resolve().parents[1] / "upper_arm_v2"
sys.path.insert(0, str(V2_DIR))

import upper_arm_v2_cad as base  # noqa: E402


# V3 only tightens the elbow-side assembly risk flagged in Step 14.
# Keep the main shoulder/elbow span unchanged.
base.ELBOW_PAD_X_MM = base.SPAN_MM - 13.0
base.ELBOW_PAD_SIZE_X_MM = 30.0
base.ELBOW_PAD_SIZE_Y_MM = 34.0
base.ELBOW_PAD_SIZE_Z_MM = 36.0
base.ELBOW_SERVO_MOUNT_CENTER_X_MM = base.SPAN_MM - 13.0
base.ELBOW_SERVO_MOUNT_SPACING_X_MM = 9.9
base.ELBOW_SERVO_MOUNT_SPACING_Y_MM = 9.9
base.ELBOW_SERVO_HOLE_RADIUS_MM = 1.75
base.ELBOW_SERVO_COUNTERBORE_RADIUS_MM = 2.95
base.ELBOW_SERVO_COUNTERBORE_DEPTH_MM = 2.40
base.RIB_WIDTH_Y_MM = 18.0
base.RIB_HEIGHT_Z_MM = 36.0


def upper_arm_v3():
    shape = base.upper_arm_v2()
    try:
        shape.label = "so101_upper_arm_v3_ai_rebuild"
    except Exception:
        pass
    return shape


def parameters() -> dict[str, float | list[float] | str]:
    params = base.parameters()
    params.update(
        {
            "elbow_pad_x_mm": base.ELBOW_PAD_X_MM,
            "elbow_pad_size_x_mm": base.ELBOW_PAD_SIZE_X_MM,
            "elbow_pad_size_y_mm": base.ELBOW_PAD_SIZE_Y_MM,
            "elbow_pad_size_z_mm": base.ELBOW_PAD_SIZE_Z_MM,
            "elbow_servo_mount_center_x_mm": base.ELBOW_SERVO_MOUNT_CENTER_X_MM,
            "elbow_servo_mount_spacing_x_mm": base.ELBOW_SERVO_MOUNT_SPACING_X_MM,
            "elbow_servo_mount_spacing_y_mm": base.ELBOW_SERVO_MOUNT_SPACING_Y_MM,
            "elbow_servo_hole_radius_mm": base.ELBOW_SERVO_HOLE_RADIUS_MM,
            "elbow_servo_counterbore_radius_mm": base.ELBOW_SERVO_COUNTERBORE_RADIUS_MM,
            "elbow_servo_counterbore_depth_mm": base.ELBOW_SERVO_COUNTERBORE_DEPTH_MM,
            "design_intent": (
                "upper_arm_v3 keeps the V2 functional interface but shifts the elbow-side "
                "mounting pattern closer to the shoulder and widens hole clearances to fix "
                "the remaining assembly warning flagged in Step 14"
            ),
        }
    )
    return params


def export_all(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    shape = upper_arm_v3()
    base.export_step(shape, out_dir / "upper_arm_v3_ai_rebuild.step")
    base.export_stl(shape, out_dir / "upper_arm_v3_ai_rebuild.stl")
    (out_dir / "upper_arm_v3_parameters.json").write_text(
        json.dumps(parameters(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    export_all(args.out_dir)
    print(f"Exported STEP: {args.out_dir / 'upper_arm_v3_ai_rebuild.step'}")
    print(f"Exported STL : {args.out_dir / 'upper_arm_v3_ai_rebuild.stl'}")
    print(f"Exported params: {args.out_dir / 'upper_arm_v3_parameters.json'}")


if __name__ == "__main__":
    main()
