from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
PAYLOAD_KG = 0.10
GRAVITY = 9.81


@dataclass(frozen=True)
class BeamModule:
    name: str
    controlling_joint: str
    length_source: str
    b_mm: float
    h_mm: float
    wall_mm: float
    linked_masses: tuple[str, ...]
    role: str
    first_redesign_direction: str
    keep_interfaces: str


MODULES = [
    BeamModule(
        name="upper_arm_link",
        controlling_joint="shoulder_lift",
        length_source="elbow_flex",
        b_mm=18.0,
        h_mm=10.0,
        wall_mm=3.0,
        linked_masses=("upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"),
        role="Main shoulder-to-elbow load path. It carries the downstream arm, wrist, gripper, and payload.",
        first_redesign_direction="Increase weak-axis stiffness with ribs/box-section features and add cable channel without moving shoulder/elbow joint frames.",
        keep_interfaces="Keep shoulder_lift and elbow_flex joint origins, axes, servo horn clearance, and screw-hole interfaces unchanged.",
    ),
    BeamModule(
        name="lower_arm_link",
        controlling_joint="elbow_flex",
        length_source="wrist_flex",
        b_mm=16.0,
        h_mm=10.0,
        wall_mm=3.0,
        linked_masses=("lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"),
        role="Forearm load path from elbow to wrist. Distal mass here strongly affects shoulder and elbow torque.",
        first_redesign_direction="Reduce distal mass and add local reinforcement around the wrist motor holder and cable strain-relief region.",
        keep_interfaces="Keep elbow_flex and wrist_flex origins, wrist motor holder alignment, and wrist mounting screw pattern unchanged.",
    ),
    BeamModule(
        name="wrist_gripper_module",
        controlling_joint="wrist_flex",
        length_source="wrist_to_tool",
        b_mm=14.0,
        h_mm=8.0,
        wall_mm=2.5,
        linked_masses=("wrist_link", "gripper_link", "moving_jaw_so101_v1_link"),
        role="Compact distal module. It controls end-effector orientation and contributes high-leverage distal mass.",
        first_redesign_direction="Reduce distal mass, add compact local ribs, and add replaceable fingertip/tool-flange features.",
        keep_interfaces="Keep wrist_flex, wrist_roll, gripper joint frames, tool frame, and gripper opening geometry compatible.",
    ),
]


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=project_root / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf",
    )
    parser.add_argument(
        "--torque-csv",
        type=Path,
        default=project_root / "04_structural_analysis" / "baseline_static_torque_samples.csv",
    )
    parser.add_argument("--out-dir", type=Path, default=project_root / "04_structural_analysis")
    parser.add_argument("--payload-kg", type=float, default=PAYLOAD_KG)
    parser.add_argument("--elastic-modulus-gpa", type=float, default=2.0)
    parser.add_argument("--reference-allowable-mpa", type=float, default=20.0)
    return parser.parse_args()


def xyz_to_array(text: str | None) -> np.ndarray:
    if not text:
        return np.zeros(3, dtype=float)
    return np.array([float(item) for item in text.split()], dtype=float)


def parse_urdf(urdf_path: Path) -> tuple[dict[str, float], dict[str, dict]]:
    root = ET.parse(urdf_path).getroot()
    link_masses: dict[str, float] = {}
    for link in root.findall("link"):
        name = link.attrib["name"]
        mass = 0.0
        inertial = link.find("inertial")
        if inertial is not None:
            mass_tag = inertial.find("mass")
            if mass_tag is not None:
                mass = float(mass_tag.attrib.get("value", "0"))
        link_masses[name] = mass

    joints: dict[str, dict] = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis = joint.find("axis")
        parent = joint.find("parent")
        child = joint.find("child")
        joints[joint.attrib["name"]] = {
            "type": joint.attrib["type"],
            "parent": parent.attrib["link"] if parent is not None else "",
            "child": child.attrib["link"] if child is not None else "",
            "origin_xyz": xyz_to_array(origin.attrib.get("xyz") if origin is not None else None),
            "axis_xyz": xyz_to_array(axis.attrib.get("xyz") if axis is not None else None),
        }
    return link_masses, joints


def read_torque_stats(torque_csv: Path, payload_kg: float) -> dict[str, dict[str, float]]:
    columns = {joint: f"tau_p{payload_kg:.2f}kg_{joint}_Nm" for joint in ARM_JOINT_NAMES}
    values = {joint: [] for joint in ARM_JOINT_NAMES}
    with torque_csv.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        missing = [column for column in columns.values() if column not in (reader.fieldnames or [])]
        if missing:
            raise RuntimeError(f"Missing expected torque columns: {missing}")
        for row in reader:
            for joint, column in columns.items():
                values[joint].append(abs(float(row[column])))

    stats: dict[str, dict[str, float]] = {}
    for joint, joint_values in values.items():
        arr = np.asarray(joint_values, dtype=float)
        stats[joint] = {
            "max_abs_nm": float(np.max(arr)),
            "p95_abs_nm": float(np.percentile(arr, 95)),
            "mean_abs_nm": float(np.mean(arr)),
        }
    return stats


def hollow_rect_i(b_m: float, h_m: float, wall_m: float) -> float:
    inner_b = max(b_m - 2.0 * wall_m, 0.0)
    inner_h = max(h_m - 2.0 * wall_m, 0.0)
    return (b_m * h_m**3 - inner_b * inner_h**3) / 12.0


def module_length(module: BeamModule, joints: dict[str, dict]) -> float:
    if module.length_source == "wrist_to_tool":
        wrist_roll = np.linalg.norm(joints["wrist_roll"]["origin_xyz"])
        gripper_frame = np.linalg.norm(joints["gripper_frame_joint"]["origin_xyz"])
        return float(wrist_roll + gripper_frame)
    return float(np.linalg.norm(joints[module.length_source]["origin_xyz"]))


def compute_rows(
    modules: list[BeamModule],
    link_masses: dict[str, float],
    joints: dict[str, dict],
    torque_stats: dict[str, dict[str, float]],
    payload_kg: float,
    elastic_modulus_pa: float,
    allowable_pa: float,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for module in modules:
        length_m = module_length(module, joints)
        b_m = module.b_mm / 1000.0
        h_m = module.h_mm / 1000.0
        wall_m = module.wall_mm / 1000.0
        second_moment_m4 = hollow_rect_i(b_m, h_m, wall_m)
        c_m = h_m / 2.0
        mass_without_payload = sum(link_masses.get(link, 0.0) for link in module.linked_masses)
        downstream_mass = mass_without_payload + payload_kg
        joint_stats = torque_stats[module.controlling_joint]
        p95_moment = joint_stats["p95_abs_nm"]
        peak_moment = joint_stats["max_abs_nm"]
        p95_stress = p95_moment * c_m / second_moment_m4
        peak_stress = peak_moment * c_m / second_moment_m4
        p95_rotation = p95_moment * length_m / (elastic_modulus_pa * second_moment_m4)
        peak_rotation = peak_moment * length_m / (elastic_modulus_pa * second_moment_m4)
        p95_deflection = p95_moment * length_m**2 / (2.0 * elastic_modulus_pa * second_moment_m4)
        peak_deflection = peak_moment * length_m**2 / (2.0 * elastic_modulus_pa * second_moment_m4)
        rows.append(
            {
                "module": module.name,
                "controlling_joint": module.controlling_joint,
                "length_m": length_m,
                "assumed_section_b_mm": module.b_mm,
                "assumed_section_h_mm": module.h_mm,
                "assumed_wall_mm": module.wall_mm,
                "second_moment_m4": second_moment_m4,
                "downstream_mass_kg_including_payload": downstream_mass,
                "p95_moment_nm": p95_moment,
                "peak_moment_nm": peak_moment,
                "p95_stress_mpa": p95_stress / 1e6,
                "peak_stress_mpa": peak_stress / 1e6,
                "p95_allowable_ratio": p95_stress / allowable_pa,
                "peak_allowable_ratio": peak_stress / allowable_pa,
                "p95_rotation_deg": math.degrees(p95_rotation),
                "peak_rotation_deg": math.degrees(peak_rotation),
                "p95_tip_deflection_mm": p95_deflection * 1000.0,
                "peak_tip_deflection_mm": peak_deflection * 1000.0,
                "role": module.role,
                "first_redesign_direction": module.first_redesign_direction,
                "keep_interfaces": module.keep_interfaces,
            }
        )

    max_moment = max(float(row["p95_moment_nm"]) for row in rows)
    max_rotation = max(float(row["p95_rotation_deg"]) for row in rows)
    max_mass = max(float(row["downstream_mass_kg_including_payload"]) for row in rows)
    for row in rows:
        torque_index = float(row["p95_moment_nm"]) / max_moment if max_moment > 0 else 0.0
        stiffness_index = float(row["p95_rotation_deg"]) / max_rotation if max_rotation > 0 else 0.0
        mass_index = float(row["downstream_mass_kg_including_payload"]) / max_mass if max_mass > 0 else 0.0
        interface_index = {"upper_arm_link": 1.0, "lower_arm_link": 0.85, "wrist_gripper_module": 0.75}.get(
            str(row["module"]), 0.5
        )
        row["risk_score_0_100"] = 100.0 * (
            0.40 * torque_index + 0.30 * stiffness_index + 0.20 * mass_index + 0.10 * interface_index
        )
    rows.sort(key=lambda item: float(item["risk_score_0_100"]), reverse=True)
    return rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        "module",
        "controlling_joint",
        "length_m",
        "assumed_section_b_mm",
        "assumed_section_h_mm",
        "assumed_wall_mm",
        "second_moment_m4",
        "downstream_mass_kg_including_payload",
        "p95_moment_nm",
        "peak_moment_nm",
        "p95_stress_mpa",
        "peak_stress_mpa",
        "p95_allowable_ratio",
        "peak_allowable_ratio",
        "p95_rotation_deg",
        "peak_rotation_deg",
        "p95_tip_deflection_mm",
        "peak_tip_deflection_mm",
        "risk_score_0_100",
        "first_redesign_direction",
        "keep_interfaces",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in fieldnames})


def write_summary(path: Path, rows: list[dict[str, object]], args: argparse.Namespace) -> None:
    top = rows[0]
    lines = [
        "# SO-101 Structural Bottleneck and Simplified Beam Stress Analysis",
        "",
        "## Purpose",
        "",
        "This report converts the baseline static torque result into structural redesign priorities for the official SO-101 Follower.",
        "",
        "## Source",
        "",
        f"- URDF: `{args.urdf}`",
        f"- Torque samples: `{args.torque_csv}`",
        f"- Payload used for structural prioritization: {args.payload_kg:.2f} kg",
        f"- Assumed elastic modulus for printed plastic comparison: {args.elastic_modulus_gpa:.2f} GPa",
        f"- Reference allowable stress for risk indexing: {args.reference_allowable_mpa:.1f} MPa",
        "",
        "## Method and Limits",
        "",
        "- Each module is approximated as a hollow rectangular cantilever beam at the weakest effective section.",
        "- Moments come from the Step 6 static torque analysis at the controlling joint.",
        "- The result is an engineering screening model, not FEA and not a certification result.",
        "- Absolute stress values depend on print orientation, filament, layer adhesion, local screw bosses, fillets, and real wall thickness.",
        "- The most useful output is the relative priority list and the interface constraints for AI-assisted redesign.",
        "",
        "## Prioritized Bottlenecks",
        "",
        "| rank | module | controlling joint | risk score | p95 moment Nm | peak moment Nm | p95 stress MPa | p95 rotation deg | first redesign direction |",
        "|---:|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for rank, row in enumerate(rows, start=1):
        lines.append(
            f"| {rank} | {row['module']} | {row['controlling_joint']} | {float(row['risk_score_0_100']):.1f} | "
            f"{float(row['p95_moment_nm']):.4f} | {float(row['peak_moment_nm']):.4f} | "
            f"{float(row['p95_stress_mpa']):.3f} | {float(row['p95_rotation_deg']):.3f} | "
            f"{row['first_redesign_direction']} |"
        )
    lines.extend(
        [
            "",
            "## Main Finding",
            "",
            f"- Highest structural priority: `{top['module']}`.",
            f"- Reason: it combines the highest upstream torque ({float(top['p95_moment_nm']):.4f} Nm at p95), large downstream mass, and critical joint-interface constraints.",
            "- The simplified stress values are not large under the assumed effective section, but stiffness, local screw-boss stress, layer orientation, and distal mass remain the practical design risks.",
            "",
            "## Design Implications",
            "",
            "- Do not change official joint axes or link lengths in the first AI redesign pass.",
            "- The first CAD redesign should focus on ribs, local box-section reinforcement, cable routing, lightening cutouts away from joint bosses, and distal mass reduction.",
            "- Base improvements should focus on mounting footprint and anti-tip fixture design rather than gravity torque around shoulder_pan.",
            "- After CAD rebuild, rerun workspace and torque checks before claiming improvement.",
            "",
            "## Generated Files",
            "",
            "- `04_structural_analysis/baseline_beam_stress_estimate.csv`",
            "- `04_structural_analysis/baseline_structural_bottleneck_plot.png`",
            "- `04_structural_analysis/baseline_structural_bottleneck_summary.md`",
            "- `04_structural_analysis/ai_structural_redesign_brief.md`",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_redesign_brief(path: Path, rows: list[dict[str, object]], joints: dict[str, dict]) -> None:
    lines = [
        "# AI Structural Redesign Brief for SO-101 Follower",
        "",
        "## Objective",
        "",
        "Use AI-assisted CAD generation to rebuild and improve selected SO-101 structural parts while preserving the official robot kinematics.",
        "",
        "## Must Preserve",
        "",
        "- Preserve official joint names, joint axes, and kinematic chain.",
        "- Preserve shoulder_lift, elbow_flex, wrist_flex, wrist_roll, and gripper frame locations in the first redesign pass.",
        "- Preserve servo mounting interfaces, horn clearance, screw patterns, and mesh/URDF link ownership.",
        "- Do not edit `00_source_snapshot/`; generate modified CAD under `05_improved_design/`.",
        "",
        "## Key Baseline Dimensions from URDF",
        "",
        f"- shoulder_lift to elbow_flex approximate span: {np.linalg.norm(joints['elbow_flex']['origin_xyz']) * 1000.0:.1f} mm",
        f"- elbow_flex to wrist_flex approximate span: {np.linalg.norm(joints['wrist_flex']['origin_xyz']) * 1000.0:.1f} mm",
        f"- wrist_flex to wrist_roll approximate span: {np.linalg.norm(joints['wrist_roll']['origin_xyz']) * 1000.0:.1f} mm",
        f"- wrist/gripper frame offset contribution: {np.linalg.norm(joints['gripper_frame_joint']['origin_xyz']) * 1000.0:.1f} mm",
        "",
        "## Redesign Targets",
        "",
    ]
    for rank, row in enumerate(rows, start=1):
        lines.extend(
            [
                f"### Target {rank}: {row['module']}",
                "",
                f"- Controlling joint: `{row['controlling_joint']}`",
                f"- Risk score: {float(row['risk_score_0_100']):.1f} / 100",
                f"- Baseline p95 moment: {float(row['p95_moment_nm']):.4f} Nm",
                f"- Simplified p95 rotation estimate: {float(row['p95_rotation_deg']):.3f} deg",
                f"- Improvement direction: {row['first_redesign_direction']}",
                f"- Interfaces to preserve: {row['keep_interfaces']}",
                "",
            ]
        )
    lines.extend(
        [
            "## Success Criteria for First Improved CAD Pass",
            "",
            "- Output editable build123d/Python CAD source plus STEP/STL sidecars.",
            "- Keep workspace reachability close to official baseline.",
            "- Reduce distal mass or improve effective weak-axis stiffness without changing kinematic frames.",
            "- Pass PyBullet URDF load and random-motion smoke test after mesh replacement.",
            "- Report tradeoffs clearly if mass, stiffness, or workspace changes in opposite directions.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_summary(path: Path, rows: list[dict[str, object]], allowable_mpa: float) -> None:
    names = [str(row["module"]) for row in rows]
    p95_stress = [float(row["p95_stress_mpa"]) for row in rows]
    peak_stress = [float(row["peak_stress_mpa"]) for row in rows]
    p95_rotation = [float(row["p95_rotation_deg"]) for row in rows]
    risk = [float(row["risk_score_0_100"]) for row in rows]
    moments = [float(row["p95_moment_nm"]) for row in rows]

    fig = plt.figure(figsize=(13, 9))
    x = np.arange(len(names))

    ax_stress = fig.add_subplot(2, 2, 1)
    ax_stress.bar(x - 0.17, p95_stress, 0.34, label="p95")
    ax_stress.bar(x + 0.17, peak_stress, 0.34, label="peak")
    ax_stress.axhline(allowable_mpa, color="tab:red", linestyle="--", linewidth=1.2, label="reference allowable")
    ax_stress.set_title("Simplified Bending Stress")
    ax_stress.set_ylabel("MPa")
    ax_stress.set_xticks(x)
    ax_stress.set_xticklabels(names, rotation=25, ha="right")
    ax_stress.legend()

    ax_rot = fig.add_subplot(2, 2, 2)
    ax_rot.bar(names, p95_rotation, color="tab:orange")
    ax_rot.set_title("Simplified p95 Tip Rotation")
    ax_rot.set_ylabel("deg")
    ax_rot.tick_params(axis="x", rotation=25)

    ax_risk = fig.add_subplot(2, 2, 3)
    ax_risk.bar(names, risk, color="tab:green")
    ax_risk.set_title("Redesign Priority Score")
    ax_risk.set_ylabel("0-100")
    ax_risk.set_ylim(0, max(100, max(risk) * 1.1))
    ax_risk.tick_params(axis="x", rotation=25)

    ax_scatter = fig.add_subplot(2, 2, 4)
    ax_scatter.scatter(moments, p95_rotation, s=[50 + item * 2 for item in risk], color="tab:blue", alpha=0.62)
    for name, moment, rotation in zip(names, moments, p95_rotation):
        ax_scatter.annotate(name, (moment, rotation), textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax_scatter.set_title("Torque vs Estimated Compliance")
    ax_scatter.set_xlabel("p95 moment / Nm")
    ax_scatter.set_ylabel("p95 rotation / deg")
    ax_scatter.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if not args.urdf.exists():
        raise FileNotFoundError(args.urdf)
    if not args.torque_csv.exists():
        raise FileNotFoundError(args.torque_csv)

    link_masses, joints = parse_urdf(args.urdf)
    torque_stats = read_torque_stats(args.torque_csv, args.payload_kg)
    rows = compute_rows(
        MODULES,
        link_masses,
        joints,
        torque_stats,
        args.payload_kg,
        args.elastic_modulus_gpa * 1e9,
        args.reference_allowable_mpa * 1e6,
    )

    write_csv(args.out_dir / "baseline_beam_stress_estimate.csv", rows)
    write_summary(args.out_dir / "baseline_structural_bottleneck_summary.md", rows, args)
    write_redesign_brief(args.out_dir / "ai_structural_redesign_brief.md", rows, joints)
    plot_summary(args.out_dir / "baseline_structural_bottleneck_plot.png", rows, args.reference_allowable_mpa)

    print(f"Saved beam estimate to {args.out_dir / 'baseline_beam_stress_estimate.csv'}")
    print(f"Saved bottleneck summary to {args.out_dir / 'baseline_structural_bottleneck_summary.md'}")
    print(f"Saved redesign brief to {args.out_dir / 'ai_structural_redesign_brief.md'}")
    print(f"Saved plot to {args.out_dir / 'baseline_structural_bottleneck_plot.png'}")
    print("priority:")
    for rank, row in enumerate(rows, start=1):
        print(
            f"{rank}. {row['module']} risk={float(row['risk_score_0_100']):.1f} "
            f"p95M={float(row['p95_moment_nm']):.4f}Nm "
            f"p95rot={float(row['p95_rotation_deg']):.3f}deg"
        )


if __name__ == "__main__":
    main()
