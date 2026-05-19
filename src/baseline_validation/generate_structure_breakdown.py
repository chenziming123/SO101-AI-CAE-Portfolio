from __future__ import annotations

import csv
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PROJECT_ROOT / "00_source_snapshot" / "Simulation_SO101" / "so101_new_calib.urdf"
OUT_DIR = PROJECT_ROOT / "02_baseline_validation"


MODULE_NOTES = {
    "base_link": {
        "module": "Base / electronics mounting module",
        "function": "Provides the fixed base, base servo holder, controller mounting plate, and the first yaw joint support.",
        "load_path": "Transfers all arm weight and payload reaction forces into the desk or external fixture.",
        "source_step": "`Base_SO101.step`, `Base_motor_holder_SO101.step`, `WaveShare_Mounting_Plate_SO101.step`",
        "improvement": "Improve desktop mounting stability, add cable routing, add broader bolt pattern or fixture interface.",
    },
    "shoulder_link": {
        "module": "Shoulder pitch module",
        "function": "Carries the shoulder motor holder and rotation-pitch bracket for lifting the upper arm.",
        "load_path": "Receives torque from shoulder_lift and carries downstream upper/lower arm, wrist, gripper, and payload.",
        "source_step": "`Motor_holder_SO101_Base.step`, `Rotation_Pitch_SO101.step`",
        "improvement": "Check bracket stiffness around servo screws and add local fillets/ribs without changing joint axis.",
    },
    "upper_arm_link": {
        "module": "Upper arm link",
        "function": "Main structural link between shoulder and elbow; includes printed arm geometry and elbow-side servo.",
        "load_path": "Bending-dominant member carrying lower arm, wrist, gripper, and payload.",
        "source_step": "`Upper_arm_SO101.step`",
        "improvement": "Add ribbing, lightweight cutouts, wire channel, and printing-friendly reinforcement.",
    },
    "lower_arm_link": {
        "module": "Under arm / forearm link",
        "function": "Second arm link between elbow and wrist, including wrist motor holder.",
        "load_path": "Carries wrist and gripper loads; distal mass strongly affects shoulder and elbow torque.",
        "source_step": "`Under_arm_SO101.step`, `Motor_holder_SO101_Wrist.step`",
        "improvement": "Reduce distal mass, improve local wrist-holder stiffness, add strain relief for cables.",
    },
    "wrist_link": {
        "module": "Wrist flex module",
        "function": "Supports wrist flex and prepares the wrist-roll/gripper interface.",
        "load_path": "Transfers gripper and payload forces through the wrist_flex joint into the forearm.",
        "source_step": "`Wrist_Roll_Pitch_SO101.step`",
        "improvement": "Improve compactness, camera/tool mounting, and cable clearance around the wrist.",
    },
    "gripper_link": {
        "module": "Wrist roll / fixed gripper body",
        "function": "Provides wrist roll output and fixed side of the follower gripper.",
        "load_path": "Carries gripping reaction forces and payload contact forces.",
        "source_step": "`Follower_Specific/Wrist_Roll_Follower_SO101.step`",
        "improvement": "Add standard tool flange, replaceable fingertips, or anti-slip pad interface.",
    },
    "moving_jaw_so101_v1_link": {
        "module": "Moving jaw",
        "function": "Movable jaw of the gripper driven by the gripper revolute joint.",
        "load_path": "Receives local gripping contact forces and transfers them through the gripper hinge.",
        "source_step": "`Follower_Specific/Moving_Jaw_SO101.step`",
        "improvement": "Improve jaw contact geometry, add replaceable fingertip insert, or tune opening range.",
    },
    "gripper_frame_link": {
        "module": "End-effector reference frame",
        "function": "Dummy frame used as a stable end-effector coordinate reference.",
        "load_path": "No physical structural role; used for kinematic measurement.",
        "source_step": "URDF dummy link",
        "improvement": "Keep as the reporting frame for workspace and reachability analysis.",
    },
}


def parse_urdf():
    tree = ET.parse(URDF_PATH)
    root = tree.getroot()
    links = {}
    for link in root.findall("link"):
        name = link.attrib["name"]
        inertial = link.find("inertial")
        mass = 0.0
        origin = ""
        if inertial is not None:
            mass_node = inertial.find("mass")
            origin_node = inertial.find("origin")
            if mass_node is not None:
                mass = float(mass_node.attrib.get("value", "0"))
            if origin_node is not None:
                origin = origin_node.attrib.get("xyz", "")
        meshes = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                meshes.append(mesh.attrib.get("filename", ""))
        links[name] = {"mass": mass, "origin": origin, "meshes": meshes}

    joints = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        axis = joint.find("axis")
        origin = joint.find("origin")
        parent = joint.find("parent")
        child = joint.find("child")
        lower = upper = effort = velocity = ""
        if limit is not None:
            lower = limit.attrib.get("lower", "")
            upper = limit.attrib.get("upper", "")
            effort = limit.attrib.get("effort", "")
            velocity = limit.attrib.get("velocity", "")
        joints.append(
            {
                "name": joint.attrib["name"],
                "type": joint.attrib.get("type", ""),
                "parent": parent.attrib.get("link", "") if parent is not None else "",
                "child": child.attrib.get("link", "") if child is not None else "",
                "axis": axis.attrib.get("xyz", "") if axis is not None else "",
                "origin": origin.attrib.get("xyz", "") if origin is not None else "",
                "lower": lower,
                "upper": upper,
                "effort": effort,
                "velocity": velocity,
            }
        )
    return links, joints


def write_link_mass_csv(links: dict[str, dict]) -> None:
    with (OUT_DIR / "so101_link_mass_summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["link", "mass_kg", "inertial_origin_xyz", "visual_mesh_count", "visual_meshes"])
        writer.writeheader()
        for name, data in links.items():
            writer.writerow(
                {
                    "link": name,
                    "mass_kg": data["mass"],
                    "inertial_origin_xyz": data["origin"],
                    "visual_mesh_count": len(data["meshes"]),
                    "visual_meshes": "; ".join(data["meshes"]),
                }
            )


def write_kinematic_chain(joints: list[dict]) -> None:
    lines = [
        "flowchart LR",
        '  base["base_link"]',
    ]
    for joint in joints:
        parent = joint["parent"]
        child = joint["child"]
        label = f"{joint['name']} ({joint['type']})"
        lines.append(f'  {parent} -->|"{label}"| {child}')
    (OUT_DIR / "so101_kinematic_chain.mmd").write_text("\n".join(lines) + "\n", encoding="utf-8")


def deg_range(joint: dict) -> str:
    if not joint["lower"] or not joint["upper"] or joint["type"] == "fixed":
        return "0.0"
    try:
        import math

        return f"{math.degrees(float(joint['upper']) - float(joint['lower'])):.1f}"
    except ValueError:
        return ""


def write_breakdown(links: dict[str, dict], joints: list[dict]) -> None:
    total_mass = sum(data["mass"] for data in links.values())
    active_arm_joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    lines = [
        "# SO-101 Follower 结构拆解报告",
        "",
        "## 1. 目的",
        "",
        "这份报告用于把官方 SO-101 Follower 从文件集合拆解成可解释的机械结构模块。",
        "它是后续 AI 辅助参数化复现和结构改进之前的 baseline 结构理解文档。",
        "",
        "## 2. 官方模型来源",
        "",
        f"- URDF: `{URDF_PATH}`",
        "- CAD: `00_source_snapshot/STEP_SO101/`",
        "- STL: `00_source_snapshot/STL_SO101/` and `00_source_snapshot/Simulation_SO101/assets/`",
        "- Simulation README 说明：URDF/MJCF 由 Onshape CAD 通过 onshape-to-robot 生成，mesh 使用相对路径，base collision 曾因仿真问题被移除。",
        "",
        "## 3. 整体结构概览",
        "",
        f"- Link 数量：{len(links)}",
        f"- Joint 数量：{len(joints)}",
        f"- URDF 质量合计：{total_mass:.3f} kg",
        f"- 主机械臂关节：{', '.join(active_arm_joints)}",
        "- 夹爪关节：`gripper`",
        "- 末端参考 frame：`gripper_frame_link`",
        "",
        "## 4. Kinematic Chain",
        "",
        "```mermaid",
        "flowchart LR",
        '  base_link["base_link"]',
    ]
    for joint in joints:
        lines.append(f'  {joint["parent"]} -->|"{joint["name"]} ({joint["type"]})"| {joint["child"]}')
    lines.extend(
        [
            "```",
            "",
            "## 5. Joint 分析",
            "",
            "| Joint | Type | Parent | Child | Axis | Range deg | 工程含义 |",
            "| --- | --- | --- | --- | --- | ---: | --- |",
        ]
    )
    joint_meaning = {
        "shoulder_pan": "底座 yaw，决定整臂水平转向",
        "shoulder_lift": "肩部 pitch，主要承担抬臂和负载力矩",
        "elbow_flex": "肘部 pitch，决定前臂伸展/收回",
        "wrist_flex": "腕部 pitch，调整末端姿态",
        "wrist_roll": "腕部 roll，调整夹爪旋转角",
        "gripper_frame_joint": "末端参考 frame 固定连接",
        "gripper": "夹爪开合，URDF 中为 revolute",
    }
    for joint in joints:
        lines.append(
            f"| `{joint['name']}` | {joint['type']} | `{joint['parent']}` | `{joint['child']}` | "
            f"`{joint['axis']}` | {deg_range(joint)} | {joint_meaning.get(joint['name'], '')} |"
        )

    lines.extend(
        [
            "",
            "## 6. Link / Module 拆解",
            "",
            "| Link | Mass kg | Module | Function | Source CAD | Improvement direction |",
            "| --- | ---: | --- | --- | --- | --- |",
        ]
    )
    for link_name, data in links.items():
        note = MODULE_NOTES.get(
            link_name,
            {
                "module": "",
                "function": "",
                "source_step": "",
                "improvement": "",
            },
        )
        lines.append(
            f"| `{link_name}` | {data['mass']:.3f} | {note['module']} | {note['function']} | "
            f"{note['source_step']} | {note['improvement']} |"
        )

    lines.extend(
        [
            "",
            "## 7. 受力路径理解",
            "",
            "SO-101 的主要静载路径可以理解为：",
            "",
            "```text",
            "payload / gripping force",
            "-> moving jaw / gripper body",
            "-> wrist roll",
            "-> wrist flex module",
            "-> lower arm",
            "-> elbow flex",
            "-> upper arm",
            "-> shoulder lift",
            "-> shoulder pan / base",
            "-> desktop fixture",
            "```",
            "",
            "机械分析优先级：",
            "",
            "1. 肩部 `shoulder_lift`：承担整个下游结构和负载，通常是最大力矩约束。",
            "2. 肘部 `elbow_flex`：承担 lower arm、wrist、gripper 和 payload。",
            "3. 腕部 `wrist_flex` / `wrist_roll`：影响末端姿态、夹爪稳定性和线束布置。",
            "4. Base：影响桌面安装稳定性、整体抗倾覆和电控安装。",
            "",
            "## 8. 适合优先改进的模块",
            "",
            "| Priority | Module | 为什么适合改 | 第一版建议 |",
            "| ---: | --- | --- | --- |",
            "| 1 | upper_arm_link | 主承力连杆，容易通过截面和加强筋优化 | 加线束槽、局部加强筋、轻量化孔位对比 |",
            "| 2 | base_link | 影响桌面稳定性和安装体验 | 增加安装孔、加宽底座、增加配重/夹持接口 |",
            "| 3 | gripper_link / moving_jaw | 末端最直观，适合展示设计改进 | 可换指尖、防滑垫、标准小法兰 |",
            "| 4 | lower_arm_link | 远端质量影响力矩 | 减重和局部加强，但要谨慎保持腕部装配 |",
            "",
            "## 9. 第一版不建议改动的内容",
            "",
            "- 不建议改 5 个主关节轴线，否则 baseline 对比会变复杂。",
            "- 不建议更换舵机型号，否则会牵连电控、安装孔和控制模型。",
            "- 不建议重做整个腕部传动，因为腕部结构复杂，容易破坏装配。",
            "- 不建议直接修改 `00_source_snapshot/`，应在 `03_ai_rebuild/` 或 `05_improved_design/` 中工作。",
            "",
            "## 10. 下一步",
            "",
            "下一步应做官方 baseline 的工作空间和力矩分析：",
            "",
            "```text",
            "04_structural_analysis/baseline_workspace_summary.md",
            "04_structural_analysis/baseline_torque_estimate.md",
            "04_structural_analysis/baseline_beam_stress_estimate.md",
            "```",
            "",
            "这些指标会作为后续 AI 改进版的对照基准。",
        ]
    )
    (OUT_DIR / "so101_structure_breakdown.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    links, joints = parse_urdf()
    write_link_mass_csv(links)
    write_kinematic_chain(joints)
    write_breakdown(links, joints)
    print(f"Wrote {OUT_DIR / 'so101_structure_breakdown.md'}")
    print(f"Wrote {OUT_DIR / 'so101_link_mass_summary.csv'}")
    print(f"Wrote {OUT_DIR / 'so101_kinematic_chain.mmd'}")


if __name__ == "__main__":
    main()
