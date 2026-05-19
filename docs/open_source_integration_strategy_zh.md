# SO-101 项目开源栈整合与改进路线

## 为什么要调整路线

前面已经完成了 official baseline、upper arm V1/V2/V3、CAD 校核、URDF/PyBullet、FEA screening 和 full V3 CAE 几何清理。这个过程证明了我们能把一个结构件从 CAD 改到仿真验证，但还有一个明显短板：目前更多是自建脚本闭环，没有充分利用成熟开源项目的完整工程能力。

如果目标是机械/CAE/AI 结构方向实习作品集，后续主线应从“我自己写脚本做了一个机械臂零件”升级为：

> 基于 SO-101 开源机械臂，复现官方 CAD/URDF/仿真资产；整合 ROS2/MoveIt/LeRobot/CAE 开源工具链；用 AI 辅助做结构改进，并用运动学、装配、有限元和可视化结果验证改进效果。

## 当前已经做了什么

- 已使用 SO-ARM100/SO-101 官方项目中的 STEP、STL、URDF、MJCF 和 Simulation assets。
- 已复现官方 SO-101 follower arm 的 URDF 加载、关节解析、随机运动、工作空间和静载力矩分析。
- 已选定 `upper_arm_link` 作为第一轮结构改进对象。
- 已完成 V1/V2/V3 参数化 CAD 重建、STEP/STL 导出、孔位校核、标准件装配检查、URDF smoke test 和力矩回归。
- 已完成 official / V3 的初步 FEA、接口边界 FEA，以及 full V3 clean mesh 的完整孔系 FEA。

## 当前不足

- CAD 部分主要是基于 build123d/OCP 的局部重建，还没有形成“官方装配 -> 参数化改件 -> 自动回写机器人描述”的完整开源 CAD pipeline。
- 仿真部分主要用 PyBullet smoke test，还没有接入更完整的 ROS2 / MoveIt / Rerun / Foxglove 等可视化和运动规划栈。
- FEA 部分目前是自建线性静力 screening solver，适合学习和快速对比，但不够像工业 CAE 工作流；后续应接入 Gmsh + CalculiX/FreeCAD FEM，生成 `.inp`、求解结果和 ParaView/VTK 后处理。
- 项目叙事还没有突出“我整合了哪些开源系统，并在其基础上做了结构优化”。

## 推荐整合的开源项目

| 角色 | 推荐项目 | 用在本项目中的作用 |
|---|---|---|
| 官方机械臂 baseline | `TheRobotStudio/SO-ARM100` | 保留为唯一 baseline 来源：CAD、STL、URDF、MJCF、装配说明、BOM。 |
| 机器人学习生态 | `huggingface/lerobot` | 作为 SO-101 的 AI/机器人学习上层生态，后续展示模仿学习或策略控制接口。 |
| ROS2 + MoveIt + Rerun | `legalaspro/so101-ros-physical-ai` | 用来补齐可视化、运动规划、真实/仿真机器人数据流，让项目不止停留在单个 PyBullet smoke test。 |
| CAD 到机器人描述 | `Rhoban/onshape-to-robot` | 用来对齐官方“CAD -> URDF/MJCF”思想，后续可研究如何把改进件更规范地接回机器人描述。 |
| CAD as code | `build123d` / `CadQuery` / 当前 `text-to-cad` | 继续用于 AI 辅助参数化结构改进，但要把它放进开源 pipeline，而不是单独存在。 |
| 网格与 CAE | `Gmsh` + `CalculiX` / `FreeCAD FEM` | 把 FEA 从自建 screening 升级为更接近真实 CAE 的开源求解流程。 |
| 结果后处理 | `meshio` / `PyVista` / `ParaView` | 输出 VTK/VTU、应力云图、位移云图，便于作品集展示。 |

## 新项目主线

### 阶段 A：开源项目复现

目标：不是马上改结构，而是证明我能复现并理解完整开源机械臂工程。

要做：
- 复现 SO-ARM100/SO-101 官方 CAD、STL、URDF、MJCF 和 Simulation assets。
- 建立 source snapshot，不直接改官方文件。
- 记录官方项目结构、装配模块、每个 link 的机械作用。
- 用 PyBullet 或 ROS2 加载官方 URDF，输出关节表、质量表、运动范围和工作空间。

产出：
- 官方资源清单。
- 官方结构模块拆解。
- 官方 URDF/运动学验证报告。
- 官方 baseline 可视化图。

### 阶段 B：开源仿真与可视化栈整合

目标：把项目从“能加载 URDF”提升到“能在成熟工具中看运动、看轨迹、看规划”。

要做：
- 优先复现 `so101-ros-physical-ai` 中的 ROS2/MoveIt/Rerun 工作流。
- 将官方 SO-101 URDF 接入 MoveIt planning scene。
- 导出或记录轨迹、关节角、末端路径。
- 后续将 V3/V4 改进件替换到 URDF 中，比较可视化运动效果。

产出：
- ROS2/MoveIt/Rerun 复现报告。
- 官方模型运动轨迹图。
- 改进模型接回仿真栈的对比图。

### 阶段 C：开源 CAE 工具链升级

目标：把当前 FEA screening 升级为更像工业岗位的开源 CAE 流程。

要做：
- 使用 Gmsh 负责 STEP/B-Rep 体网格。
- 使用 CalculiX 或 FreeCAD FEM 负责线性静力求解。
- 生成 `.inp` 求解文件、`.frd/.dat` 或等效结果文件。
- 用 meshio/PyVista/ParaView 做后处理。
- 保留当前自建 solver 作为快速 screening，对比开源求解器结果。

产出：
- official / V3 / V4 的 CalculiX 输入文件。
- official / V3 / V4 的应力、位移和安全系数结果。
- 网格收敛性检查：例如 7 mm、5 mm、3 mm 对比。

### 阶段 D：AI 辅助结构改进

目标：AI 不只是画模型，而是参与“问题定位 -> 结构方案 -> 参数化建模 -> 仿真验证 -> 迭代”的闭环。

要做：
- 根据 FEA 和运动学结果定位问题：V3 更轻，但刚度不足。
- 让 AI 生成 V4 设计约束：加强中部梁、上下梁连接、肋板布局，保留接口孔位。
- 用 build123d/text-to-cad 实现参数化 V4。
- 自动导出 STEP/STL。
- 自动跑 CAD 孔位检查、标准件装配检查、URDF smoke、力矩回归、FEA。

产出：
- V4 参数化 CAD 源码。
- V4 STEP/STL/URDF 替换件。
- official / V3 / V4 对比报告。
- 结构改进前后图、应力图、位移图、质量和安全系数表。

## 现在最应该做的下一步

下一步不急着继续 V4，而是做：

> Step 21：开源项目整合审计与复现计划。

具体做三件事：

1. 在服务器建立 `08_open_source_integration/`，记录已使用和待整合的开源项目。
2. 对 `SO-ARM100`、`LeRobot`、`so101-ros-physical-ai`、`onshape-to-robot`、`Gmsh/CalculiX` 做可运行性检查。
3. 决定后续主线：先复现 ROS2/MoveIt/Rerun 可视化栈，再把 V3/V4 改进件接进去；FEA 同时从自建 solver 升级到 CalculiX/FreeCAD FEM。

## 面试讲述方式

可以这样讲：

> 我不是从零造一个没有来源的机械臂，而是选择了 SO-101 这个开源机械臂作为 baseline。第一阶段我复现了它的 CAD、URDF 和仿真资源；第二阶段我用 AI 辅助对 upper arm 做参数化重建和结构轻量化；第三阶段我发现轻量化带来装配孔余量和刚度问题，于是用 CAD 特征校核、标准件检查和有限元结果迭代到 V3；下一阶段我把项目升级为开源栈整合，接入 ROS2/MoveIt/Rerun 和 CalculiX/FreeCAD FEM，使结构改进能在更完整的机器人仿真和 CAE 流程中验证。

这比单纯说“我用 AI 画了一个机械臂”更有说服力，因为它体现的是开源工程复现、工具链整合、结构优化和验证闭环。
