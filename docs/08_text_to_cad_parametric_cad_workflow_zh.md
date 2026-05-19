# Text-to-CAD机械结构参数化建模与工程校核流程

## 项目定位

本项目用于说明：AI 不是只生成一张机械臂外观图，而是参与“结构需求拆解、参数化建模、STEP/STL 导出、CAD 特征校核、装配检查、URDF 回归和 FEA 验证”的完整工程流程。

在简历中对应项目名称为：

> Text-to-CAD机械结构参数化建模与工程校核流程

## 为什么做这一步

直接用 AI 生成 CAD 或 mesh 容易停留在外观层面，常见问题包括：

- 孔位、沉孔和装配接口不准确。
- 生成的模型不能可靠导出 STEP/STL。
- mesh 能看但不能做真实装配或有限元网格。
- 修改结构后无法证明机械臂 URDF 运动链仍然正确。

因此本项目把 Text-to-CAD 思路改造成可验证流程：AI 负责辅助提出结构方案、拆解参数和生成脚本，工程脚本负责 CAD 导出、孔位校核、装配检查和仿真验证。

## 技术路线

```mermaid
flowchart LR
  A["文本需求 / 结构目标"] --> B["AI 辅助参数拆解"]
  B --> C["build123d / OCP 参数化 CAD"]
  C --> D["STEP / STL 导出"]
  D --> E["STEP B-Rep 孔位与圆柱特征校核"]
  E --> F["标准件装配检查"]
  F --> G["URDF / PyBullet smoke test"]
  G --> H["Gmsh / CalculiX FEA 验证"]
  H --> I["下一版结构改进建议"]
```

## 在本项目中的实现

### 1. CAD 环境与仿真环境拆分

CAD 环境主要用于：

- `build123d`
- `OCP / OpenCascade`
- `FreeCADCmd`
- `Gmsh`
- STEP/STL 导出

仿真环境主要用于：

- `PyBullet`
- URDF 加载
- 工作空间采样
- 静载力矩估算
- 运动学 smoke test

这样做的原因是 CAD 内核、OpenCascade、PyBullet 和机器人学习依赖容易冲突。环境拆分后，CAD 生成和仿真验证可以各自稳定运行。

### 2. 参数化建模对象

第一阶段选取 SO-101 follower arm 的 `upper_arm_link`，因为它同时具备：

- 明确的机械臂结构功能。
- 与 shoulder / elbow 关节相关的装配接口。
- 对整机刚度、质量和静载力矩有明显影响。
- 适合做轻量化、孔位修正、肋板布局和有限元对比。

### 3. V1 / V2 / V3 迭代

| 版本 | 目标 | 主要变化 | 校核结果 |
|---|---|---|---|
| V1 | 轻量化概念重建 | 保留主关节中心距，建立基本梁体和主孔 | 能导出 STEP/STL，但关键装配接口不足 |
| V2 | 补齐装配接口 | 增加 M3 安装孔、沉孔、肩部夹紧/定位孔 | 装配表达更完整，但肘部孔系材料余量偏紧 |
| V3 | 修正孔系余量 | 调整肘部孔位、孔径、沉孔和局部 pad | 孔位复核 PASS，标准件装配检查 PASS |

## 工程校核方法

### STEP 特征级校核

不是只看 STL 外观，而是读取 STEP / B-Rep 中的圆柱面特征，检查：

- 主关节孔直径。
- M3 安装孔直径。
- 沉孔直径和深度。
- 孔轴方向。
- 孔中心距。
- 孔边材料余量。
- shoulder clamp 和 elbow mount 的装配接口。

相关数据位于：

- `data/cad/upper_arm_v1_mounting_check.json`
- `data/cad/upper_arm_v2_mounting_check.json`
- `data/cad/upper_arm_v3_mounting_check.json`
- `data/cad/upper_arm_v2_standard_part_checks.csv`
- `data/cad/upper_arm_v3_standard_part_checks.csv`

### URDF / PyBullet 回归

将改进后的 upper arm mesh 接回 URDF 后，进行 2001 个姿态的 smoke test，检查末端位置是否与 baseline 保持一致。V3 的最大末端位置差为 `0`，说明视觉 mesh 替换没有破坏机械臂原运动链。

相关代码位于：

- `src/cad/upper_arm_v1/upper_arm_v1_urdf_smoke.py`
- `src/cad/upper_arm_v2/upper_arm_v2_urdf_smoke.py`
- `src/cad/upper_arm_v3/upper_arm_v3_urdf_smoke.py`

### FEA 连接

Text-to-CAD 生成的结构不是最终结论，还需要进入 CAE。V3 在 full clean FEA 中可以用 5 mm 四面体网格完成 Gmsh / CalculiX 静力分析。

结论是：V3 质量降低，但刚度下降明显，后续 V4 需要加强中部梁、上下梁连接和肋板布局。

## 对应文件

CAD 脚本：

- `src/cad/upper_arm_v1/upper_arm_v1_cad.py`
- `src/cad/upper_arm_v2/upper_arm_v2_cad.py`
- `src/cad/upper_arm_v3/upper_arm_v3_cad.py`

模型文件：

- `models/upper_arm_v1/upper_arm_v1_ai_rebuild.step`
- `models/upper_arm_v2/upper_arm_v2_ai_rebuild.step`
- `models/upper_arm_v3/upper_arm_v3_ai_rebuild.step`

装配模型：

- `models/assemblies/upper_arm_v2_simplified_standard_parts_assembly.step`
- `models/assemblies/upper_arm_v3_simplified_standard_parts_assembly.step`

可视化图片：

- `figures/cad/upper_arm_official_v1_v2_v3_isometric.png`
- `figures/cad/upper_arm_v1_v2_v3_three_views.png`
- `figures/cad/upper_arm_v2_v3_elbow_hole_zoom.png`

## 简历中可以这样讲

我做的 Text-to-CAD 项目不是简单让 AI 画机械臂，而是把自然语言结构需求转成参数化 CAD 脚本，再导出 STEP/STL，并通过 STEP B-Rep 特征校核、标准件装配检查、URDF/PyBullet 回归和 FEA 验证结果。这个流程能证明 AI 生成的结构不是停留在外观层面，而是可以进入机械工程验证链路。
