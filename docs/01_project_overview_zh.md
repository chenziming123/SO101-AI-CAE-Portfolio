# 项目总览：SO-101 AI 辅助机械臂结构建模与 CAE 验证

## 项目目标

围绕开源 SO-101 follower arm，构建一个面向机械结构/CAE 岗位的作品集项目。项目强调“AI 参与工程流程”，但不把 AI 输出当作最终答案，而是用 CAD 特征校核、URDF 运动学回归、静载力矩分析和有限元结果反向约束设计。

## 技术路线

```mermaid
flowchart LR
  A["开源 SO-101 baseline"] --> B["CAD/STL/URDF 资产盘点"]
  B --> C["PyBullet baseline 运动验证"]
  C --> D["工作空间和静载力矩分析"]
  D --> E["锁定 upper_arm_link 优化对象"]
  E --> F["AI + 参数化 CAD 生成 V1/V2/V3"]
  F --> G["STEP/STL 导出与孔位校核"]
  G --> H["URDF smoke test 与力矩回归"]
  H --> I["Gmsh/CalculiX 静力有限元"]
  I --> J["V4 结构加强方向"]
```

## 作品集中能体现的能力

- 机械结构拆解：从整机中定位 upper arm 的结构和受力角色。
- CAD 建模：用参数化脚本生成可导出的 STEP/STL，而不是只做图片。
- 装配意识：检查孔位、沉孔、标准件和接口余量。
- 仿真闭环：把 CAD 改动接回 URDF/PyBullet 验证运动链。
- CAE 能力：搭建 Gmsh/CalculiX 有限元流程，输出应力/位移/安全系数。
- AI 辅助工程：用 AI 提高建模、脚本、报告和设计迭代效率，同时用工程校核约束 AI 结果。

## 当前阶段结论

V3 已经实现了从 CAD 到 URDF 到 FEA 的完整闭环。它相对 official 更轻，但 full V3 FEA 结果显示刚度下降明显，下一阶段应基于 FEA 结果设计 V4，加强中部梁、上下梁连接和肋板布局。
