# text-to-cad 与开源工具链整合说明

## 为什么引入 text-to-cad

本项目不是简单手工改一个 STL，而是希望形成“AI 提示词 / 参数化脚本 / CAD 文件 / URDF / FEA”的链路。`text-to-cad` 的价值在于提供 CAD-as-code 思路：把自然语言设计意图转成可编辑、可复用、可导出的建模脚本。

## 在本项目中的角色

- 用 AI 辅助梳理 upper arm 的结构参数、孔位和装配关系。
- 用 Python 参数化脚本表达 V1/V2/V3 的结构版本。
- 输出 STEP/STL，进入后续 CAD 校核、URDF smoke test 和 FEA。
- 将 AI 生成从“外观模型”推进到“可验证工程模型”。

## 工具链拆分

项目采用两个环境：

- CAD 环境：`build123d / OCP / Gmsh / meshio`，负责建模、STEP/STL 导出和网格处理。
- 仿真环境：`PyBullet / numpy / scipy / matplotlib`，负责 baseline 验证、工作空间和力矩分析。

拆分环境的原因是 CAD 内核、OpenCascade、PyBullet 和机器人学习依赖容易冲突。把 CAD 和仿真环境分开，更接近真实工程项目中的工具链管理方式。

## 目前边界

当前 text-to-cad 不是直接“一句话生成最终机械臂”，而是参与结构方案生成、参数化脚本编写和流程组织。最终结果仍由 STEP 特征、装配孔位、URDF 回归和 FEA 结果验证。
