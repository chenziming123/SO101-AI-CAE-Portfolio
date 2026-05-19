# SO-101 结构瓶颈与简化梁应力分析

## 目的

本报告把官方 SO-101 Follower 的静载力矩分析结果转化为结构改进优先级，为后续 AI 辅助 CAD 重建提供依据。

## 数据来源

- URDF：`<repo>/00_source_snapshot/Simulation_SO101/so101_new_calib.urdf`
- 力矩采样数据：`<repo>/04_structural_analysis/baseline_static_torque_samples.csv`
- 结构筛查使用 payload：0.10 kg
- 假设打印塑料弹性模量：2.00 GPa
- 风险索引用参考许用应力：20.0 MPa

## 方法与限制

- 把每个结构模块近似为弱截面处的空心矩形悬臂梁。
- 弯矩来自 Step 6 静载关节力矩分析中的控制关节。
- 本结果是工程筛查模型，不是有限元仿真，也不是强度认证。
- 绝对应力值会受到打印方向、材料、层间结合、局部螺丝柱、圆角、真实壁厚和线槽影响。
- 本报告最有价值的部分是相对优先级和 AI 重建设计约束。

## 结构瓶颈优先级

| 排名 | 模块 | 控制关节 | 风险分数 | p95 弯矩 Nm | 峰值弯矩 Nm | p95 应力 MPa | p95 转角 deg | 第一轮改进方向 |
|---:|---|---|---:|---:|---:|---:|---:|---|
| 1 | upper_arm_link | shoulder_lift | 100.0 | 1.1496 | 1.2649 | 4.003 | 2.660 | 通过加强筋、盒式截面和线槽提高弱轴刚度，同时不移动肩部和肘部关节 frame。 |
| 2 | lower_arm_link | elbow_flex | 73.4 | 0.7142 | 0.7415 | 2.790 | 2.158 | 降低远端质量，并在腕部电机座和线束应力释放区域增加局部加强。 |
| 3 | wrist_gripper_module | wrist_flex | 53.2 | 0.2725 | 0.2733 | 1.889 | 2.194 | 降低远端质量，增加紧凑局部加强筋，并加入可换指尖或小型工具法兰。 |

## 主要结论

- 最高优先级结构件：`upper_arm_link`。
- 原因：它同时承受最高上游力矩、较大下游质量，并且肩部/肘部接口约束关键。
- 在假设截面下，简化应力值并不高；但真实工程风险更多来自局部螺丝柱、打印层方向、薄壁区域、线槽削弱和远端质量。

## 设计启示

- 第一轮 AI 重建设计不要改变官方关节轴线和连杆长度。
- CAD 改进重点应放在加强筋、局部盒式截面、线束通道、避开关节柱的轻量化孔和远端减重。
- base 的改进重点不是 shoulder_pan 的重力力矩，而是桌面安装面积、防倾覆和固定方式。
- CAD 重建后必须重新跑工作空间、力矩和 PyBullet 加载检查，再说明是否真正改进。

## 生成文件

- `04_structural_analysis/baseline_beam_stress_estimate.csv`
- `04_structural_analysis/baseline_structural_bottleneck_plot.png`
- `04_structural_analysis/baseline_structural_bottleneck_summary.md`
- `04_structural_analysis/baseline_structural_bottleneck_summary_zh.md`
- `04_structural_analysis/ai_structural_redesign_brief.md`
- `04_structural_analysis/ai_structural_redesign_brief_zh.md`
