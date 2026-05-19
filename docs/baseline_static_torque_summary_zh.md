# SO-101 官方 Baseline 静载关节力矩分析

## 目的

本报告用于估算官方 SO-101 Follower 在未进行 AI 辅助结构改进前的静载关节力矩需求。

## 数据来源与假设

- URDF：`<repo>/00_source_snapshot/Simulation_SO101/so101_new_calib.urdf`
- 末端负载作用点：`gripper_frame_link`
- 采样主关节：`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`
- PyBullet 逆动力学使用的活动关节：`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`、`gripper`
- 随机静态姿态数量：10000
- 重力加速度：9.81 m/s^2
- 末端 payload：0.00 kg、0.05 kg、0.10 kg、0.20 kg
- 主要展示 payload：0.10 kg

计算方法：

- 机械臂自身重力力矩：使用 PyBullet inverse dynamics，令 `qdot=0`、`qddot=0`。
- 末端 payload 力矩：把 payload 视为作用在末端的竖直向下外力，通过 `-J^T F` 换算到各关节。
- 本结果是静载估算，不包括加速度力矩、摩擦、线束拖拽、接触冲击、舵机热衰减和厂家完整力矩曲线。

## 力矩统计

| payload kg | 关节 | 最大绝对力矩 Nm | 95 分位绝对力矩 Nm | 平均绝对力矩 Nm | RMS Nm |
|---:|---|---:|---:|---:|---:|
| 0.00 | shoulder_pan | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 0.00 | shoulder_lift | 0.8630 | 0.7950 | 0.3712 | 0.4482 |
| 0.00 | elbow_flex | 0.4529 | 0.4379 | 0.2650 | 0.2951 |
| 0.00 | wrist_flex | 0.1171 | 0.1167 | 0.0739 | 0.0822 |
| 0.00 | wrist_roll | 0.0024 | 0.0022 | 0.0010 | 0.0012 |
| 0.05 | shoulder_pan | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 0.05 | shoulder_lift | 1.0640 | 0.9714 | 0.4522 | 0.5461 |
| 0.05 | elbow_flex | 0.5972 | 0.5758 | 0.3438 | 0.3835 |
| 0.05 | wrist_flex | 0.1952 | 0.1946 | 0.1232 | 0.1371 |
| 0.05 | wrist_roll | 0.0015 | 0.0014 | 0.0007 | 0.0008 |
| 0.10 | shoulder_pan | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 0.10 | shoulder_lift | 1.2649 | 1.1496 | 0.5365 | 0.6462 |
| 0.10 | elbow_flex | 0.7415 | 0.7142 | 0.4235 | 0.4730 |
| 0.10 | wrist_flex | 0.2733 | 0.2725 | 0.1725 | 0.1920 |
| 0.10 | wrist_roll | 0.0054 | 0.0050 | 0.0024 | 0.0028 |
| 0.20 | shoulder_pan | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 0.20 | shoulder_lift | 1.6669 | 1.5085 | 0.7100 | 0.8498 |
| 0.20 | elbow_flex | 1.0302 | 0.9917 | 0.5842 | 0.6533 |
| 0.20 | wrist_flex | 0.4297 | 0.4283 | 0.2712 | 0.3018 |
| 0.20 | wrist_roll | 0.0131 | 0.0122 | 0.0058 | 0.0069 |

## 0.10 kg Payload 下的主要结论

- 95 分位力矩最高的关节：`shoulder_lift` = 1.1496 Nm。
- 采样峰值力矩最高的关节：`shoulder_lift` = 1.2649 Nm。
- 采样水平半径范围：0.001 到 0.477 m。
- 采样末端 z 范围：-0.221 到 0.526 m。

## 工程解释

- 静载重力主要影响 pitch 链路关节：`shoulder_lift`、`elbow_flex`、`wrist_flex`。
- `shoulder_pan` 的轴线接近竖直，因此不是重力静载瓶颈；它更影响水平转向动态性能和底座刚度。
- 远端质量会放大上游关节力矩。减轻腕部、夹爪和下臂质量通常可以降低肩部和肘部负担。
- 第一轮 AI 辅助结构改进应优先考虑 upper arm、lower arm、wrist/gripper 和 base fixture，同时保留官方关节轴线和连杆长度。

## 生成文件

- `04_structural_analysis/baseline_static_torque_samples.csv`
- `04_structural_analysis/baseline_static_torque_plot.png`
- `04_structural_analysis/baseline_static_torque_summary.md`
- `04_structural_analysis/baseline_static_torque_summary_zh.md`
