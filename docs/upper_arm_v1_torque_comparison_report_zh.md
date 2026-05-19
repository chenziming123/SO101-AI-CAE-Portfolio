# upper_arm_v1 惯量更新与静载力矩对比报告

## 目的

验证 `upper_arm_v1` 减重后，在 URDF 惯量层面是否会降低 SO-101 的静载关节力矩。

## 方法

- 保留官方 URDF 作为 baseline，不覆盖 `00_source_snapshot/`。
- 使用 Step 8 的 STL 体积对比得到 V1 / 官方 upper arm 的体积比例。
- 将官方 `upper_arm_link` 的质量和惯量张量按同一比例缩放，生成 inertial screening 版 URDF。
- 使用同一批随机关节姿态，对官方版和 V1 inertial 版分别计算静载关节力矩。
- 该方法是工程筛查，不是最终真实惯量标定。

## 质量与惯量更新

- 官方 upper arm STL 体积：117328.19 mm^3
- V1 upper arm STL 体积：93885.71 mm^3
- V1 / 官方体积比例：0.8002
- 官方 URDF `upper_arm_link` 质量：0.103000 kg
- V1 screening 质量：0.082420 kg
- 质量变化：-0.020580 kg (-19.98%)

说明：官方 URDF link 质量和 STL 体积估算质量并不完全一致。因此本步骤采用“URDF 质量按 STL 体积比例缩放”的筛查方式，目的是观察趋势，不作为最终物理参数。

## 0.10 kg Payload 下的 p95 静载力矩对比

| 关节 | 官方 p95 Nm | V1 p95 Nm | 变化 Nm | 变化比例 |
|---|---:|---:|---:|---:|
| shoulder_pan | 0.0000 | 0.0000 | -0.0000 | -1.68% |
| shoulder_lift | 1.1431 | 1.1268 | -0.0163 | -1.43% |
| elbow_flex | 0.7126 | 0.7126 | 0.0000 | 0.00% |
| wrist_flex | 0.2724 | 0.2724 | 0.0000 | 0.00% |
| wrist_roll | 0.0050 | 0.0050 | -0.0000 | -0.00% |

说明：`shoulder_pan` 的静载重力力矩接近 0，因此它的百分比变化没有实际工程意义，主要看绝对值即可。

## 主要结论

- `shoulder_lift` p95 力矩：1.1431 -> 1.1268 Nm，变化 -0.0163 Nm (-1.43%)。
- `elbow_flex` p95 力矩：0.7126 -> 0.7126 Nm，变化 0.0000 Nm (0.00%)。
- 结果符合机械直觉：减轻 upper arm 主要降低 shoulder_lift 负担，对 elbow_flex 的影响很小，因为 upper arm 位于 elbow 上游。

## 当前限制

- 仍未进行真实 CAD 装配孔位校核。
- 仍未进行真实材料、打印方向、局部螺丝柱的 FEA。
- 当前 inertia 是按比例缩放的筛查值，不是从完整 CAD 质量属性直接导出的最终惯量。
- 下一步应把这个结论整理成 before/after 对比表，并决定是否继续优化 lower_arm 或 gripper。

## 生成文件

- `05_improved_design/upper_arm_v1/inertial_compare/so101_upper_arm_v1_inertial_scaled.urdf`
- `05_improved_design/upper_arm_v1/inertial_compare/upper_arm_v1_torque_comparison.csv`
- `05_improved_design/upper_arm_v1/inertial_compare/upper_arm_v1_torque_comparison_plot.png`
- `05_improved_design/upper_arm_v1/inertial_compare/upper_arm_v1_torque_comparison_report_zh.md`
- 静态姿态数量（含 zero pose）：10001
