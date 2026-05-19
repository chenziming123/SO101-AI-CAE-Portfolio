# upper_arm_v3 惯量更新与静载力矩对比报告

## 这一步在做什么

将 `upper_arm_v3` 的体积变化转化为 URDF inertial screening 参数，并与官方 SO-101 baseline 做静载关节力矩对比。

## 方法

- 保留官方 URDF 作为 baseline，不覆盖 `00_source_snapshot/`。
- 使用 V3 STL 与官方 upper arm STL 的体积比例，按比例缩放官方 `upper_arm_link` 的质量和惯量张量。
- 生成 `so101_upper_arm_v3_inertial_scaled.urdf`。
- 使用同一批随机关节姿态，对官方版和 V3 inertial 版计算静载关节力矩。
- 该方法是工程筛查，不是最终真实惯量标定。

## 质量与惯量更新

- 官方 upper arm STL 体积：117328.19 mm^3
- V1 upper arm STL 体积：93885.71 mm^3
- V2 upper arm STL 体积：98626.10 mm^3
- V3 upper arm STL 体积：101891.50 mm^3
- V1 / 官方体积比例：0.8002
- V2 / 官方体积比例：0.8406
- V3 / 官方体积比例：0.8684
- V2 / V1 体积比例：1.0505
- V3 / V2 体积比例：1.0331
- 官方 URDF `upper_arm_link` 质量：0.103000 kg
- V3 screening 质量：0.089448 kg
- V3 相对官方质量变化：-0.013552 kg (-13.16%)

说明：官方 URDF link 质量和 STL 体积估算质量并不完全一致。因此本步骤采用“URDF 质量按 STL 体积比例缩放”的筛查方式，目的是观察趋势，不作为最终物理参数。

## 0.10 kg Payload 下的 p95 静载力矩对比

| 关节 | 官方 p95 Nm | V3 p95 Nm | 变化 Nm | 变化比例 |
|---|---:|---:|---:|---:|
| shoulder_pan | 0.0000 | 0.0000 | -0.0000 | -0.98% |
| shoulder_lift | 1.1422 | 1.1313 | -0.0109 | -0.95% |
| elbow_flex | 0.7130 | 0.7130 | 0.0000 | 0.00% |
| wrist_flex | 0.2724 | 0.2724 | 0.0000 | 0.00% |
| wrist_roll | 0.0050 | 0.0050 | 0.0000 | 0.00% |

## Official / V1 / V2 / V3 四版本对比

| 关节 | 官方 p95 Nm | V1 p95 Nm | V2 p95 Nm | V3 p95 Nm | V3 相对 V2 变化 Nm |
|---|---:|---:|---:|---:|---:|
| shoulder_pan | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| shoulder_lift | 1.1422 | 1.1254 | 1.1288 | 1.1313 | 0.0025 |
| elbow_flex | 0.7130 | 0.7130 | 0.7130 | 0.7130 | 0.0000 |
| wrist_flex | 0.2724 | 0.2724 | 0.2724 | 0.2724 | -0.0000 |
| wrist_roll | 0.0050 | 0.0050 | 0.0050 | 0.0050 | -0.0000 |

## 主要结论

- `shoulder_lift` p95 力矩：1.1422 -> 1.1313 Nm，变化 -0.0109 Nm (-0.95%)。
- `elbow_flex` p95 力矩：0.7130 -> 0.7130 Nm，变化 0.0000 Nm (0.00%)。
- V3 比 V2 略重，因为它继续保留并加厚了肘部孔系周边材料；这一点与 Step 15 的装配余量修复目标一致。
- 结果仍符合机械直觉：upper arm 的质量变化主要影响 `shoulder_lift`，对下游 `elbow_flex` 和 `wrist_flex` 影响很小。

## 当前限制

- 当前 inertia 是按体积比例缩放的筛查值，不是从 CAD 质量属性直接导出的最终惯量。
- Official / V1 / V2 / V3 四版本对比使用同一批随机关节姿态重新计算，避免不同随机采样导致的 p95 细微偏差。
- 尚未替换 collision mesh，也尚未引入舵机、轴承、螺丝等标准件做完整装配干涉检查。
- 下一步建议整理 official/V1/V2/V3 总对比报告，或者继续向 lower arm / wrist 结构推进。

## 生成文件

- `05_improved_design/upper_arm_v3/inertial_compare/so101_upper_arm_v3_inertial_scaled.urdf`
- `05_improved_design/upper_arm_v3/inertial_compare/upper_arm_v3_torque_comparison.csv`
- `05_improved_design/upper_arm_v3/inertial_compare/upper_arm_v3_torque_comparison_plot.png`
- `05_improved_design/upper_arm_v3/inertial_compare/upper_arm_v3_torque_comparison_report_zh.md`
- 静态姿态数量（含 zero pose）：10001
