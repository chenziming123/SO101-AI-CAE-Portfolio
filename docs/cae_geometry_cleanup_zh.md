# upper_arm_v3 CAE 几何清理诊断报告

## 这一步在做什么

Step 20 不做 V4，而是先清理和诊断 V3 的 CAE 几何。目标是回答一个明确问题：为什么 full V3 带小装配孔和沉孔时不能稳定生成四面体网格。

## 诊断方法

- 保持 V3 主体、梁、肋板、减重孔和主关节孔不变。
- 将小特征拆成不同组合测试：肩部夹紧孔、肘部安装通孔、肘部沉孔、完整孔系。
- 对每个组合分别测试两种布尔策略：一次性融合 cutter 后切除、逐个 cutter 顺序切除。
- 每种策略再测试是否进行 OpenCASCADE healing。

## 结论

- 7 mm 目标网格下，full V3 以及任何带小装配孔/沉孔的组合都会失败，失败集中表现为 `overlapping facets` 或 `segment and facet intersect`。
- 补充网格尺寸扫描后，full V3 完整孔系在 5.0 mm 目标网格下已经可以稳定生成四面体网格。
- 因此这次问题不是 full V3 几何完全不可用，而是小孔、沉孔、薄壁附近的局部特征尺寸小于 7 mm 粗网格能够可靠表达的尺度。
- CAE 清理策略从“删除小孔跑通”改为“保留完整孔系，并使用更合理的精细网格控制”。

## full V3 失败摘要

- `full_mount_features_fused_noheal`：Invalid boundary mesh (overlapping facets) on surface 141 surface 141
- `full_mount_features_fused_heal`：Invalid boundary mesh (overlapping facets) on surface 2 surface 2
- `full_mount_features_sequential_noheal`：Invalid boundary mesh (overlapping facets) on surface 140 surface 140
- `full_mount_features_sequential_heal`：Invalid boundary mesh (overlapping facets) on surface 72 surface 72

## 补充网格尺寸扫描

在 7 mm 诊断失败后，对 full V3 完整孔系继续做目标网格尺寸扫描。结果显示：

- 5.0 mm 目标网格：`PASS`
- 节点数：`3255`
- 四面体单元数：`11048`
- 输出 STEP：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/v3_full_cae_clean_5mm.step`

这说明 full V3 的 shoulder clamp holes、elbow mount through holes 和 counterbores 可以保留进入 CAE。真正需要清理的是网格策略：小孔和沉孔附近不能继续使用 7 mm 粗网格，否则体网格器会把相邻边界面表达成重叠或相交。

## 完整 full V3 clean FEA 结果

基于 5.0 mm 网格，已经重新对 official 和 full V3 做同口径静力 FEA：

| 版本 | 网格尺寸 mm | 节点 | 四面体 | 质量 g | 最大位移 mm | 最大 von Mises MPa | 安全系数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| official | 5.0 | 3750 | 12410 | 144.07 | 0.024591 | 0.327 | 153.11 |
| full V3 | 5.0 | 3255 | 11048 | 127.49 | 0.071577 | 0.770 | 64.96 |

对比 official，full V3 在完整孔/沉孔保留后仍减重 `11.51%`，但最大位移增加 `191.07%`，最大 von Mises 应力增加 `135.71%`。这说明 V3 的装配几何已经可以进入 CAE，但结构刚度仍弱于 official，后续 V4 应以加强中部梁、上下梁连接和肋板布局为目标。

## 全部尝试结果

| 尝试 | 状态 | 肩部夹紧孔 | 肘部安装孔 | 沉孔 | 布尔策略 | healing | 节点 | 四面体 | 失败原因 |
|---|---|---|---|---|---|---|---:|---:|---|
| core_fused_noheal | PASS | False | False | False | fused | False | 1759 | 5620 |  |
| shoulder_clamp_only_fused_noheal | FAIL | True | False | False | fused | False | 0 | 0 | PLC Error:  A segment and a facet intersect at point |
| elbow_through_only_fused_noheal | FAIL | False | True | False | fused | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| elbow_with_counterbores_fused_noheal | FAIL | False | True | True | fused | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 135 surface 135 |
| all_mount_no_counterbores_fused_noheal | FAIL | True | True | False | fused | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| full_mount_features_fused_noheal | FAIL | True | True | True | fused | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 141 surface 141 |
| core_fused_heal | PASS | False | False | False | fused | True | 1758 | 5614 |  |
| shoulder_clamp_only_fused_heal | FAIL | True | False | False | fused | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 67 surface 67 |
| elbow_through_only_fused_heal | FAIL | False | True | False | fused | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| elbow_with_counterbores_fused_heal | FAIL | False | True | True | fused | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 2 surface 2 |
| all_mount_no_counterbores_fused_heal | FAIL | True | True | False | fused | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| full_mount_features_fused_heal | FAIL | True | True | True | fused | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 2 surface 2 |
| core_sequential_noheal | PASS | False | False | False | sequential | False | 1759 | 5621 |  |
| shoulder_clamp_only_sequential_noheal | FAIL | True | False | False | sequential | False | 0 | 0 | PLC Error:  A segment and a facet intersect at point |
| elbow_through_only_sequential_noheal | FAIL | False | True | False | sequential | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| elbow_with_counterbores_sequential_noheal | FAIL | False | True | True | sequential | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 138 surface 138 |
| all_mount_no_counterbores_sequential_noheal | FAIL | True | True | False | sequential | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| full_mount_features_sequential_noheal | FAIL | True | True | True | sequential | False | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 140 surface 140 |
| core_sequential_heal | PASS | False | False | False | sequential | True | 1758 | 5614 |  |
| shoulder_clamp_only_sequential_heal | FAIL | True | False | False | sequential | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 67 surface 67 |
| elbow_through_only_sequential_heal | FAIL | False | True | False | sequential | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| elbow_with_counterbores_sequential_heal | FAIL | False | True | True | sequential | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 138 surface 138 |
| all_mount_no_counterbores_sequential_heal | FAIL | True | True | False | sequential | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 59 surface 59 |
| full_mount_features_sequential_heal | FAIL | True | True | True | sequential | True | 0 | 0 | Invalid boundary mesh (overlapping facets) on surface 72 surface 72 |

## 输出文件

- 诊断 CSV：`<repo>/07_fea_analysis/upper_arm_static_fea/cae_geometry_cleanup/upper_arm_v3_cae_geometry_cleanup_matrix.csv`
- 诊断目录：`<repo>/07_fea_analysis/upper_arm_static_fea/cae_geometry_cleanup`
- full V3 clean FEA 报告：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_report_zh.md`
- full V3 clean 应力图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_von_mises.png`
- full V3 clean 位移图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_displacement.png`

## 当前边界

- 这一步是 CAE 前处理诊断，不是最终强度结论。
- 5 mm 精细网格已经解决 full V3 的体网格问题，但当前边界条件仍是 screening 工况，不等于真实螺钉接触、轴承接触、预紧和打印各向异性验证。
- 下一步建议在 full V3 clean mesh 上重跑接口边界 FEA，再基于 full V3 的真实孔系结果设计 `upper_arm_v4`。
