# SO-101 upper_arm_link full V3 CAE 几何清理与 FEA 报告

## 这一步在做什么

Step 20 针对 full V3 在 7 mm 网格下出现的 `overlapping facets` 问题进行 CAE 几何清理验证。诊断发现，小装配孔和沉孔不是不能做 FEA，而是 7 mm 目标网格相对 M3 通孔、沉孔和夹紧孔过粗。将目标四面体网格尺寸降到 5.0 mm 后，full V3 带完整小孔和沉孔可以成功网格化。

## 工况设定

- official：官方 `Upper_arm_SO101.step`，重新用 5.0 mm 网格划分。
- V3：Gmsh/OpenCASCADE 参数化重建的 full V3 CAE 几何，包含肩部夹紧孔、肘部安装通孔和沉孔。
- 材料：PLA screening，弹性模量 2500 MPa，泊松比 0.35，屈服强度按 50 MPa 做 screening。
- 约束/载荷：沿零件最长方向，shoulder 侧前 12% 固定，elbow 侧后 10% 施加 10.0 N 向下力。

## 核心结果

| 版本 | 几何 | 网格尺寸 mm | 节点数 | 四面体数 | 质量 g | 最大位移 mm | 最大 von Mises MPa | p95 von Mises MPa | 屈服安全系数 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| official | step_refined_5mm | 5.0 | 3750 | 12410 | 144.07 | 0.024591 | 0.327 | 0.140 | 153.11 |
| full V3 | full_v3_cae_clean_5mm | 5.0 | 3255 | 11048 | 127.49 | 0.071577 | 0.770 | 0.335 | 64.96 |

## 对比结论

- full V3 相对 official 的估算质量变化：-11.51%。
- full V3 相对 official 的最大位移变化：+191.07%。
- full V3 相对 official 的最大 von Mises 应力变化：+135.71%。
- full V3 的 CAE 网格问题已初步解决：关键不是删除小孔，而是对小孔/沉孔区域采用更细的网格。
- 这一步把 Step 18 的“特征抑制版 V3”推进到“完整孔系 full V3 可网格化 FEA”。

## 输出文件

- 清理后的 full V3 STEP：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/v3_full_cae_clean_5mm.step`
- 指标 JSON：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_metrics.json`
- 指标 CSV：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_metrics.csv`
- official VTK：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/official_upper_arm_full_v3_clean_fea.vtu`
- full V3 VTK：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/v3_full_upper_arm_full_v3_clean_fea.vtu`
- 应力图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_von_mises.png`
- 位移图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_displacement.png`

## 当前边界

- 当前解决的是 full V3 在 Gmsh 下的体网格可生成问题，还不是最终高保真接触仿真。
- 约束/载荷仍是区域式 screening；后续还应在 full V3 clean mesh 上继续做接口边界工况。
- 更细网格会增加计算量，但这是小孔和沉孔进入 CAE 的必要代价。
