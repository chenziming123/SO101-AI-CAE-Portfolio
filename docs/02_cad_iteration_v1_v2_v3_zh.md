# SO-101 upper_arm_link Official / V1 / V2 / V3 总对比报告

## 这一步在做什么

Step 17 把 upper arm 这条线从 baseline、工作空间、静载力矩、CAD 重建、STEP 孔位校核、标准件装配检查到 URDF/PyBullet 回归，整理成一版完整作品集总稿。

## 为什么这一步重要

前面的单点结果已经足够证明每一版模型都“能做出来”，但作品集真正需要的是一条完整工程链：

1. 官方 baseline 能加载、能运动、能分析。
2. AI 辅助 CAD 能做出参数化版本。
3. STEP 特征级校核能发现真实装配问题。
4. URDF/PyBullet 回归能证明结构件改动没有破坏运动学。
5. 力矩回归能说明质量变化对机械性能的影响。

## 四版本核心指标

| 版本 | 角色 | 体积 mm^3 | PLA 质量 g | URDF screening 质量 kg | 相对官方体积变化 | 孔位/接口校核 | smoke 最大末端差 m | shoulder_lift p95 Nm | 相对官方变化 |
|---|---|---:|---:|---:|---:|---|---:|---:|---:|
| official | baseline | 117328.19 | 145.49 | 0.103000 | 0.00% | 官方基准 | 0.000000e+00 | 1.1422 | 0.00% |
| V1 | 轻量化概念版 | 93885.71 | 116.42 | 0.082420 | -19.98% | PASS / 关键装配未完全复现 | 0.000000e+00 | 1.1254 | -1.47% |
| V2 | 装配接口增强版 | 98626.10 | 122.30 | 0.086582 | -15.94% | PASS | 0.000000e+00 | 1.1288 | -1.17% |
| V3 | 孔系余量修正版 | 101891.50 | 126.35 | 0.089448 | -13.16% | PASS | 0.000000e+00 | 1.1313 | -0.95% |

## 设计迭代脉络

### Official

- 官方 SO-101 upper arm 作为真实开源 baseline，不改动。
- 作用是给后续所有改进提供统一参照系。

### V1

- 先做轻量化概念版，保留主关节中心距和接口轴线。
- 它证明 AI 辅助参数化建模可以把体积和惯量降下来。
- 但它还没有把完整装配接口补齐。

### V2

- 在 V1 上补齐肘部 2x2 安装孔、沉孔和肩部夹紧/定位孔。
- 它把 upper arm 从“概念件”推进到“装配表达更完整的结构件”。
- 但标准件装配检查仍暴露出肘部孔系余量偏紧。

### V3

- 针对 Step 14 的警告，移动肘部孔系并放宽孔径、沉孔和局部 pad。
- 结果是 V3 在 CAD 装配检查和 PyBullet smoke test 上都通过，且 100g payload 下 `shoulder_lift` p95 仍低于官方 baseline。

## 我学到的工程方法

- 不要只看 mesh 外观，要回到 STEP B-Rep 特征去验证孔位和轴线。
- 不要只做 CAD，要把 CAD 接回 URDF/PyBullet 验证运动链。
- 不要只看一版结果，要做同口径的 V1/V2/V3 对比。
- 不要默认 AI 生成结果是最终答案，要用工程脚本筛查和修正。

## 当前边界

- 现在 upper arm 这条线已经闭环，但还没有做真实 FEA、打印方向、螺钉预紧和实物装机。
- 目前惯量仍是 screening 值，不是最终材料标定。
- 还可以继续推进 lower arm、wrist 或完整整机对比。

## 模型图对比

- 整体等轴视图：`<repo>/06_portfolio_summary/model_visuals/upper_arm_official_v1_v2_v3_isometric.png`
- V1/V2/V3 三视图：`<repo>/06_portfolio_summary/model_visuals/upper_arm_v1_v2_v3_three_views.png`
- V2/V3 肘部孔系局部差异图：`<repo>/06_portfolio_summary/model_visuals/upper_arm_v2_v3_elbow_hole_zoom.png`
- 图片说明页：`<repo>/06_portfolio_summary/model_visuals/upper_arm_model_visual_comparison_zh.md`

这些图的作用是把“参数、孔位和力矩结果”转成直观可看的模型差异，方便后续面试或作品集展示时说明：V1 先做轻量化，V2 补装配接口，V3 针对肘部孔系装配余量做修正。

## 有限元初步验证

已补充 Official / V3 upper arm 第一版静力有限元 screening：

- 报告：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_report_zh.md`
- 应力图：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_von_mises.png`
- 位移图：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_displacement.png`

当前 FEA 结论是：V3 的可网格化 CAE 变体相对 official 仍然更轻，但在 10 N 向下载荷 screening 工况下，最大位移和最大 von Mises 应力高于 official；不过两者按 PLA 屈服强度估算的安全系数都很高。这个结果适合用于说明“轻量化需要通过 FEA 复核刚度和应力”，也为后续 V4 加强筋或局部加厚提供依据。

已进一步补充接口边界 FEA：

- 报告：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_report_zh.md`
- 应力图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_von_mises.png`
- 位移图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_displacement.png`

接口边界工况下，V3 仍然更轻，最大位移仍高于 official，但最大 von Mises 应力低于 official。这个组合说明 V3 当前不是强度直接失效问题，而是轻量化后刚度不足更明显，后续 V4 应优先加强中部梁和肋板布局。

已补充 full V3 CAE 几何清理与完整孔系 FEA：

- CAE 清理诊断：`<repo>/07_fea_analysis/upper_arm_static_fea/cae_geometry_cleanup/upper_arm_v3_cae_geometry_cleanup_report_zh.md`
- full V3 clean FEA 报告：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_report_zh.md`
- full V3 clean 应力图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_von_mises.png`
- full V3 clean 位移图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_displacement.png`

这一步解决了前面 full V3 无法直接体网格化的问题：7 mm 粗网格在小孔和沉孔附近会产生重叠边界面，改用 5 mm 精细网格后，保留 shoulder clamp holes、elbow mount through holes 和 counterbores 的 full V3 可以正常生成四面体网格。完整孔系 FEA 下，full V3 相对 official 质量降低 `11.51%`，但最大位移增加 `191.07%`，最大 von Mises 应力增加 `135.71%`，说明 V3 已经可以作为真实 CAE 输入，但下一版仍应优先补刚度。

## 下一步

Step 21 建议先在 full V3 clean mesh 上重跑接口边界 FEA，再基于完整孔系的应力/位移结果设计 `upper_arm_v4`。这样 V4 的加强位置不是凭感觉决定，而是由完整孔系 CAE 结果反推。

## 输出文件

- 关键指标 CSV：`<repo>/06_portfolio_summary/upper_arm_v1_v2_v3_key_metrics.csv`
- 汇总图：`<repo>/06_portfolio_summary/upper_arm_v1_v2_v3_summary_plot.png`
- 模型图对比说明页：`<repo>/06_portfolio_summary/model_visuals/upper_arm_model_visual_comparison_zh.md`
- FEA 报告：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_report_zh.md`
- 接口边界 FEA 报告：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_report_zh.md`
- full V3 clean FEA 报告：`<repo>/07_fea_analysis/upper_arm_static_fea/results_full_v3_clean/upper_arm_full_v3_clean_fea_report_zh.md`
- 总对比报告：`<repo>/06_portfolio_summary/upper_arm_v1_v2_v3_total_comparison_zh.md`
- 面试讲述稿：`<repo>/06_portfolio_summary/upper_arm_v1_v2_v3_interview_talking_points_zh.md`
