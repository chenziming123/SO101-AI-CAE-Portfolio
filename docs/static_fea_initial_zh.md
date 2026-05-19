# SO-101 upper_arm_link Official / V3 静力有限元初步对比

## 这一步在做什么

Step 18 对官方 upper arm 和 AI 改进后的 V3 upper arm 建立相同的线性静力有限元工况，用同一套材料、网格尺寸、固定区域和载荷区域对比结构性能。

## 工况设定

- 几何对象：`official` 使用官方 STEP；`V3` 使用根据 V3 参数在 Gmsh/OpenCASCADE 中重建的 CAE 几何。
- 单位体系：mm、N、MPa。
- 材料：PLA screening，弹性模量 2500 MPa，泊松比 0.35，屈服强度按 50 MPa 做 screening。
- 约束：固定 shoulder 侧沿零件最长方向前 12% 区域的全部平动自由度。
- 载荷：在 elbow 侧沿零件最长方向后 10% 区域施加合力 10.0 N，方向为 Z 负方向。
- 网格：Gmsh 四面体网格，目标尺寸 7.0 mm。

## 核心结果

| 版本 | 几何 | 主轴 | 节点数 | 四面体数 | 网格体积 mm^3 | 估算质量 g | 最大位移 mm | 最大 von Mises MPa | p95 von Mises MPa | 屈服安全系数 |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| official | step | x | 2845 | 8943 | 115465.86 | 143.18 | 0.023253 | 0.324 | 0.134 | 154.21 |
| V3 | gmsh_occ_parametric_no_small_mount_holes | x | 1759 | 5620 | 104561.17 | 129.66 | 0.069782 | 0.588 | 0.356 | 85.02 |

## 对比结论

- V3 相对 official 的网格体积/估算质量变化：-9.44%。
- V3 相对 official 的最大位移变化：+200.10%。
- V3 相对 official 的最大 von Mises 应力变化：+81.37%。
- V3 当前可网格化 CAE 变体为 `gmsh_occ_parametric_no_small_mount_holes`。full V3 和去沉孔版本在体网格阶段暴露出重叠边界问题，说明后续需要继续做 CAD/CAE 几何清理；本轮先保留主体、减重孔和主关节孔，暂时去掉小装配孔来完成第一版强度 screening。
- 这一步把前面的 CAD 改进从“几何与运动学验证”推进到“结构强度 screening 验证”。

## 输出文件

- 指标 JSON：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_metrics.json`
- 指标 CSV：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_metrics.csv`
- official VTK：`<repo>/07_fea_analysis/upper_arm_static_fea/results/official_upper_arm_static_fea.vtu`
- V3 VTK：`<repo>/07_fea_analysis/upper_arm_static_fea/results/v3_upper_arm_static_fea.vtu`
- 应力图：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_von_mises.png`
- 位移图：`<repo>/07_fea_analysis/upper_arm_static_fea/results/upper_arm_static_fea_displacement.png`

## 当前边界

- 这是第一版开源工具链 screening FEA，边界条件采用 x 方向区域选择，还不是基于真实螺钉接触、轴承接触或舵机装配接触的高保真模型。
- STL 到四面体网格会带来离散误差，后续可用 STEP/B-Rep 特征进一步定义孔壁、接触面和载荷面。
- 结果适合用于方案筛选、作品集方法展示和下一轮设计判断，不作为最终生产强度认证。
