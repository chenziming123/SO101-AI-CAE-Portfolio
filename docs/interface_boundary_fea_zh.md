# SO-101 upper_arm_link Official / V3 接口边界静力有限元对比

## 这一步在做什么

Step 19 在 Step 18 已经跑通的四面体网格基础上，只升级边界条件：从“固定/加载一大段长度区域”改为“固定/加载靠近 shoulder/elbow 端部接口的窄区域节点”。这样可以更接近机械接口受力，而不改变材料、网格和总载荷。

## 工况设定

- 几何和网格：复用 Step 18 的 official STEP 网格和 V3 可网格化 CAE 变体网格。
- 单位体系：mm、N、MPa。
- 材料：PLA screening，弹性模量 2500 MPa，泊松比 0.35，屈服强度按 50 MPa 做 screening。
- 约束：固定 shoulder 侧端部接口窄带节点。
- 载荷：在 elbow 侧端部接口窄带节点施加合力 10.0 N，方向为 Z 负方向。
- 边界节点选择：从 2.0 mm 端部窄带开始，如果节点太少，自动放宽到最多 10.0 mm。

## 核心结果

| 版本 | 主轴 | 固定节点 | 加载节点 | 固定带宽 mm | 加载带宽 mm | 质量 g | 最大位移 mm | 最大 von Mises MPa | p95 von Mises MPa | 屈服安全系数 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| official | x | 102 | 27 | 2.0 | 3.0 | 143.18 | 0.033668 | 0.726 | 0.187 | 68.90 |
| V3 | x | 56 | 25 | 2.0 | 4.0 | 129.66 | 0.086954 | 0.636 | 0.381 | 78.60 |

## 对比结论

- V3 相对 official 的估算质量变化：-9.44%。
- V3 相对 official 的最大位移变化：+158.27%。
- V3 相对 official 的最大 von Mises 应力变化：-12.34%。
- 与 Step 18 相比，Step 19 的意义不是追求更好看的数值，而是让边界条件更像真实接口载荷；这会让应力集中更明显，也更适合指导下一版结构加强。

## 输出文件

- 指标 JSON：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_metrics.json`
- 指标 CSV：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_metrics.csv`
- official VTK：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/official_upper_arm_interface_fea.vtu`
- V3 VTK：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/v3_upper_arm_interface_fea.vtu`
- 应力图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_von_mises.png`
- 位移图：`<repo>/07_fea_analysis/upper_arm_static_fea/results_interface_boundary/upper_arm_interface_fea_displacement.png`

## 当前边界

- 当前仍是端部接口窄带节点选择，还不是从 STEP B-Rep 精确识别孔壁面、螺钉接触面或轴承接触面。
- V3 仍采用 Step 18 跑通的 `no_small_mount_holes` CAE 变体；full V3 的小装配孔和沉孔仍需要继续清理。
- 这一步适合用于“FEA 边界条件升级”和“V4 加强设计依据”，不作为最终生产强度认证。
