# SO-101 upper_arm_link CalculiX 求解链 smoke test 报告

## 这一步在做什么

Step 23 使用 Step 20 已经跑通的 official / full V3 clean 5 mm 四面体网格，生成 CalculiX `.inp` 文件，并调用 `ccx` 运行线性静力求解。

这一步的目标不是替代最终高保真 FEA，而是先证明开源 CAE 求解链已经打通：

```text
Gmsh 四面体网格 -> CalculiX .inp -> ccx 求解 -> .dat/.frd 结果文件
```

## 工具链

- CalculiX：`ccx 2.23`
- 输入网格：Step 20 full V3 clean 5 mm `.msh`
- 单元：一阶四面体 `C3D4`
- 材料：PLA screening，E=2500 MPa，nu=0.35
- 载荷：沿零件最长方向，shoulder 侧固定，elbow 侧施加总计 10 N 向下力

## 核心结果

| 版本 | 状态 | 节点 | 四面体 | 固定节点 | 加载节点 | 最大加载位移 mm | 最大 von Mises MPa | 安全系数 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| official | PASS | 3750 | 12410 | 679 | 569 | 0.024591 | 0.327 | 153.11 |
| v3_full | PASS | 3255 | 11048 | 491 | 213 | 0.071577 | 0.770 | 64.96 |

## 输出文件大小检查

| 版本 | ccx 返回码 | DAT bytes | FRD bytes |
|---|---:|---:|---:|
| official | 0 | 2487588 | 1702805 |
| v3_full | 0 | 2199044 | 1498439 |

## 输出文件

- 指标 CSV：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/upper_arm_calculix_smoke_metrics.csv`
- 指标 JSON：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/upper_arm_calculix_smoke_metrics.json`

### official

- INP：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/official/official_upper_arm_ccx_smoke.inp`
- DAT：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/official/official_upper_arm_ccx_smoke.dat`
- FRD：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/official/official_upper_arm_ccx_smoke.frd`
- LOG：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/official/official_upper_arm_ccx_smoke_ccx.log`

### full V3

- INP：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/v3_full/v3_full_upper_arm_ccx_smoke.inp`
- DAT：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/v3_full/v3_full_upper_arm_ccx_smoke.dat`
- FRD：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/v3_full/v3_full_upper_arm_ccx_smoke.frd`
- LOG：`<repo>/07_fea_analysis/upper_arm_static_fea/results_calculix_smoke/v3_full/v3_full_upper_arm_ccx_smoke_ccx.log`

## 当前结论

- 如果两个 case 都为 `PASS`，说明 CalculiX 求解器链路已经可用于本项目。
- 下一步可以开始把 Python screening solver 的结果与 `ccx` 的位移/应力结果做同口径对比。
- 之后再考虑更复杂的边界条件、孔壁面载荷、螺钉接触、打印方向和材料各向异性。

## 当前边界

- 这是 smoke test，只验证求解链路，不作为最终强度结论。
- 当前仍使用节点集合近似固定和加载区域，没有做真实接触。
- 当前使用 C3D4 一阶四面体，后续可以评估 C3D10 或网格收敛。
