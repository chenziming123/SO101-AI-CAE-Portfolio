# upper_arm_link V1 几何验证报告

## 目的

检查 AI 辅助生成的 `upper_arm_v1` 是否成功导出为 STEP/STL，并与官方 upper arm STL 做基础几何对比。

## 文件

- 官方 STL：`<repo>/00_source_snapshot/STL_SO101/Individual/Upper_arm_SO101.stl`
- 改进版 STL：`<repo>/05_improved_design/upper_arm_v1/upper_arm_v1_ai_rebuild.stl`
- 改进版 STEP：`<repo>/05_improved_design/upper_arm_v1/upper_arm_v1_ai_rebuild.step`

## 基础几何统计

| 模型 | 顶点数 | 面数 | watertight | 包围盒尺寸 xyz | 体积 | PLA 估算质量 |
|---|---:|---:|---|---:|---:|---:|
| official_upper_arm | 3961 | 8038 | True | 142.15, 24.50, 67.30 mm | 117328.19 mm^3 | 145.49 g |
| improved_upper_arm_v1 | 2192 | 4412 | True | 150.50, 38.00, 39.30 mm | 93885.71 mm^3 | 116.42 g |

## 对比结论

- 改进版 X 向总包围盒长度约 150.50 mm；设计中保留的 shoulder 到 elbow 轴线间距为 116.0 mm。
- 与官方 STL 相比，改进版体积变化约 -23442.49 mm^3，比例约 -20.0%。
- V1 是参数化概念重建件，重点是建立可编辑 CAD 源文件和结构改进方向，不直接声称已经满足装机打印要求。
- 下一步需要可视化检查外观，并准备把改进 mesh 接回 URDF/PyBullet 做加载测试。
