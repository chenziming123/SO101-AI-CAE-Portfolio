# upper_arm_link V3 几何验证报告

## 目的

检查 `upper_arm_v3` 是否成功导出为 STEP/STL，并与官方 upper arm、V1、V2 做基础网格几何对比。

V3 的目标不是重新设计整根 upper arm，而是针对 Step 14 暴露的肘部孔系余量问题做局部修复。

## 文件

- 官方 STL：`<repo>/00_source_snapshot/STL_SO101/Individual/Upper_arm_SO101.stl`
- V1 STL：`<repo>/05_improved_design/upper_arm_v1/upper_arm_v1_ai_rebuild.stl`
- V2 STL：`<repo>/05_improved_design/upper_arm_v2/upper_arm_v2_ai_rebuild.stl`
- V3 STL：`<repo>/05_improved_design/upper_arm_v3/upper_arm_v3_ai_rebuild.stl`
- V3 STEP：`<repo>/05_improved_design/upper_arm_v3/upper_arm_v3_ai_rebuild.step`

## 基础几何统计

| 模型 | 顶点数 | 面数 | watertight | 包围盒尺寸 xyz | 体积 | PLA 估算质量 |
|---|---:|---:|---|---:|---:|---:|
| official_upper_arm | 3961 | 8038 | True | 142.15, 24.50, 67.30 mm | 117328.19 mm^3 | 145.49 g |
| upper_arm_v1 | 2192 | 4412 | True | 150.50, 38.00, 39.30 mm | 93885.71 mm^3 | 116.42 g |
| upper_arm_v2 | 5692 | 11436 | True | 150.50, 36.00, 39.30 mm | 98626.10 mm^3 | 122.30 g |
| upper_arm_v3 | 5518 | 11088 | True | 150.50, 36.00, 39.30 mm | 101891.50 mm^3 | 126.35 g |

## 对比结论

- V3 相对官方 STL 体积变化：-15436.69 mm^3，比例 -13.16%。
- V3 相对 V2 体积变化：3265.40 mm^3，比例 3.31%。
- V3 应结合 `assembly_check/upper_arm_v3_standard_part_check_report_zh.md` 判断是否真正修复了 V2 的装配余量风险。
