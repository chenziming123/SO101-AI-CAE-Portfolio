# upper_arm_link V2 几何验证报告

## 目的

检查 `upper_arm_v2` 是否成功导出为 STEP/STL，并与官方 upper arm、V1 做基础网格几何对比。

V2 的设计重点是补充装配孔系，因此质量可能相对 V1 略有变化；这一步只做几何与体积筛查，孔位由单独的 mounting check 报告判断。

## 文件

- 官方 STL：`<repo>/00_source_snapshot/STL_SO101/Individual/Upper_arm_SO101.stl`
- V1 STL：`<repo>/05_improved_design/upper_arm_v1/upper_arm_v1_ai_rebuild.stl`
- V2 STL：`<repo>/05_improved_design/upper_arm_v2/upper_arm_v2_ai_rebuild.stl`
- V2 STEP：`<repo>/05_improved_design/upper_arm_v2/upper_arm_v2_ai_rebuild.step`

## 基础几何统计

| 模型 | 顶点数 | 面数 | watertight | 包围盒尺寸 xyz | 体积 | PLA 估算质量 |
|---|---:|---:|---|---:|---:|---:|
| official_upper_arm | 3961 | 8038 | True | 142.15, 24.50, 67.30 mm | 117328.19 mm^3 | 145.49 g |
| upper_arm_v1 | 2192 | 4412 | True | 150.50, 38.00, 39.30 mm | 93885.71 mm^3 | 116.42 g |
| upper_arm_v2 | 5692 | 11436 | True | 150.50, 36.00, 39.30 mm | 98626.10 mm^3 | 122.30 g |

## 对比结论

- V2 相对官方 STL 体积变化：-18702.09 mm^3，比例 -15.94%。
- V2 相对 V1 体积变化：4740.39 mm^3，比例 5.05%。
- 若 V2 比 V1 略重，这是为了加入舵机安装孔周边加厚 pad 与定位孔结构，属于装配可靠性换取质量的取舍。
- 是否真正补上目标孔系，需要看 `mounting_check/upper_arm_v2_mounting_check_report_zh.md`。
