# upper_arm_link 模型图对比说明

## 目的

把 official、V1、V2、V3 的模型差异可视化，方便直接查看结构变化，而不是只看体积和力矩数字。

## 图片文件

- 四版本整体等轴视图：`upper_arm_official_v1_v2_v3_isometric.png`
- V1/V2/V3 三视图演化：`upper_arm_v1_v2_v3_three_views.png`
- V2/V3 肘部孔系局部对比：`upper_arm_v2_v3_elbow_hole_zoom.png`

## 主要观察点

- V1：主要体现轻量化骨架，主关节中心距和主孔轴线保留，但装配孔系不足。
- V2：增加肘部 2x2 安装孔、沉孔和肩部夹紧/定位孔，装配表达更完整。
- V3：在 V2 基础上移动并放宽肘部孔系，局部 pad 加厚，用少量质量换取标准件装配余量。
- official：作为开源 baseline，不要求和 AI 重建件外形完全一致，主要用于对比体积、质量和接口约束。

## 当前限制

- 这些图是 STL/参数的可视化渲染，不是生产图纸。
- official STL 与 V1/V2/V3 的本地坐标系不完全一致，因此 official 主要作为外观和体积参照，不建议直接和 V1/V2/V3 叠加判断孔位。

![四版本整体等轴视图](upper_arm_official_v1_v2_v3_isometric.png)

![V1/V2/V3 三视图演化](upper_arm_v1_v2_v3_three_views.png)

![V2/V3 肘部孔系局部对比](upper_arm_v2_v3_elbow_hole_zoom.png)
