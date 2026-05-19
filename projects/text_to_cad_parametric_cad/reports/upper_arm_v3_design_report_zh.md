# upper_arm_link V3 AI 辅助结构迭代设计报告

## 这一步在做什么

Step 15 在 V2 标准件装配检查的基础上，针对肘部孔系余量不足的问题生成 `upper_arm_v3`。

V3 不是推翻 V2，而是做一次明确的局部工程修正：保持 shoulder/elbow 主关节轴线、116.000 mm 中心距和整体运动链不变，只优化肘部 2x2 安装孔、沉孔和局部 pad。

## 为什么要做 V3

Step 14 中 V2 的总体状态是 `PASS_WITH_WARNINGS`，没有 FAIL，但有 3 个关键 WARN：

- 肘部 M3 通孔径向余量只有 0.100 mm，真实打印后偏紧。
- 肘部 M3 螺钉头沉孔径向余量只有 0.100 mm，螺钉头适配风险偏高。
- 最近的肘部 M3 孔与 elbow 主孔之间材料余量约 0.250 mm，局部材料桥偏薄。

这些问题说明 V2 已经能表达装配孔位，但还不适合作为更可信的可打印结构版本。V3 的目标就是把这些警告转化为设计修改和再次验证。

## V3 设计修改

| 项目 | V2 | V3 | 作用 |
|---|---:|---:|---|
| 肘部 2x2 孔系中心 x | 105.0 mm | 103.0 mm | 让最近的 M3 孔远离 elbow 主孔 |
| 肘部 M3 通孔半径 | 1.60 mm | 1.75 mm | 增大 M3 螺钉装配余量 |
| 肘部 M3 沉孔半径 | 2.70 mm | 2.95 mm | 增大螺钉头装配余量 |
| 肘部 M3 沉孔深度 | 2.20 mm | 2.40 mm | 增大螺钉头高度余量 |
| 肘部 pad 尺寸 | 28 x 32 x 35 mm | 30 x 34 x 36 mm | 用局部加厚抵消孔径放大带来的材料削弱 |

## 生成文件

- 参数化 CAD 源码：`05_improved_design/upper_arm_v3/upper_arm_v3_cad.py`
- V3 STEP：`05_improved_design/upper_arm_v3/upper_arm_v3_ai_rebuild.step`
- V3 STL：`05_improved_design/upper_arm_v3/upper_arm_v3_ai_rebuild.stl`
- 参数 JSON：`05_improved_design/upper_arm_v3/upper_arm_v3_parameters.json`
- 几何验证报告：`05_improved_design/upper_arm_v3/upper_arm_v3_validation_zh.md`
- CAD 孔位复核报告：`05_improved_design/upper_arm_v3/mounting_check/upper_arm_v3_mounting_check_report_zh.md`
- 标准件装配检查报告：`05_improved_design/upper_arm_v3/assembly_check/upper_arm_v3_standard_part_check_report_zh.md`
- 简化标准件装配 STEP：`05_improved_design/upper_arm_v3/assembly_check/upper_arm_v3_simplified_standard_parts_assembly.step`
- 简化标准件装配 STL：`05_improved_design/upper_arm_v3/assembly_check/upper_arm_v3_simplified_standard_parts_assembly.stl`

## 几何与质量对比

| 模型 | STL 体积 | PLA 粗估质量 | watertight | 说明 |
|---|---:|---:|---|---|
| official upper arm | 117328.19 mm^3 | 145.49 g | true | 官方结构 |
| upper_arm_v1 | 93885.71 mm^3 | 116.42 g | true | 轻量化概念版 |
| upper_arm_v2 | 98626.10 mm^3 | 122.30 g | true | 增加装配孔位版 |
| upper_arm_v3 | 101891.50 mm^3 | 126.35 g | true | 修正肘部孔系余量版 |

关键解释：

- V3 相对官方 upper arm 体积仍减少约 13.15%，PLA 粗估质量少约 19.14 g。
- V3 相对 V2 体积增加约 3.31%，PLA 粗估质量增加约 4.05 g。
- 这部分增加的质量来自肘部 pad 局部加厚和孔系余量修正，属于用小幅质量代价换取装配可靠性。

## CAD 孔位复核结果

V3 的 CAD 圆柱特征复核总体为 `PASS`。

关键通过项：

- 肩部主孔半径：4.200 mm，PASS。
- 肘部主孔半径：4.200 mm，PASS。
- 肩部到肘部主孔中心距：116.000 mm，PASS。
- 主孔轴线方向：Y 轴，误差 0.000 deg，PASS。
- 肘部 2x2 安装孔：4 个 Z 轴通孔全部检测到，孔半径 1.750 mm，PASS。
- 肩部侧向夹紧/定位孔：2 个 Y 轴通孔全部检测到，孔半径 2.000 mm，PASS。

这说明 V3 没有破坏 V2 已建立的主关节接口和二级装配孔系。

## 标准件装配检查结果

V3 的简化标准件装配检查总体为 `PASS`，FAIL 数量 0，WARN 数量 0。

| 检查项 | 结果 |
|---|---|
| 主关节 D8 轴/轴套径向余量 | 0.200 mm，PASS |
| 肘部 M3 通孔径向余量 | 0.250 mm，PASS |
| 肘部 M3 螺钉头沉孔径向余量 | 0.350 mm，PASS |
| 肘部 M3 螺钉头沉孔深度余量 | 0.400 mm，PASS |
| 肘部安装孔到 pad 边缘最小材料 | 通孔 8.300 mm，沉孔 7.100 mm，PASS |
| 肘部 2x2 孔间最小材料 | 通孔 6.400 mm，沉孔 4.000 mm，PASS |
| 肘部安装孔与 elbow 主孔最小孔间材料 | 通孔外缘 2.100 mm，沉孔外缘 0.900 mm，PASS |
| 肩部 M3 夹紧/定位孔径向余量 | 0.500 mm，PASS |
| 肩部夹紧孔到 pad 边缘最小材料 | 2.500 mm，PASS |
| 肩部夹紧孔与 shoulder 主孔最小孔间材料 | 7.000 mm，PASS |
| 简化舵机/连接板 footprint | 最小边缘余量 4.000 mm，PASS |

## 工程意义

V3 的价值不在于“又生成了一个模型”，而在于形成了一个完整的工程迭代闭环：

1. V2 做出装配孔位。
2. Step 14 用标准件装配检查发现具体风险。
3. V3 针对风险修改参数。
4. 再用 CAD 孔位复核和标准件装配检查证明问题被修复。

这比单纯展示一个 AI 生成模型更有说服力，因为它体现了机械结构设计中“发现问题、定位原因、修改结构、重新验证”的过程。

## 当前边界

- V3 仍然是基于简化标准件的设计阶段检查，不等于真实生产装配认证。
- 尚未做真实 FEA、打印方向、层间强度、螺钉预紧力和疲劳验证。
- 尚未将 V3 接回 URDF/PyBullet 做惯量更新和静载力矩回归。

## 下一步

Step 17 可以继续做两件事之一：

- 整理 upper arm 这一条线的作品集总稿，把 baseline、V1、V2、V3、CAD、装配检查、URDF smoke 和惯量回归串成一页能讲清的流程。
- 继续推进下一个结构件，优先选择 lower arm，用同样方法复用 baseline -> CAD -> URDF -> 力矩的闭环。
