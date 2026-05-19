# upper_arm_v3 标准件装配与简化干涉检查报告

## 这一步在做什么

Step 15 针对 Step 14 在 V2 中发现的肘部孔系余量问题，生成 V3 的简化标准件装配检查。

本步骤用简化 D8 主轴、M3 螺钉、M3 螺钉头和简化舵机连接板，检查 V3 的孔径、沉孔、孔间材料和 pad 边缘材料是否可接受。

## V3 相比 V2 的修改

- 肘部 2x2 安装孔中心从 x=105.0 mm 移到 x=103.0 mm，让孔系远离 elbow 主孔。
- 肘部 M3 通孔半径从 1.60 mm 放宽到 1.75 mm，提高打印后螺钉装入成功率。
- 肘部 M3 沉孔半径从 2.70 mm 放宽到 2.95 mm，提高螺钉头装配余量。
- 肘部 pad 从 28 x 32 x 35 mm 增加到 30 x 34 x 36 mm，用局部加厚抵消放宽孔径带来的材料削弱。

## 标准件简化假设

- 主关节轴/轴套：半径 4.00 mm，对应约 D8 标准件包络。
- 肘部安装螺钉：M3 简化螺钉，杆部半径 1.50 mm，头部半径 2.60 mm，高度 2.00 mm。
- 肩部夹紧/定位螺钉：M3 简化螺钉，杆部半径 1.50 mm，外侧头部半径 3.00 mm。
- 肘部简化舵机/连接板 footprint：22.0 x 22.0 mm。

## 总体结论

- 装配检查总体状态：`PASS`。
- FAIL 项数量：0。
- WARN 项数量：0。
- Step 14 中 V2 的 3 个装配余量 WARN 项在 V3 中已消除。

## 检查明细

| 检查项 | 要求 | 实测/计算 | 状态 | 说明 |
|---|---|---|---|---|
| 主关节 D8 轴/轴套径向余量 | 孔半径 4.20 mm，应大于标准件半径 4.00 mm | 0.200 mm | PASS | 用于判断 shoulder/elbow 主轴或轴套是否能装入。 |
| 肘部 M3 螺钉通孔径向余量 | 孔半径 1.75 mm，应大于 M3 螺钉半径 1.50 mm | 0.250 mm | PASS | V3 已将 V2 的偏紧通孔放宽，给 3D 打印误差留出更稳定余量。 |
| 肘部 M3 螺钉头沉孔径向余量 | 沉孔半径 2.95 mm，应大于头部半径 2.60 mm | 0.350 mm | PASS | V3 已放宽沉孔半径，降低实际螺钉头装不进去的风险。 |
| 肘部 M3 螺钉头沉孔深度余量 | 沉孔深度 2.40 mm，应大于头部高度 2.00 mm | 0.400 mm | PASS | 沉孔深度可容纳简化螺钉头。 |
| 肘部安装孔到 pad 边缘最小材料 | 建议 >= 2.0 mm | 通孔边缘 8.300 mm，沉孔边缘 7.100 mm | PASS | V3 同时加大 elbow pad，使放宽孔径后仍保留边缘材料。 |
| 肘部 2x2 孔间最小材料 | 建议通孔/沉孔之间保留 >= 2.0 mm | 通孔间 6.400 mm，沉孔间 4.000 mm | PASS | 孔系自身材料余量仍可接受。 |
| 肘部安装孔与 elbow 主孔最小孔间材料 | 保守按沉孔外缘建议 >= 0.8 mm | 通孔外缘 2.100 mm，沉孔外缘 0.900 mm | PASS | 这是 Step 14 的主要风险项。V3 将孔系向近端移动后，最近 M3 孔与 elbow 主孔之间的材料桥明显增加。 |
| 肩部 M3 夹紧/定位孔径向余量 | 孔半径 2.00 mm，应大于 M3 螺钉半径 1.50 mm | 0.500 mm | PASS | 肩部侧向孔给 M3 螺钉留有较宽松余量。 |
| 肩部夹紧孔到 pad 边缘最小材料 | 建议 >= 2.0 mm | 2.500 mm | PASS | V3 没有改肩部接口，该项继承 V2 的可接受结果。 |
| 肩部夹紧孔与 shoulder 主孔最小孔间材料 | 建议 >= 2.0 mm | 7.000 mm | PASS | 肩部夹紧孔与主孔之间有较充足材料。 |
| 简化舵机/连接板 footprint 是否落在 elbow pad 内 | 参考板投影应在 pad 内 | 最小边缘余量 4.000 mm | PASS | 用于检查简化 mating plate 不会明显悬空。 |

## 工程解释

- V3 的主关节孔、肩部夹紧孔和肘部 M3 孔在简化标准件尺寸下都能装入。
- V3 用少量局部材料换取更好的装配余量，这比继续追求极限轻量化更适合作品集阶段的工程表达。
- 本检查仍然是设计阶段筛查，不等同于真实打印、螺钉锁紧、疲劳和冲击载荷验证。

## 下一步

Step 16 应将 V3 接回 URDF/PyBullet，复查关节链、末端运动学和惯量更新后的静载力矩，确认 V3 的局部加厚没有明显破坏前面建立的仿真 baseline。

## 输出文件

- 简化标准件装配 STEP：`<repo>/05_improved_design/upper_arm_v3/assembly_check/upper_arm_v3_simplified_standard_parts_assembly.step`
- 简化标准件装配 STL：`<repo>/05_improved_design/upper_arm_v3/assembly_check/upper_arm_v3_simplified_standard_parts_assembly.stl`
- 结构化结果 JSON：`<repo>/05_improved_design/upper_arm_v3/assembly_check/upper_arm_v3_standard_part_check.json`
- 检查明细 CSV：`<repo>/05_improved_design/upper_arm_v3/assembly_check/upper_arm_v3_standard_part_checks.csv`
