# upper_arm_link V1 AI 辅助 CAD 重建设计报告

## 当前步骤

Step 8：AI 辅助 `upper_arm_link` 参数化 CAD 重建。

## 这一步做了什么

- 在 `05_improved_design/upper_arm_v1/` 中建立第一版改进件目录。
- 编写 `upper_arm_v1_cad.py`，用 build123d 参数化生成 `upper_arm_link` 第一版重建模型。
- 使用服务器 `<text-to-cad-cad-env>` 中的 CAD 环境导出 STEP/STL。
- 编写 `validate_upper_arm_v1.py`，对官方 STL 和改进 STL 做基础几何对比。
- 生成可编辑源文件、STEP、STL、参数 JSON 和中文验证报告。

## 为什么先做 upper_arm_link

前面 Step 6 和 Step 7 已经得到工程依据：

- 100g 末端负载下，`shoulder_lift` 是主要静载力矩瓶颈。
- `upper_arm_link` 是 shoulder 到 elbow 的主承力结构件。
- 简化结构筛查中，`upper_arm_link` 的结构改进优先级最高。

因此第一轮不重做整台机械臂，而是先对 `upper_arm_link` 做局部参数化重建，风险更可控，也更容易和官方 baseline 对比。

## 设计约束

- 保留 shoulder_lift 到 elbow_flex 的设计跨度：116.0 mm。
- 不修改官方 `00_source_snapshot/`。
- 不改变官方 URDF 的 joint 名称、joint axis 和运动链。
- 不直接声称 V1 已经可以装机打印；它是第一版可编辑 CAD 概念件。

## V1 结构特征

- 上下双 rail：形成主要抗弯路径。
- 中央 web：形成盒式/桁架式受力结构。
- 轻量化孔：降低中部材料用量。
- 竖向 ribs：提高弱轴刚度。
- 顶部线槽：体现线束管理能力。
- 两端 boss：保留 shoulder/elbow 关节接口的参数化位置。

## 导出结果

- `upper_arm_v1_cad.py`：参数化 CAD 源文件。
- `upper_arm_v1_ai_rebuild.step`：STEP CAD 文件。
- `upper_arm_v1_ai_rebuild.stl`：STL mesh 文件。
- `upper_arm_v1_parameters.json`：参数记录。
- `upper_arm_v1_validation_zh.md`：几何验证报告。

## 基础几何对比

| 模型 | watertight | 包围盒尺寸 xyz | 体积 | PLA 估算质量 |
|---|---|---:|---:|---:|
| 官方 upper arm | True | 142.15, 24.50, 67.30 mm | 117328.19 mm^3 | 145.49 g |
| 改进版 V1 | True | 150.50, 38.00, 39.30 mm | 93885.71 mm^3 | 116.42 g |

## 当前结论

- V1 已经成功导出 STEP/STL。
- V1 STL 是 watertight mesh。
- 按 PLA 密度粗估，V1 相比官方 upper arm 体积/质量约下降 20.0%。
- V1 的 z 向高度更低，y 向宽度更大，说明它把结构从高窄形态改成更宽的盒式/桁架式形态。

## 当前限制

- V1 还没有和官方 URDF 完成 mesh 替换。
- V1 还没有做 PyBullet 加载验证。
- V1 还没有做与官方装配孔位的一一精确匹配。
- 当前质量估算基于 STL 体积和 PLA 密度，是粗略比较，不等于真实打印质量。

## 下一步

Step 9：将 `upper_arm_v1` 接回 SO-101 URDF/PyBullet 做 smoke test。

具体要做：

- 复制官方 URDF，建立改进版 URDF，不修改官方 baseline。
- 把 `upper_arm_link` 的 visual mesh 指向 `upper_arm_v1_ai_rebuild.stl`。
- 先只做 visual 替换，不立刻改 collision/inertial。
- 用 PyBullet 检查 URDF 是否能加载、关节是否正常、随机运动是否正常。
- 如果加载成功，再做质量/inertial 替换和力矩对比。
