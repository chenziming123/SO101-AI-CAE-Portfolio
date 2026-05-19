# upper_arm_v1 接回 URDF/PyBullet Smoke Test 报告

## 目的

验证 AI 辅助生成的 `upper_arm_v1` STL 是否可以接回 SO-101 的 URDF/PyBullet 仿真链路。

## 做法

- 复制官方 `Simulation_SO101/assets` 到 `05_improved_design/upper_arm_v1/urdf_smoke/assets`。
- 将 `upper_arm_v1_ai_rebuild.stl` 从毫米单位缩放为米单位，生成 `upper_arm_v1_ai_rebuild_m.stl`。
- 复制官方 URDF，生成 `so101_upper_arm_v1_visual_smoke.urdf`。
- 只替换 `upper_arm_link` 的 3D 打印件 visual mesh，不修改 collision 和 inertial。
- 用同一批随机关节角度比较官方 URDF 和改进 URDF 的末端位置。

## 输入文件

- 官方 URDF：`<repo>/00_source_snapshot/Simulation_SO101/so101_new_calib.urdf`
- 改进 URDF：`<repo>/05_improved_design/upper_arm_v1/urdf_smoke/so101_upper_arm_v1_visual_smoke.urdf`

## Mesh 缩放检查

- V1 原始 STL 包围盒：[150.5, 38.0, 39.299831]
- 缩放后仿真 mesh 包围盒：[0.1505, 0.038, 0.0393] m
- 缩放后 mesh watertight：True

## PyBullet Smoke Test 结果

- 关节名称是否一致：True
- 官方关节数：7
- 改进版关节数：7
- 随机姿态数量（含 zero pose）：2001
- zero pose 末端位置差：0.000000e+00 m
- 最大末端位置差：0.000000e+00 m
- 平均末端位置差：0.000000e+00 m

## 结论

- 改进版 URDF 可以被 PyBullet 加载。
- 关节数量和关节名称保持不变。
- 由于本步骤只替换 visual mesh，不修改 joint、collision、inertial，末端运动学应与官方 baseline 保持一致。
- 这一步证明 V1 CAD/STL 已经能进入机器人仿真链路，不再只是孤立 CAD 文件。

## 当前限制

- 本 smoke test 只替换 visual mesh，尚未替换 collision mesh。
- 尚未把 V1 的质量和惯量写入 URDF。
- 尚未做改进版静载力矩对比；下一步需要基于 V1 粗估质量更新 inertial 后再对比。

## 生成文件

- `05_improved_design/upper_arm_v1/urdf_smoke/so101_upper_arm_v1_visual_smoke.urdf`
- `05_improved_design/upper_arm_v1/urdf_smoke/assets/upper_arm_v1_ai_rebuild_m.stl`
- `05_improved_design/upper_arm_v1/urdf_smoke/upper_arm_v1_urdf_smoke_motion.csv`
- `05_improved_design/upper_arm_v1/urdf_smoke/upper_arm_v1_urdf_smoke_plot.png`
- `05_improved_design/upper_arm_v1/urdf_smoke/upper_arm_v1_urdf_smoke_report_zh.md`
