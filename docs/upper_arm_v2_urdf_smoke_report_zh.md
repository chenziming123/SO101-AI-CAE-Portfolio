# upper_arm_v2 接回 URDF/PyBullet Smoke Test 报告

## 这一步在做什么

验证 `upper_arm_v2` 的 STL 是否可以接回 SO-101 的 URDF/PyBullet 仿真链路，并确认它不会破坏原机器人的关节链和末端运动学。

## 做法

- 复制官方 `Simulation_SO101/assets` 到 V2 的 `urdf_smoke/assets`。
- 将 `upper_arm_v2_ai_rebuild.stl` 从毫米单位缩放为米单位，生成 `upper_arm_v2_ai_rebuild_m.stl`。
- 复制官方 URDF，生成 `so101_upper_arm_v2_visual_smoke.urdf`。
- 只替换 `upper_arm_link` 的 visual mesh，不修改 collision、joint 和 inertial。
- 使用同一批随机关节角度比较官方 URDF 和 V2 visual URDF 的末端位置。

## 输入文件

- 官方 URDF：`<repo>/00_source_snapshot/Simulation_SO101/so101_new_calib.urdf`
- V2 visual smoke URDF：`<repo>/05_improved_design/upper_arm_v2/urdf_smoke/so101_upper_arm_v2_visual_smoke.urdf`

## Mesh 缩放检查

- V2 原始 STL 包围盒：[150.5, 36.0, 39.299236]
- 缩放后仿真 mesh 包围盒：[0.1505, 0.036, 0.039299] m
- 缩放后 mesh watertight：True

## PyBullet Smoke Test 结果

- 关节名称是否一致：True
- 官方关节数：7
- V2 关节数：7
- 随机姿态数量（含 zero pose）：2001
- zero pose 末端位置差：0.000000e+00 m
- 最大末端位置差：0.000000e+00 m
- 平均末端位置差：0.000000e+00 m

## 结论

- V2 visual URDF 可以被 PyBullet 加载。
- 关节数量和关节名称保持不变。
- 由于本步骤只替换 visual mesh，不修改 joint、collision、inertial，末端运动学与官方 baseline 保持一致。
- 这说明 V2 不只是一个独立 CAD 文件，已经重新进入 SO-101 的仿真链路。

## 当前限制

- 本 smoke test 只替换 visual mesh，尚未替换 collision mesh。
- 尚未把 V2 的质量和惯量写入 URDF。
- 需要下一步基于 V2 体积比例更新 inertial，并重新做静载力矩对比。

## 生成文件

- `05_improved_design/upper_arm_v2/urdf_smoke/so101_upper_arm_v2_visual_smoke.urdf`
- `05_improved_design/upper_arm_v2/urdf_smoke/assets/upper_arm_v2_ai_rebuild_m.stl`
- `05_improved_design/upper_arm_v2/urdf_smoke/upper_arm_v2_urdf_smoke_motion.csv`
- `05_improved_design/upper_arm_v2/urdf_smoke/upper_arm_v2_urdf_smoke_plot.png`
- `05_improved_design/upper_arm_v2/urdf_smoke/upper_arm_v2_urdf_smoke_report_zh.md`
