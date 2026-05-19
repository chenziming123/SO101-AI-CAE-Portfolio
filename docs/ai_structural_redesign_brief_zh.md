# SO-101 Follower AI 结构重建设计 Brief

## 目标

使用 AI 辅助 CAD 生成方法，对 SO-101 的选定结构件进行参数化重建和局部改进，同时保留官方机械臂的运动学结构。

## 必须保留的内容

- 保留官方 joint 名称、joint axis 和运动链。
- 第一轮重建设计中保留 `shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll` 和 gripper frame 的位置。
- 保留舵机安装接口、舵盘间隙、螺丝孔位和 URDF link 归属关系。
- 不修改 `00_source_snapshot/`；所有改进版 CAD 放在 `05_improved_design/`。

## 来自 URDF 的关键 Baseline 尺寸

- shoulder_lift 到 elbow_flex 近似跨度：116.0 mm
- elbow_flex 到 wrist_flex 近似跨度：135.0 mm
- wrist_flex 到 wrist_roll 近似跨度：63.7 mm
- wrist/gripper frame offset 贡献：98.4 mm

## 重建设计目标

### 目标 1：upper_arm_link

- 控制关节：`shoulder_lift`
- 风险分数：100.0 / 100
- baseline p95 弯矩：1.1496 Nm
- 简化 p95 转角估计：2.660 deg
- 改进方向：通过加强筋、盒式截面和线槽提高弱轴刚度，同时不移动肩部和肘部关节 frame。
- 必须保留的接口：保留 `shoulder_lift` 和 `elbow_flex` 的 joint origin、axis、舵盘间隙和螺丝安装接口。

### 目标 2：lower_arm_link

- 控制关节：`elbow_flex`
- 风险分数：73.4 / 100
- baseline p95 弯矩：0.7142 Nm
- 简化 p95 转角估计：2.158 deg
- 改进方向：降低远端质量，并在腕部电机座和线束应力释放区域增加局部加强。
- 必须保留的接口：保留 `elbow_flex` 和 `wrist_flex` 的 joint origin、腕部电机座对齐关系和腕部安装孔位。

### 目标 3：wrist_gripper_module

- 控制关节：`wrist_flex`
- 风险分数：53.2 / 100
- baseline p95 弯矩：0.2725 Nm
- 简化 p95 转角估计：2.194 deg
- 改进方向：降低远端质量，增加紧凑局部加强筋，并加入可换指尖或小型工具法兰。
- 必须保留的接口：保留 `wrist_flex`、`wrist_roll`、`gripper` joint frame、tool frame 和夹爪开合几何兼容性。

## 第一版改进 CAD 的成功标准

- 输出可编辑的 build123d/Python CAD 源文件，并导出 STEP/STL。
- 工作空间可达性接近官方 baseline。
- 在不改变运动学 frame 的前提下，降低远端质量或提高弱轴有效刚度。
- mesh 替换后通过 PyBullet URDF 加载和随机运动 smoke test。
- 如果质量、刚度或工作空间出现取舍，必须在报告中明确说明。
