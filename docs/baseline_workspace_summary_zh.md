# SO-101 官方 Baseline 工作空间与任务区域可达性分析

## 目的

本报告用于量化官方 SO-101 Follower 在未进行任何 AI 辅助重建或结构改进前的工作空间表现。

## 数据来源

- URDF：`<repo>/00_source_snapshot/Simulation_SO101/so101_new_calib.urdf`
- 末端执行器参考点：`gripper_frame_link`
- 采样主关节：`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`
- 随机采样姿态数量：30000

## 工作空间统计

- x 方向范围：-0.334 到 0.477 m
- y 方向范围：-0.438 到 0.437 m
- z 方向范围：-0.221 到 0.527 m
- 水平半径范围：0.002 到 0.477 m
- 0.025 m 分辨率下占用体素数量：11935
- 近似采样工作空间体积：0.18648 m^3

## 桌面任务区域可达性

设定桌面任务区域：

- x：0.15 到 0.40 m
- y：-0.25 到 0.25 m
- z：0.02 到 0.35 m
- 最近点距离阈值：0.035 m

结果：

- 估计可达目标比例：0.998，约 99.8%
- 平均最近采样距离：0.0124 m
- 95 分位最近采样距离：0.0201 m

## 工程解释

- 这是官方 SO-101 的几何工作空间 baseline，后续 AI 重建版和结构改进版都需要与它对比。
- 如果改进版结构明显缩小工作空间，就必须说明换来了什么收益，例如更高刚度、更低质量或更低力矩。
- 当前设定的桌面抓取/操作任务区域基本可以被官方 SO-101 覆盖，因此第一轮结构改进不应改变主关节轴线和连杆长度。

## 生成文件

- `04_structural_analysis/baseline_workspace_points.csv`
- `04_structural_analysis/baseline_workspace_plot.png`
- `04_structural_analysis/baseline_workspace_summary.md`
- `04_structural_analysis/baseline_workspace_summary_zh.md`
