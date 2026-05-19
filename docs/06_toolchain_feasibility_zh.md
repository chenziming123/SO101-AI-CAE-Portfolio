# Ubuntu 22.04 下 ROS2 / colcon / FreeCAD / CalculiX 可用性说明

## 结论

当前服务器是 Ubuntu 22.04.5，可以继续使用这些工具，但路线要分清：

- ROS2：应使用 **ROS2 Humble**，不要在 22.04 上强行按 `so101-ros-physical-ai` 的 Jazzy/Ubuntu 24.04 路线安装。
- colcon：可以随 ROS2 开发工具安装，也可以用 Python 方式安装；当前服务器还没有。
- FreeCAD：Ubuntu 22.04 apt 源中有 `freecad`，版本候选为 `0.19.2+dfsg1-3ubuntu1`。
- CalculiX：Ubuntu 22.04 apt 源中有 `calculix-ccx`，版本候选为 `2.17-3`。
- Gmsh：系统 apt 源中有 `gmsh`，版本候选为 `4.8.4+ds2-2build1`；同时当前 `text-to-cad` venv 已有 Python `gmsh 4.15.2`。

## 当前服务器实际状态

| 工具 | 当前是否已安装 | Ubuntu 22.04 是否可用 | 备注 |
|---|---|---|---|
| `ros2` | 未安装 | 可用，推荐 ROS2 Humble | 需要添加 ROS2 apt 源后安装。 |
| `colcon` | 未安装 | 可用 | 建议随 ROS2 开发工具安装。 |
| `freecad` / `freecadcmd` | 未安装 | 可用 | apt 源有 0.19.2，适合基础 FreeCAD FEM/STEP 检查；如需新版可考虑 AppImage/conda。 |
| `ccx` | 未安装 | 可用 | apt 源有 `calculix-ccx 2.17-3`。 |
| 系统 `gmsh` | 未安装 | 可用 | apt 源有 4.8.4；当前 Python venv 已有 `gmsh 4.15.2`。 |
| Docker | 命令存在 | 当前不可直接用 | 用户无 `/var/run/docker.sock` 权限。 |

## 推荐安装路线

### 路线 A：先补 CAE 工具，风险最低

优先安装：

```bash
sudo apt update
sudo apt install -y freecad calculix-ccx gmsh
```

作用：

- `freecad/freecadcmd`：用于 FreeCAD FEM、STEP 检查、CAD 转换和自动化脚本。
- `ccx`：用于 CalculiX 线性静力求解，把当前 FEA 从自建 screening 升级到开源求解器流程。
- `gmsh`：用于系统级网格生成；Python venv 的 gmsh 仍可继续保留。

这条线和当前项目最直接相关，因为我们已经有 STEP、Gmsh mesh、VTK/PNG 后处理。

### 路线 B：ROS2 使用 Humble，不使用 Jazzy

Ubuntu 22.04 对应 ROS2 Humble。安装完成后再装 MoveIt 相关包。

推荐思路：

```bash
# 添加 ROS2 apt 源后
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
```

然后：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
colcon --help
```

注意：

- `so101-ros-physical-ai` README 写的是 Ubuntu 24.04 + ROS2 Jazzy。
- 在当前服务器 22.04 上，完整复现该项目需要做 Humble 适配，不能完全照搬 Jazzy 的依赖。
- 短期可以先做 `so101_description`/URDF/mesh 层整合；中期再用 Humble 跑 RViz/MoveIt。

### 路线 C：Jazzy 用容器或 24.04 环境

如果后续一定要严格复现 `so101-ros-physical-ai` 原项目，推荐：

- Ubuntu 24.04 + ROS2 Jazzy 实机/虚拟机；或
- Docker 容器跑 Ubuntu 24.04 + ROS2 Jazzy。

当前问题：

- 服务器已有 Docker 命令，但当前用户没有 Docker daemon 权限。
- 需要管理员把当前用户加入 docker 组，或提供可用容器环境。

## 对当前项目的建议

建议不要一次性安装所有工具。顺序应为：

1. 先安装 `freecad calculix-ccx gmsh`，把 CAE 工具链补起来。
2. 用 CalculiX 重跑 official / full V3 / V4 的静力分析，生成 `.inp`、`.frd/.dat`、VTK/PNG。
3. 再安装 ROS2 Humble + colcon，先跑 `so101_description`/RViz/MoveIt 的基础显示和规划。
4. 等 Docker 或 Ubuntu 24.04 环境可用后，再完整复现 Jazzy 版 `so101-ros-physical-ai`。

## 这一步的工程意义

这一步明确了工具链不是“不能用”，而是要按 Ubuntu 22.04 的生态重新选型：

- ROS2 走 Humble。
- CAE 走 FreeCAD + Gmsh + CalculiX。
- Jazzy/24.04 作为后续容器或新环境目标。

这样项目不会因为版本不匹配卡住，同时仍然能逐步接入成熟开源工具链。
