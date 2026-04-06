# SRP PyBullet Aerial Manipulator Demo

一个基于 PyBullet 的空中机械臂抓取演示项目。当前仓库提供了无人机基座控制、机械臂逆运动学求解、虚拟抓取约束，以及完整的抓取状态机流程，可用于演示、课程展示和后续算法开发。

## 项目内容

本项目当前实现了一个完整的抓取演示流程：

- 无人机起飞并飞行到预抓取位置
- 机械臂根据目标位置进行末端逼近
- 满足条件后创建虚拟抓取约束
- 抓取成功后抬升目标并稳定悬停

主入口见 [`main.py`](./main.py)。

## 环境要求

- Python 3.11 推荐
- Windows + PowerShell 已验证
- 依赖见 [`requirements.txt`](./requirements.txt)

## 安装方式

建议在仓库根目录创建虚拟环境后再安装依赖：

```powershell
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install -r requirements.txt
```


## 常用参数

`main.py` 支持以下常用参数：

- `--gui`：使用 PyBullet 图形界面
- `--headless`：无图形界面运行
- `--fast`：尽可能快速运行，不按真实时间播放
- `--close-on-finish`：结束后立即关闭
- `--sim-steps`：最大仿真步数
- `--time-step`：仿真步长
- `--hover-z`：悬停高度
- `--grasp-z`：抓取阶段飞行高度
- `--lift-z`：抓取后抬升高度
- `--transit-x` / `--transit-y`：预抓取航点位置



## 项目结构

```text
.
├─ main.py                 # 仿真入口与抓取状态机
├─ uav_controller.py       # 无人机基座控制
├─ arm_controller.py       # 机械臂逆运动学与关节控制
├─ grasping.py             # 虚拟抓取逻辑
├─ simulation_utils.py     # URDF 预处理、场景创建、资源解析
├─ urdf/
│  └─ final.urdf           # 机器人总装模型
├─ project/                # CAD/URDF 导出资源与 mesh
├─ generated/              # 运行时自动生成的 PyBullet URDF
└─ requirements.txt
```

## 资源文件说明

这个仓库不仅依赖 [`urdf/final.urdf`](./urdf/final.urdf)，还依赖 [`project/`](./project) 目录下的 mesh 和导出资源。

[`simulation_utils.py`](./simulation_utils.py) 会把 URDF 里的 `package://...` 路径解析到以下目录：

- `project/uav_urdf_export_v1/...`
- `project/solidworks_inertia/...`

如果缺少这些资源，程序会在加载 URDF 或 mesh 时失败。



## 运行输出与已知现象

- 运行时会打印关节布局、目标生成位置、状态切换信息
- PyBullet 可能输出部分 inertia warning
- 当前已验证无界面模式可以完整跑通抓取流程

这些 warning 目前不会阻止演示完成，但如果后续要做更高保真动力学建模，建议进一步检查源 URDF 的惯量参数。

## 开发说明

当前仓库的核心逻辑分布如下：

- [`main.py`](./main.py)：整体流程与高层状态机
- [`uav_controller.py`](./uav_controller.py)：无人机位置控制
- [`arm_controller.py`](./arm_controller.py)：机械臂 DLS 逆解与运动控制
- [`grasping.py`](./grasping.py)：接近判断与抓取约束
- [`simulation_utils.py`](./simulation_utils.py)：URDF 修补、资源路径解析、场景生成


