# 仿真平台技术说明与二次开发手册

面向项目：空中机械臂具身智能平台  
版本基线：当前 `main.py + uav_controller.py + arm_controller.py + grasping.py + simulation_utils.py` 实现  
适用目的：帮助你快速理解现有框架，并能从“改第一个控制参数”开始，逐步过渡到 IK、轨迹规划、LLM 调度与异常重规划研究

---

## 1. 平台总览

当前项目已经具备一个可运行的 PyBullet 抓取演示链路：

1. `main.py` 初始化 PyBullet 仿真环境并运行抓取有限状态机。
2. `simulation_utils.py` 负责 URDF 预处理、动力学基础配置和演示场景生成。
3. `uav_controller.py` 控制无人机基座按等效模型悬停和飞行。
4. `arm_controller.py` 用 DLS 逆运动学求解机械臂末端目标位姿。
5. `grasping.py` 用约束式虚拟夹爪完成“抓住目标”的物理效果。

一句话理解：

`main.py` 是总导演，`uav_controller.py` 是飞行驾驶员，`arm_controller.py` 是机械臂逆解器，`grasping.py` 是抓取执行器，`simulation_utils.py` 是仿真底座与建模适配层。

---

## 2. 文件系统全景图

下面只列当前项目中真正参与仿真主链路的核心文件。

| 文件 | 作用 | 谁调用它 | 你何时修改它 |
| --- | --- | --- | --- |
| `main.py` | 仿真入口、PyBullet 初始化、任务状态机、抓取流程编排 | 直接运行 | 你想改任务流程、状态切换、阶段目标点、异常恢复逻辑时 |
| `simulation_utils.py` | URDF 查找、`package://` 路径解析、关节限位补丁、场景搭建 | `main.py` | 你想改模型加载、碰撞体、目标物、支撑台、关节阻尼时 |
| `uav_controller.py` | 无人机等效 PD 飞行控制 | `main.py` | 你想改悬停性能、飞行平滑性、轨迹跟踪时 |
| `arm_controller.py` | 机械臂关节映射、DLS IK、雅可比估计、关节位置控制 | `main.py` | 你想研究 IK、约束、末端稳定性、轨迹平滑时 |
| `grasping.py` | 接触/距离判定、相对速度判定、固定约束抓取 | `main.py` | 你想改抓取成功条件、抓取姿态容差、约束刚度时 |
| `urdf/fianl.urdf` | 机器人原始总装 URDF | `simulation_utils.py` 间接读取 | 你想改结构、惯量、碰撞几何、连杆定义时 |
| `generated/fianl_pybullet.urdf` | 预处理后供 PyBullet 加载的 URDF | `main.py` 动态生成 | 一般不手改，优先改 `simulation_utils.py` 的生成逻辑 |
| `requirements.txt` | 运行依赖 | 人工安装 | 增加算法包、数据处理包时 |
| `.gitignore` | 忽略虚拟环境、缓存、生成物 | Git | 增加日志、数据集、导出结果目录时 |
| `project/项目文档.docx` | 项目背景指导书 | 人工阅读 | 理解研究目标与实验要求时 |
| `project/.../sw_inertia_to_urdf.py` | SolidWorks 惯量写回 URDF 的工具脚本 | 离线使用 | 你要重新校准质量和惯量时 |
| `project/.../urdf_inertia_check.py` | 惯量检查脚本 | 离线使用 | 你怀疑 CAD 导出的惯量不合理时 |

### 2.1 当前主调用链

```text
python main.py
  -> ensure_project_venv()
  -> PyBullet connect / gravity / timestep / plane
  -> find_robot_urdf()
  -> prepare_urdf_for_pybullet()
  -> p.loadURDF(...)
  -> configure_loaded_robot()
  -> UAVController(...)
  -> ArmController(...)
  -> VirtualGripper(...)
  -> create_demo_scene(...)
  -> 有限状态机循环
       TAKEOFF
       TRANSIT
       ALIGN
       DESCEND
       APPROACH
       STABILIZE
       GRASP
       SECURE
       LIFT
       HOLD
```

### 2.2 哪个文件负责什么

#### `main.py`

核心职责：

- 负责物理引擎初始化
- 负责读取命令行参数
- 负责装配控制器
- 负责高层任务调度
- 负责闭环状态判定

你可以把它理解为“平台的大脑外壳”。

关键位置：

- `ensure_project_venv()`：自动切换到项目 `.venv`
- `parse_args()`：所有演示参数入口
- `main()`：环境初始化和主控制循环
- `DemoState`：高层任务状态机
- `compute_side_approach_offsets()`：侧向逼近策略
- `constrain_grasp_target()`：防止末端穿入柱台的安全约束

#### `simulation_utils.py`

核心职责：

- 把 CAD 导出的 URDF 修补成 PyBullet 可稳定加载的版本
- 给零宽关节限位补上可用范围
- 为关节补充阻尼与摩擦
- 搭建演示场景中的柱台和目标方块

它是“建模文件”和“仿真控制代码”之间的适配层。

#### `uav_controller.py`

核心职责：

- 提供无人机基座的等效悬停/飞行控制
- 将位置误差转换为平移速度命令
- 强制机体保持水平，忽略机械臂对基座的反作用扰动

注意：

当前并不是严格的旋翼动力学模型，而是为了稳定操作任务而采用的“等效运动学/PD 基座模型”。

#### `arm_controller.py`

核心职责：

- 识别机械臂关节索引
- 读取关节限位
- 建立隐藏的固定基座运动学代理模型
- 用数值雅可比 + DLS 求逆
- 输出平滑的关节位置控制命令

这就是当前项目中最核心的运动学算法文件。

#### `grasping.py`

核心职责：

- 判断末端与目标是否接近
- 判断相对速度是否足够小
- 在满足条件时创建固定约束模拟抓取
- 抓取后禁用不必要碰撞，减小抖动

它不是“真实夹爪”，而是“无夹爪模型情况下的等效抓取器”。

---

## 3. 当前平台里，谁是“LLM 与底层驱动的桥梁”？

这里必须实话实说：

当前代码仓库里，还没有真正接入 LLM API，也没有独立的 `planner.py` 或 `policy_runtime.py` 文件。

所以现阶段：

- `main.py` 实际承担了“高层策略调度器”的角色
- `uav_controller.py / arm_controller.py / grasping.py` 实际承担了“底层原子动作执行器”的角色

换句话说，当前平台已经具备了 **Code as Policies 的雏形**，但还没有把“策略代码生成器”从人工写死状态机升级成“由 LLM 动态编排”。

### 3.1 现阶段的“代码即策略”是怎样工作的

当前策略并非由大模型生成，而是人工写在 `main.py` 里的状态机：

- `TAKEOFF`：无人机起飞到悬停高度
- `TRANSIT`：飞到远端预抓取点
- `DESCEND`：降低到抓取高度
- `APPROACH`：机械臂靠近目标
- `GRASP`：检测可抓取条件并创建约束
- `LIFT/HOLD`：抓后提升并稳定保持

这本质上就是一段“人工编写的策略代码”。

如果未来换成 LLM，那么 LLM 输出的将不再是自然语言，而是一串对底层技能 API 的调用组合，例如：

```python
fly_to([0.55, 0.25, 1.25])
move_ee_to(target.pregrasp_pose)
if grasp_ready(target):
    grasp(target)
lift_object(0.12)
```

这就是 Code as Policies 在本项目中的目标形态。

---

## 4. 算法学习与修改入口

这部分最重要。我直接告诉你“想学什么，就先改哪里”。

### 4.1 想学习和优化 IK / DLS 算法

首选文件：

- `arm_controller.py`

首选函数：

- `solve_dls()`：DLS 逆解主函数
- `_compute_arm_jacobian()`：数值雅可比估计
- `_orientation_error()`：姿态误差定义
- `set_joint_targets()`：将逆解结果变成关节控制命令

#### 关键修改点 1：DLS 主求解器

当前入口在 `arm_controller.py` 的 `solve_dls()`。

```python
def solve_dls(
    self,
    target_position,
    target_orientation=None,
    damping=0.08,              # 奇异位形附近的阻尼项，越大越稳，越小越灵敏
    step_size=0.7,             # 每轮关节更新步长，越大收敛快，越大也越容易振荡
    max_iterations=80,         # 每次 IK 的最大迭代次数
    position_tolerance=0.01,   # 位置收敛阈值
    orientation_tolerance=0.08 # 姿态收敛阈值
):
```

逻辑说明：

- `damping` 决定奇异位形附近的鲁棒性
- `step_size` 决定每次求解更新是否激进
- `max_iterations` 决定是否愿意用更多算力换更小误差
- `position_tolerance` 决定状态机多早认为“到位了”

研究建议：

- 想研究奇异抑制，优先改 `damping`
- 想研究收敛速度与震荡权衡，优先改 `step_size`
- 想做带权 DLS，可从 `task_jacobian` 和 `task_error` 加权开始
- 想做关节极限回避，可在 `dq` 后加入 null-space 项

#### 关键修改点 2：数值雅可比

当前入口在 `arm_controller.py` 的 `_compute_arm_jacobian()`。

```python
def _compute_arm_jacobian(..., epsilon=1e-4):
    # 通过对每个关节做微小扰动，近似计算末端位置/姿态对关节的偏导
```

逻辑说明：

- `epsilon` 太小容易数值噪声大
- `epsilon` 太大又会让线性化误差增加
- 当前采用的是“数值差分雅可比”，实现简单，适合科研原型验证

研究建议：

- 若你要做高性能控制，可替换为解析雅可比
- 若你要做精度分析，可比较不同 `epsilon` 下的收敛残差与抖动

#### 关键修改点 3：关节控制平滑器

当前入口在 `arm_controller.py` 的构造函数和 `set_joint_targets()`。

```python
def __init__(..., joint_step_limit=0.035, max_motor_force=35.0):
    ...

def set_joint_targets(...):
    clipped_targets = np.clip(
        target,
        current_positions - self.joint_step_limit,
        current_positions + self.joint_step_limit,
    )
```

逻辑说明：

- `joint_step_limit` 是每个仿真步允许的最大关节变化量
- `max_motor_force` 决定位置控制器的执行刚性
- 这是解决“末端颤抖”“抓后关节抖动”的第一调参入口

如果你现在就想做第一个实验：

把 `joint_step_limit` 从 `0.035` 改到 `0.02`，重新运行观察抓后末端抖动是否减小。

---

### 4.2 想调整无人机飞行控制器

首选文件：

- `uav_controller.py`

首选函数：

- `__init__()`：控制器增益入口
- `step()`：飞行控制主逻辑
- `at_target()`：到位判定

#### 关键修改点 1：PD 增益

```python
def __init__(
    ...,
    kp_xy=18.0,
    kd_xy=10.0,
    kp_z=24.0,
    kd_z=12.0,
    kp_attitude=4.5,
    kd_attitude=1.0,
    kp_yaw=2.5,
    kd_yaw=0.8,
):
```

逻辑说明：

- `kp_xy / kd_xy` 决定水平面平移响应
- `kp_z / kd_z` 决定高度悬停稳定性
- `kp_yaw / kd_yaw` 决定偏航调整

注意：

当前 `step()` 中实际上采用的是“位置误差 -> 期望线速度 -> 直接重置机体位姿”的等效控制方式，因此它比真实动力学控制稳定，但也更抽象。

#### 关键修改点 2：飞行等效模型

`step()` 的核心逻辑是：

```python
position_error = target - state.position
velocity_error = -state.linear_velocity

desired_linear_velocity = KP * position_error + KD * velocity_error
next_position = state.position + desired_linear_velocity * self.time_step

pb.resetBasePositionAndOrientation(...)
pb.resetBaseVelocity(...)
```

逻辑说明：

- 当前不是通过 `applyExternalForce` 施加力
- 而是通过“位置积分 + 直接重置基座”的方式保证飞行段稳定
- 适合先做操作任务验证，再逐步替换成更真实的动力学控制器

如果你后续要做更真实的 UAV 动力学：

- 在 `step()` 中改为推力/力矩分配
- 用 `applyExternalForce()` 和 `applyExternalTorque()` 替代 `resetBasePositionAndOrientation()`
- 同时要重新处理机械臂反作用对基座姿态的耦合

---

### 4.3 想调整机械臂轨迹规划

当前项目里，轨迹规划主要不在 `arm_controller.py`，而在 `main.py` 的状态机逻辑里。

首选位置：

- `compute_side_approach_offsets()`：定义预抓取点和抓取点的相对偏移
- `constrain_grasp_target()`：限制末端目标点不要扎入支撑柱
- `APPROACH / STABILIZE / GRASP / LIFT / HOLD` 各状态里的目标生成逻辑

#### 关键修改点：抓取路径不是一条函数，而是“状态 + 目标点生成”

例如当前逻辑是：

1. 先飞到远端悬停点
2. 再下降到抓取高度
3. 机械臂从侧面预抓取
4. 机械臂进一步压近
5. 成功后再上抬

所以如果你要加“曲线轨迹”“分段轨迹”“避障路径”，最应该改的是 `main.py`，而不是只盯着 IK 求解器。

你可以从下面这个思路开始：

```python
# 现在：直接给一个抓取目标点
arm_controller.command_end_effector(grasp_target)

# 下一步可升级为：先生成轨迹路标点，再逐点跟踪
for waypoint in grasp_waypoints:
    arm_controller.command_end_effector(waypoint)
```

---

### 4.4 想调整抓取判定与抓取稳定性

首选文件：

- `grasping.py`

首选函数：

- `measure_target_alignment()`
- `is_grasp_ready()`
- `close_with_options()`

关键入口：

```python
def __init__(..., grasp_distance_threshold=0.085, max_relative_speed=1.20):
```

逻辑说明：

- `grasp_distance_threshold` 越小，抓取越严格
- `max_relative_speed` 越小，抓取越稳定，但越容易错过时机

抓取后稳定性还受下面几项影响：

- `changeConstraint(..., maxForce=400.0)` 的约束强度
- 抓后是否禁用物体与机器人自碰撞
- 目标物本身的阻尼参数

---

## 5. 关键代码入口，按“注释 + 逻辑说明”理解

### 5.1 `main.py` 是高层任务编排器

```python
state = DemoState.TAKEOFF

for step in range(args.sim_steps):
    if state == DemoState.TAKEOFF:
        uav_controller.step(hover_target)
        arm_controller.command_home()

    elif state == DemoState.APPROACH:
        uav_controller.step(hover_target)
        residual = arm_controller.command_end_effector(approach_target)

    elif state == DemoState.GRASP:
        arm_controller.command_end_effector(grasp_target)
        if gripper.is_grasp_ready(scene.target_id):
            attempt = gripper.close_with_options(...)
```

逻辑说明：

- `main.py` 不做底层控制求解
- 它做的是“当前阶段要飞到哪”“当前阶段末端要到哪”“什么时候可以切状态”
- 这正是未来接 LLM 时最容易替换的一层

### 5.2 `arm_controller.py` 是末端空间到关节空间的桥

```python
def command_end_effector(self, target_position, target_orientation=None):
    solution, residual = self.solve_dls(target_position, target_orientation)
    self.set_joint_targets(solution)
    return residual
```

逻辑说明：

- 你只给末端目标
- `solve_dls()` 负责解关节角
- `set_joint_targets()` 负责把关节角送给 PyBullet 电机

这就是机械臂的“底层运动学桥梁”。

### 5.3 `grasping.py` 是“抓取动作原子化”的典型示例

```python
if distance > self.grasp_distance_threshold:
    return GraspAttempt(False, "target is outside the grasp threshold")

if relative_speed > self.max_relative_speed:
    return GraspAttempt(False, "target is still moving too fast")

self.constraint_id = self.pb.createConstraint(...)
```

逻辑说明：

- 先判断是否满足抓取前提
- 再执行约束附着
- 最后调整碰撞和阻尼，减小抓后抖动

这是你今后设计“原子技能库”时非常好的模板。

---

## 6. 具身智能接入机制：如何把它升级为 Code as Policies

### 6.1 当前状态

当前仓库还没有真正的 LLM 接口文件，因此严格来说：

- 已有“原子动作执行层”
- 已有“手写高层策略层”
- 尚未有“LLM 生成策略层”

### 6.2 推荐的后续模块划分

为了不打乱现有结构，建议新增两个文件，而不是把 LLM 逻辑硬塞进 `main.py`：

| 建议新增文件 | 作用 |
| --- | --- |
| `skills.py` | 定义原子技能库，对底层控制器做统一 API 封装 |
| `policy_runtime.py` | 接收自然语言或 LLM 输出代码，调用 `skills.py` 中的技能 |

推荐关系：

```text
自然语言指令
  -> LLM
  -> 生成策略代码 / 函数调用序列
  -> policy_runtime.py
  -> skills.py
  -> uav_controller.py / arm_controller.py / grasping.py
  -> PyBullet
```

### 6.3 什么叫“原子技能库”

原子技能是“高层策略可以直接调用，但底层已经封装好细节”的动作接口。

例如可以定义：

```python
class SkillLibrary:
    def takeoff(self, z: float): ...
    def fly_to(self, xyz: list[float]): ...
    def move_ee_to(self, xyz: list[float], quat: list[float] | None = None): ...
    def grasp(self, target_id: int): ...
    def lift(self, dz: float): ...
    def hold_pose(self, xyz: list[float], quat=None): ...
    def check_grasp_ready(self, target_id: int) -> bool: ...
    def check_failure(self) -> dict: ...
```

这些函数内部不要暴露太多 PyBullet 细节，而是只暴露对策略最有意义的语义接口。

### 6.4 如果你要增加一个新的动作：例如“旋转抓取”

你应该按下面顺序做：

1. 在 `skills.py` 里新增技能 API，例如 `rotate_and_grasp(...)`
2. 在技能函数内部调用 `arm_controller.command_end_effector(..., target_orientation=...)`
3. 给这个技能写清楚输入、输出、失败条件
4. 在 `policy_runtime.py` 里把这个技能暴露给 LLM 可调用环境
5. 在 system prompt 或工具描述里告诉大模型这个技能的语义和约束

示例：

```python
def rotate_and_grasp(self, target_position, target_orientation, target_id):
    # 1. 先旋转末端到目标姿态
    self.arm_controller.command_end_effector(
        target_position,
        target_orientation=target_orientation,
    )

    # 2. 再检测是否具备抓取条件
    if self.gripper.is_grasp_ready(target_id):
        return self.gripper.close(target_id)
    return {"success": False, "reason": "not ready"}
```

### 6.5 怎样让大模型“感知到新技能”

关键不是让大模型看源码，而是给它一份稳定、简洁、可执行的技能清单。

推荐做法：

1. 给每个技能写清晰 docstring
2. 限制输入参数数量和含义
3. 提供机器可读的技能注册表
4. 给 LLM 一个状态摘要接口，而不是整个仿真底层对象

例如：

```python
SKILL_REGISTRY = {
    "fly_to": {
        "signature": "fly_to(xyz: list[float])",
        "description": "Move the UAV base to a hover waypoint.",
    },
    "rotate_and_grasp": {
        "signature": "rotate_and_grasp(target_position, target_orientation, target_id)",
        "description": "Rotate wrist to desired grasp orientation and attach target if ready.",
    },
}
```

这样 LLM 就能把“抓住并旋转这个物体”映射成已有 API，而不是自己发明底层控制细节。

---

## 7. 异常监测与自主纠错：当前已有基础，尚需继续完善

### 7.1 当前代码里已经存在的闭环检测

当前闭环检测主要分散在以下位置：

- `uav_controller.at_target()`：判断无人机是否到达目标点
- `arm_controller.at_pose()`：判断末端是否到达目标位置
- `gripper.is_grasp_ready()`：判断是否具备抓取前提
- `main.py` 中 `GRASP` 状态超时：失败时抛异常

这说明平台已经有“监测”能力，但还没有完整“自主纠错/重规划”闭环。

### 7.2 推荐的下一步：加入恢复状态

如果你准备做科研增强，建议在 `main.py` 状态机中补两个状态：

- `RECOVER`
- `REPLAN`

建议的失败处理逻辑：

```text
抓取失败
  -> RECOVER: 机械臂先退回安全位
  -> 重新测量目标位置
  -> REPLAN: 重新生成 approach/grasp waypoint
  -> 再次进入 APPROACH
```

### 7.3 推荐新增的监测指标

- 末端位置误差 `||x_target - x_current||`
- IK 残差 `residual`
- 抓取距离 `distance`
- 相对速度 `relative_speed`
- 基座悬停误差 `||p_target - p_uav||`
- 关节是否逼近限位
- 是否与柱台发生接触 `getContactPoints(robot, pedestal)`

---

## 8. 科研新手学习路线图

如果你现在还不确定“该怎么学”，建议按下面三个阶段推进。

### 第一阶段：基础环境熟悉

目标：

- 跑通仿真
- 看懂文件职责
- 能修改一个参数并观察现象变化

建议任务：

1. 运行 `python main.py`
2. 修改 `uav_controller.py` 里的 `kp_z`
3. 修改 `arm_controller.py` 里的 `joint_step_limit`
4. 观察无人机悬停和抓后抖动变化

阶段产出：

- 你能说清楚谁控制 UAV，谁控制 arm，谁做 grasp
- 你能独立完成一次“小参数改动 -> 结果对比”

### 第二阶段：底层控制调优

目标：

- 学会分析抖动、超调、抓取失败、穿模
- 学会针对控制器和 IK 做小规模算法改进

建议任务：

1. 调 `solve_dls()` 的 `damping` 和 `step_size`
2. 调 `grasp_distance_threshold` 和 `max_relative_speed`
3. 把 `compute_side_approach_offsets()` 改成三段式逼近
4. 给 `main.py` 增加失败恢复状态

阶段产出：

- 一组更稳的控制参数
- 一个更合理的预抓取/抓取轨迹
- 初步的异常恢复机制

### 第三阶段：高层逻辑调度与 LLM 接入

目标：

- 让自然语言能够驱动技能编排
- 形成 Code as Policies 的论文原型

建议任务：

1. 新增 `skills.py`
2. 把 `main.py` 里的状态逻辑拆成技能 API
3. 新增 `policy_runtime.py`
4. 用自然语言生成技能调用序列
5. 加入失败检测和自动重规划

阶段产出：

- 一套原子技能库
- 一个可解释的策略执行层
- 一个可复现实验的具身智能演示系统

---

## 9. 最常用的 PyBullet 调试方法

这是你之后最常用的一组“科研调试工具箱”。

### 9.1 最值得打印和记录的接口

- `p.getBasePositionAndOrientation(robot_id)`
- `p.getBaseVelocity(robot_id)`
- `p.getJointState(robot_id, joint_index)`
- `p.getLinkState(robot_id, end_effector_link_index, computeForwardKinematics=True)`
- `p.getContactPoints(bodyA=..., bodyB=...)`
- `p.getClosestPoints(bodyA=..., bodyB=..., distance=...)`

### 9.2 最值得观察的指标

- 无人机位置误差
- 无人机高度误差
- 末端位置误差
- IK 残差
- 物块与末端距离
- 抓取成功前的相对速度
- 抓取后物块相对末端是否滑移
- 机械臂和柱台是否产生接触

### 9.3 图形化调试建议

推荐你逐步加入：

- `p.addUserDebugText()`：显示当前状态机状态
- `p.addUserDebugLine()`：画出目标点和末端轨迹
- `print()` 状态切换日志：当前已经有
- 减慢仿真速度：当前 `--sleep` 已支持

非常实用的调试思路：

先看“数值误差”再看“视觉效果”，不要只靠肉眼判断控制器好坏。

---

## 10. 第一个最值得你马上动手改的参数

如果你希望阅读后立刻开始第一次实验，我建议从下面这个参数开始：

文件：

- `arm_controller.py`

位置：

- `ArmController.__init__(..., joint_step_limit=0.035, ...)`

建议实验：

```python
joint_step_limit = 0.02
```

为什么先改它：

- 改动最小
- 影响最直观
- 对抓后抖动、末端颤抖通常最敏感
- 不会像大改 IK 或 UAV 动力学那样一下子把系统整体打散

你这次实验时建议同时记录三件事：

1. 抓取成功率是否下降
2. 抓后末端抖动是否减小
3. 整个抓取耗时是否变长

---

## 11. 下一步推荐顺序

如果你问“接下来我先做什么最划算”，建议顺序如下：

1. 先调 `joint_step_limit` 和 `max_motor_force`
2. 再调 `solve_dls()` 的 `damping` 和 `step_size`
3. 然后把 `main.py` 的接近轨迹改成多路标点
4. 再加入 `RECOVER / REPLAN` 状态
5. 最后再抽象 `skills.py`，接入 LLM

这个顺序的好处是：

- 先把底层执行稳定住
- 再做高层智能调度
- 避免“策略很聪明，但底层根本不稳”的典型科研原型问题

---

## 12. 结语

对于当前这个项目，你可以把学习主线牢牢记成一句话：

先把 **仿真能稳稳地飞、稳稳地抓、稳稳地抬**，再把这些动作封装成技能，最后再让 LLM 去调度这些技能。

这条路线最符合科研实现顺序，也最容易产出高质量、可复现、可写论文的实验系统。
