# 6 轴机械臂项目 — 知识点与问题记录

## 一、核心知识点

### 1. 正运动学 (FK) — DH 参数法
- 每个关节用 4 个 DH 参数描述：连杆长度 $a$、扭转角 $\alpha$、偏距 $d$、关节角 $\theta$
- 齐次变换矩阵连乘：$T_n^0 = T_1^0 \cdot T_2^1 \cdots T_n^{n-1}$
- C++ 实现：`Eigen::Matrix4d` 表示变换矩阵，`Eigen::AngleAxisd` 处理旋转

### 2. 逆运动学 (IK) — 数值法
- 基于雅可比矩阵（Jacobian）伪逆的迭代法
- 迭代公式：$\theta_{k+1} = \theta_k + J^+(\theta_k) \cdot e$
- 数值微分计算雅可比：逐关节微扰 $\Delta\theta$ 观察末端位移

### 3. URDF / Xacro 建模
- `<link>` 定义连杆（visual + collision + inertial）
- `<joint>` 连接两个 link，定义运动类型和轴向
- `<ros2_control>` 标签声明硬件接口（position command / state）
- **Xacro** 支持 `$(find package)` 宏和条件表达式

### 4. ros2_control 控制器架构
- `controller_manager` → 管理所有控制器的生命周期
- `joint_state_broadcaster` → 广播关节状态到 `/joint_states`
- `joint_trajectory_controller` → 接收轨迹并执行 PID 跟踪
- 配置通过 `controllers.yaml` 文件声明

### 5. MoveIt2 运动规划
- **Planning Scene** — MoveIt 维护的世界模型（robot + obstacles）
- **OMPL** — 采样基规划器（RRTConnect），自动避障
- **SRDF** — 定义规划组、碰撞矩阵、默认姿态
- **KDL** — 默认运动学求解插件
- **FollowJointTrajectory** — MoveIt → ros2_control 的 Action 接口

### 6. 避障逻辑
- 构建 `CollisionObject` 消息（形状 + 位姿 + ADD 操作）
- 通过 `/planning_scene` 话题发布到 move_group
- OMPL 采样时自动检测碰撞状态 → 剔除碰撞路径点
- 支持 Box、Cylinder、Sphere 等基本形状

---

## 二、遇到的问题与挑战

### 问题 1：Gazebo Classic 与 Gazebo Harmonic 不兼容
**现象**：ROS2 Jazzy 不再支持 Gazebo Classic（`gazebo_ros_pkgs`），必须使用 Gazebo Harmonic。  
**解决**：
- 将 `gazebo_ros` 替换为 `ros_gz_sim`
- 插件从 `gazebo_ros2_control` 改为 `gz_ros2_control::GazeboSimROS2ControlPlugin`
- Launch 文件完全重写，使用 `ros_gz_sim` 的 launch API

### 问题 2：VMware 虚拟机上 Gazebo GUI 崩溃
**现象**：Gazebo GUI 打开后立刻关闭，`libEGL warning` 错误。  
**原因**：VMware 的虚拟 GPU 不完全支持 OpenGL ES，Gazebo 的 Ogre2 渲染引擎需要硬件加速。  
**解决**：
- 将 Gazebo 拆分为 headless server（`-s` 参数）+ 延迟 GUI client
- 物理引擎独立运行不受 GPU 影响，GUI 崩溃不影响仿真

### 问题 3：控制器加载失败
**现象**：`spawner` 报错 `Could not contact service /controller_manager/list_controllers`。  
**原因**：
1. Gazebo GUI 渲染与控制器初始化竞争 CPU/GPU 资源
2. `robot_description` 需要先发布才能初始化 controller_manager  
**解决**：
- 使用 `OnProcessExit` 事件处理器：等 `spawn_entity` 完成后再加载控制器
- 分离 headless server 和 GUI，server 先完成初始化

### 问题 4：URDF 连杆视觉断开
**现象**：link_2 和 link_3 的视觉模型看起来没有物理连接。  
**原因**：cylinder 几何体的 origin 偏移与 joint 的坐标系不匹配。  
**解决**：
- 调整 `<visual>` 和 `<collision>` 的 `<origin>` 旋转（`rpy="0 1.5708 0"`）
- 确保 cylinder 沿关节连接方向对齐

### 问题 5：RViz 卡死 — 仿真时间与系统时间冲突
**现象**：启动 MoveIt + RViz 后大量 `Detected jump back in time` 警告，界面完全冻结。  
**原因**：`robot_state_publisher` 使用系统墙上时钟发布 TF，但 MoveIt 设置了 `use_sim_time: True`。  
**解决**：
- 所有节点统一使用 `use_sim_time: True`
- 启动前执行 `ros2 daemon stop` 清除 ROS2 daemon 缓存的旧 TF 数据

### 问题 6：MoveIt 规划失败 — START_STATE_IN_COLLISION
**现象**：点击 Plan & Execute 报错 `1 contact(s) detected: link_2 - link_4`。  
**原因**：URDF 中 link_2 和 link_4 的碰撞体积在 home 位置重叠，SRDF 碰撞矩阵未忽略此对。  
**解决**：
- 在 SRDF 的 `<disable_collisions>` 中添加所有不应检测碰撞的 link 对
- 不仅相邻 link，间隔一个的 link 对也需添加（link_2-link_4, link_3-link_5 等）

### 问题 7：MoveIt 执行中止 — 起始状态容差过严
**现象**：规划成功但执行被拒绝，`MoveGroupInterface::move() failed or timeout reached`。  
**原因**：`allowed_start_tolerance: 0.01` 太严格，仿真中的微小关节偏差导致起始状态检查失败。  
**解决**：
- 设置 `allowed_start_tolerance: 0.0` 禁用起始状态检查
- 增大 `allowed_execution_duration_scaling` 和 `allowed_goal_duration_margin`

---

## 三、关键经验总结

1. **VMware 上的 Gazebo**：始终分离 physics server 和 GUI，物理引擎可靠运行于 headless 模式
2. **时间同步**：使用仿真时钟时，**所有** ROS2 节点必须设置 `use_sim_time: True`
3. **碰撞矩阵**：SRDF 中的 `disable_collisions` 必须覆盖所有正常工作范围内可能重叠的 link 对
4. **启动顺序**：Gazebo → 等控制器就绪 → MoveIt → 障碍物节点，顺序不可打乱
5. **清理缓存**：重启仿真前务必 `ros2 daemon stop`，避免旧数据导致时间跳跃
