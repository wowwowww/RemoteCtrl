# StandardRobots AMR 控制包

本软件包提供 StandardRobots AMR 平台（也称 SROS，Standard Robots Operating System）底盘控制的 ROS2 节点。

请注意：本包中所有单位均为国际单位制（米、弧度等）。

```bash
colcon build --packages-up-to sr_amr_control
```

构建完成后，请先执行以下命令，再在其他包中使用本节点：

```bash
source install/setup.zsh
```

你可以用以下命令运行 AMR 控制节点：

```bash
# 直接运行节点
ros2 run sr_amr_control control_node --ros-args -p connect_ip:=<IP_ADDRESS>

# 或通过 launch 文件运行，命名空间为 sr_amr_control
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=<IP_ADDRESS>

# 当 map 定位更可靠、速度积分漂移较大时，关闭 odom 积分
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=<IP_ADDRESS> odom_use_integration:=false
```

可选参数：

- frame_id：发布话题和 TF 的机器人本体坐标系（默认："base_link"）
- parent_frame_id：TF 全局父坐标系（默认："map"）
- odom_frame_id：里程计坐标系名称（默认："odom"）
- odom_topic：里程计消息发布的话题名（默认："/odom"）
- odom_use_integration：是否使用 system_state 速度积分生成 odom 位姿（true）或直接跟随 map 位姿（false），默认 true
- lidar_points_frame_id：输入激光 protobuf 点（loc_laser_points）重建前的坐标系，一般为 map（默认："map"）
- lidar：启动时是否启用激光扫描上报（默认：false）
- connect_username：连接 AMR 的用户名（默认值 [MASKED]）
- connect_passwd：连接 AMR 的密码（默认值 [MASKED]）

## TF 行为说明

节点按以下链路发布 TF：

- map -> odom
- odom -> base_link（或你配置的 frame_id）

odom_use_integration 控制 odom -> base_link 的来源：

- true：使用 system_state 中的 linear_velocity_x、linear_velocity_y、angular_velocity 进行积分
- false：跳过积分，直接跟随定位位姿生成 odom -> base_link，同时将 map -> odom 发布为单位变换

补偿机制（当 odom_use_integration=true）：

- 节点先由速度积分得到 odom -> base_link。
- 然后使用 system_state.location_pose（map 下位姿）反算 map -> odom。
- 这样可使 TF 链满足：map->odom * odom->base_link 约等于 location_pose 对应的 map->base_link。
- 因此，map 系下的 base_link 结果会贴近定位值；漂移主要体现在 odom 系下。

注意事项：

- 当定位重定位或跳变时，map -> odom 也会出现相应调整。
- 在 3.33 Hz 更新频率下，仍可能有时间戳与离散更新导致的小误差。

推荐配置：

- 若定位比速度积分更可靠，建议设置 odom_use_integration:=false。

## 消息接口

### Actions

- move_to_station：sr_amr_interfaces/MoveToStation，控制 AMR 移动到指定站点
- move_follow_path：sr_amr_interfaces/MoveFollowPath，控制 AMR 按给定路径运动
- start_charging：sr_amr_interfaces/StartCharging，开始充电（可选充电限制）
- stop_charging：sr_amr_interfaces/StopCharging，停止充电
- locate_by_pose：sr_amr_interfaces/LocateByPose，设置初始位姿并开始定位
- locate_by_station：sr_amr_interfaces/LocateByStation，按站点 ID 进行定位
- set_map_with_initial_pose：sr_amr_interfaces/SetMapWithInitialPose，切换地图并设置初始位姿

### Services

- move_pause：std_srvs/Trigger，暂停当前运动
- move_resume：std_srvs/Trigger，恢复已暂停运动
- remote_control_enabled：std_srvs/SetBool，启用/禁用遥控
- remote_control_oba_enabled：std_srvs/SetBool，启用/禁用遥控避障
- lidar_enabled：std_srvs/SetBool，启用/禁用激光扫描上报
- emergency_stop：std_srvs/Trigger，急停
- release_emergency_stop：std_srvs/Trigger，解除急停状态

### Topics

发布（Publishers）：

- system_state：sr_amr_interfaces/SystemState，3.33hz，发布 AMR 系统状态
- battery_state：sr_amr_interfaces/BatteryState，3.33hz，发布 AMR 电池状态
- /odom：nav_msgs/Odometry，3.33hz，发布与 odom -> base_link TF 对齐的里程计信息
- front/scan：sensor_msgs/LaserScan，启用上报时发布前激光雷达扫描
- rear/scan：sensor_msgs/LaserScan，启用上报时发布后激光雷达扫描

注意：SROS protobuf 的 loc_laser_points 是处理后的障碍点（通常在 map/world），不是原始激光扫描。本节点会将这些点变换到各激光雷达坐标系后，按 atan2 重建为 sensor_msgs/LaserScan。

订阅（Subscribers）：

- remote_control_cmd_vel：geometry_msgs/Twist，订阅遥控速度指令

## 错误码

调用服务或发送 action goal 时，可能会返回错误码。

对于 ROS2 标准服务（如 std_srvs/Trigger、std_srvs/SetBool），当 success 为 False 时，错误码会出现在响应的 message 字段中。

对于 sr_amr_interfaces action，请直接检查 result 字段。
例如：充电相关 action 使用 result/error_code，定位相关 action 使用 success/message。

可参考错误码文件：../resources/error_code.csv

## 许可证

本项目采用 BSD-3-Clause License，详见 ../LICENSE。
