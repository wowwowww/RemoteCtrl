推荐使用双臂启动文件：

```bash
cd /usr/robot_ws/RemoteCtrl
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch jaka_driver dual_arm_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 \
  gripper_force:=50 \
  gripper_speed:=50 \
  gripper_initialize:=true
```

其中：

- `gripper_rs485_channel:=1`：TIO RS485 通道 2（RS485L），当前默认值
- `gripper_force`：夹爪力度，范围 `20~100`
- `gripper_speed`：夹爪速度，范围 `1~100`
- `gripper_initialize:=true`：启动时执行夹爪初始化，可能产生运动

如果 JAKA App 已经配置好 TIO，可跳过 SDK 的 TIO 引脚配置：

```bash
ros2 launch jaka_driver dual_arm_teleop.launch.py \
  gripper_rs485_channel:=1 \
  gripper_configure_tio:=false \
  gripper_force:=50 \
  gripper_speed:=50
```

如果实际接线是 TIO RS485H，则改为：

```bash
ros2 launch jaka_driver dual_arm_teleop.launch.py \
  gripper_rs485_channel:=0
```

使用数字前端按钮控制夹爪：

```bash
ros2 launch jaka_driver dual_arm_teleop.launch.py \
  gripper_rs485_channel:=1 \
  gripper_use_button:=true
```

默认情况下：

- 左臂使用 `LTr`
- 右臂使用 `RTr`
- 按住按钮闭合
- 松开按钮打开

`dual_arm_teleop.launch.py` 会同时启动 Quest 读取节点和双臂控制节点。