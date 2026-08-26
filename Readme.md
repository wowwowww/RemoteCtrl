基于ROS2的双JAKA S5-斯坦德机器人遥操软件

源码放置于/RemoteCtrl/src文件夹
遥操作主要使用/jaka_driver、/quest_vr两个文件夹
sdk_lib_version--2.2.2

#头显无线连接（自动）
启动时自动连接无线 adb（use_wifi 默认 true，vr_ip 默认 192.168.1.104）：
ros2 launch jaka_driver dual_arm_teleop.launch.py
# 日志提示"已成功无线连接 192.168.1.104:5555，可拔掉USB线"后即可拔掉 USB；
# 无线连接失败自动回退有线；IP 变化时用 vr_ip:=<Quest无线IP> 覆盖，多设备时用 vr_serial:=<序列号> 指定。
# 手动方式（可选）：
adb tcpip 5555#开启监听端口
头显无线连接MAC：78-C4-FA-CC-88-23 

#项目编译与加载
cd  workplace/RemoteCtrl
colcon build --symlink-install
source ./install/setup.bash

###启动前准备
#控制主机和机械臂之间网络通信正常
左192.168.71.37
右192.168.71.36
#vr头显与主机正常连接
（#动态库文件正确）



#启动所有节点（VR发布，机械臂控制）
ros2 launch jaka_driver dual_arm_teleop.launch.py

#操作方式：
按下手柄侧键介入控制，机械臂末端法兰位置与姿态与手柄随动，松开侧键停止控制；
按下扳机键控制夹爪开合，夹爪随扳机按下程度闭合；


#话题列表
/rc_ctrl/left_target
/rc_ctrl/left_target
/rc_ctrl/button



