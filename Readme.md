基于ROS2的双JAKA S5-斯坦德机器人遥操软件

源码放置于/RemoteCtrl/src文件夹
遥操作主要使用/jaka_driver、/quest_vr两个文件夹

#项目编译与加载
cd  workplace/RemoteCtrl
colcon build --symlink-install
source ./install/setup.bash

###启动前准备
#控制主机和机械臂之间网络通信正常
左192.168.71.37
右192.168.71.36
#vr头显与主机正常连接
（#动态库文件选择正确）



#启动所有节点（VR发布，机械臂控制）
ros2 launch jaka_driver dual_arm_teleop.launch.py

#操作方式：
按下手柄侧键介入控制，机械臂末端法兰位置与姿态与手柄随动，松开侧键停止控制；
按下扳机键控制夹爪开合，夹爪随扳机按下程度闭合；


#话题列表
/rc_ctrl/left_target
/rc_ctrl/left_target
/rc_ctrl/button



