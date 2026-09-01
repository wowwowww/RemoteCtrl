### 基于ROS2的双JAKA S5-斯坦德机器人遥操软件

源码置于/RemoteCtrl/src文件夹
遥操作主要使用/jaka_driver、/quest_vr、/amr_control、/robot_bringup
sdk_lib_version:2.2.2
robot_left_ip:192.168.71.37
robot_right_ip:192.168.71.36
amr_connect_ip:192.168.71.50


# 头显无线连接（自动）
启动时自动连接无线 adb（use_wifi 默认 true）：
ros2 launch jaka_driver dual_arm_teleop.launch.py

头显无线连接MAC：78-C4-FA-CC-88-23（vr_mac 默认值）。

启动时按 vr_mac 自动从 ARP 表解析 Quest 当前无线 IP 并连接（无需每次手动改 IP）；
日志提示"已成功无线连接 <ip>:5555，可拔掉USB线"后即可拔掉 USB。
无线连接失败自动回退有线；解析失败回退 vr_ip 默认 192.168.1.104。
覆盖方式：MAC 变化用 vr_mac:=<MAC>，网段变化用 vr_subnet:=<网段>（默认 192.168.1.0/24），
关闭自动解析用 vr_mac:=（空串），多设备时用 vr_serial:=<序列号> 指定。
# 手动方式（可选）：
adb tcpip 5555#开启监听端口

#### 项目编译与加载（如果环境没安装完往下查看 ###环境依赖）
cd  workplace/RemoteCtrl
colcon build --symlink-install
source ./install/setup.bash
source venv/bin/activate
export PYTHONPATH=./venv/lib/python3.10/site-packages:$PYTHONPATH


### 启动
## 启动前准备
#控制主机和机械臂之间网络通信正常
左192.168.71.37
右192.168.71.36
底盘192.168.71.50
#vr头显与主机正常连接
（#动态库文件正确）
## VR发布
ros2 launch quest_vr quest_reader.launch.py
## 上半身（包含VR发布，双机械臂控制）
ros2 launch jaka_driver dual_arm_teleop.launch.py
## 单独底盘（遥控需要另外启动VR发布）
ros2 launch sr_amr_control amr_control.launch.py
## 全身启动（这个这个这个这个！......还有这个！！！！）
ros2 launch robot_bringup RC_robot_bringup.launch.py

# 启动参数
use_rpy_ctrl #姿态角遥控，默认false
enable_y_limit #机械臂中间隔离，默认true

# 操作方式：
按下手柄侧键介入控制，机械臂末端法兰位置与姿态与手柄随动，松开侧键停止控制；
按下扳机键控制夹爪开合，夹爪随扳机按下程度闭合；

### 环境依赖
## python库依赖（可安装在本项目手动自建的venv环境下）
在根目录requirements.txt中

## 软件依赖
# adb
 sudo apt update
 sudo apt install adb

## sros_sdk_py手动安装到venv
# 安装 virtualenv
pip3 install --user virtualenv
# 创建虚拟环境
virtualenv venv
# 激活
source venv/bin/activate
# 安装 whl 文件
pip install ./src/amr_control/sros_sdk_py-1.4.1-py3-none-any.whl

### 话题列表
/rc_ctrl/left_target
/rc_ctrl/left_target
/rc_ctrl/button



