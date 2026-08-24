"""读取 Quest VR 眼镜位姿并发布为双臂伺服目标。

默认通过 adb logcat 实时读取眼镜 APK 输出的手柄数据（标签由参数 adb_tag
指定），逐行解析左右两个手柄的 4x4 齐次变换矩阵（平移米 + 旋转矩阵），先对
旋转矩阵转置再左乘一个固定正当旋转，对齐手柄↔rpy 的轴（手柄 x->rx, y->ry,
z->rz），再转成四元数并做一阶低通滤波，经「眼镜帧 -> 机器人基帧」的静态 TF
变换后，发布两个 PoseStamped 给 rc_ctrl_node 做绝对伺服跟踪（控制频率 125 Hz）。

若设置了参数 command，则改为从该子进程的 stdout 读取（离线测试用）。
"""

import math
import shlex
import subprocess
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from tf2_geometry_msgs import do_transform_pose_stamped

def parse_buttons(text):
    split_text = text.split(',')
    buttons = {}
    if 'R' in split_text: # right hand if available
        split_text.remove('R') # remove marker
        buttons.update({
                        'A': False,
                        'B': False,
                        'RThU': False, # indicates that right thumb is up from the rest position
                        'RJ': False, # joystick pressed
                        'RG': False, # boolean value for trigger on the grip (delivered by SDK)
                        'RTr': False, # boolean value for trigger on the index finger (delivered by SDK)
                        'rightJS': (0.0, 0.0), # joystick position (x, y) in range (-1.0, 1.0)
                        'rightTrig': 0.0, # trigger value on the index finger in range (0.0, 1.0)
                        'rightGrip': 0.0, # trigger value on the grip in range (0.0, 1.0)
                        })
        # besides following keys are provided:
        # 'rightJS' / 'leftJS' - (x, y) position of joystick. x, y both in range (-1.0, 1.0)
        # 'rightGrip' / 'leftGrip' - float value for trigger on the grip in range (0.0, 1.0)
        # 'rightTrig' / 'leftTrig' - float value for trigger on the index finger in range (0.0, 1.0)

    if 'L' in split_text: # left hand accordingly
        split_text.remove('L') # remove marker
        buttons.update({'X': False,
                        'Y': False, 
                        'LThU': False, 
                        'LJ': False, 
                        'LG': False, 
                        'LTr': False, 
                        'leftJS': (0.0, 0.0), 
                        'leftTrig': 0.0, 
                        'leftGrip': 0.0})
    for key in buttons.keys():
        if key in list(split_text):
            buttons[key] = True
            split_text.remove(key)
    for elem in split_text:
        split_elem = elem.split(' ')
        if len(split_elem) < 2:
            continue
        key = split_elem[0]
        value = tuple([float(x) for x in split_elem[1:]])
        buttons[key] = value
    return buttons

def parse_data(string):
    """解析一帧 stdout 字符串。

    格式：`<transforms>&<buttons>`
      transforms: `hand:16个浮点|hand:16个浮点`（``|`` 分隔左右手柄，16 个
      空白分隔浮点按行主序填成 4x4 齐次矩阵）
      buttons:    `key:0/1&key:0/1`（``1`` 表示按下，先解析备用）

    返回 ({hand: 4x4 ndarray}, {button: bool})。
    """
    transforms = {}
    buttons = {}

    if '&' in string:
        transforms_string, buttons_string = string.split('&', 1)
    else:
        transforms_string, buttons_string = string, ''

    for pair_string in transforms_string.split('|'):
        pair = pair_string.split(':', 1)
        if len(pair) != 2:
            continue
        hand = pair[0].strip()
        values = pair[1].split()

        matrix = np.zeros((4, 4))
        idx = 0
        for val in values:
            if val:
                try:
                    matrix[idx // 4][idx % 4] = float(val)
                    idx += 1
                except ValueError:
                    pass
            if idx == 16:
                break
        if idx == 16:
            transforms[hand] = matrix

    if buttons_string:
        buttons = parse_buttons(buttons_string)
    
    return transforms, buttons


def rot_matrix_to_quaternion(R):
    """3x3 旋转矩阵（行主序） -> 四元数 [w, x, y, z]（Shepperd 稳健法）。"""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [w / norm, x / norm, y / norm, z / norm]


def matrix_to_pose(matrix):
    """4x4 齐次矩阵 -> (平移 3 向量, 四元数 [w,x,y,z])。"""
    init_trans = matrix[0:3, 3]  # 平移（米）
    trans = np.array([-init_trans[2], -init_trans[0], init_trans[1]])

    R = matrix[0:3, 0:3]
    # 对齐手柄 -> 机器人 rpy 轴（目标 手柄 x->rx, y->ry, z->rz）。
    # 实测原始矩阵（不做任何修正）的手柄->rpy「增量」映射是 x->rz, y->-rx, z->ry，
    # 即 rpy = P@h，P = [[0,-1,0],[0,0,1],[1,0,0]]，det(P)=-1（镜像）。链条里存在一次
    # 转置/手性翻转，单靠左乘正当旋转无法表达，故先转置 R.T 再左乘正当旋转 C=-P^T：
    # C@R.T 永远是正当旋转（det=+1），且把增量映射修正为恒等（x->rx, y->ry, z->rz）。
    C = np.array([[0.0, 0.0, -1.0],
                  [1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0]])
    R = C @ R.T
    quat = rot_matrix_to_quaternion(R)
    return trans, quat


def buttons_to_joy(buttons):
    """把 parse_buttons() 的 dict 转成 sensor_msgs/Joy。

    固定 axes 布局（rc_ctrl_node 只用前两项）：
      axes[0]=leftGrip  axes[1]=rightGrip  axes[2]=leftTrig  axes[3]=rightTrig
      axes[4..5]=leftJS(x,y)  axes[6..7]=rightJS(x,y)
    模拟量键可能是浮点 0.0（默认）或 (val,) 元组（解析到值），摇杆是 (x, y) 元组。
    """
    def fv(key):
        v = buttons.get(key, 0.0)
        return float(v[0]) if isinstance(v, (tuple, list)) else float(v)

    def xy(key):
        v = buttons.get(key, (0.0, 0.0))
        return [float(v[0]), float(v[1])] if isinstance(v, (tuple, list)) else [0.0, 0.0]

    def digital(key):
        """Normalize a boolean or numeric tuple to a Joy button state."""
        v = buttons.get(key, False)
        if isinstance(v, (tuple, list)):
            return int(any(abs(float(item)) >= 0.5 for item in v))
        return int(bool(v))

    joy = Joy()
    joy.axes = [fv('leftGrip'), fv('rightGrip'),
                fv('leftTrig'), fv('rightTrig')] + xy('leftJS') + xy('rightJS')
    # Keep a stable digital-button layout for drivers that prefer a front
    # button over the analog trigger axes. The PGI driver can select this
    # layout with gripper_use_button=true and gripper_button_index=N.
    digital_order = [
        'X', 'Y', 'A', 'B', 'LThU', 'RThU',
        'LJ', 'RJ', 'LG', 'RG', 'LTr', 'RTr',
    ]
    joy.buttons = [digital(key) for key in digital_order]
    return joy


class QuestReader(Node):
    def __init__(self):
        super().__init__('quest_reader')

        # 参数
        command = self.declare_parameter('command', '').value
        self.command = shlex.split(command) if command else []
        self.glasses_frame = self.declare_parameter('glasses_frame', 'quest').value
        self.robot_base_frame = self.declare_parameter('robot_base_frame', 'world').value
        self.tf_translation = self.declare_parameter('tf_translation', [0.0, 0.0, 0.0]).value
        self.tf_rotation = self.declare_parameter('tf_rotation', [1.0, 0.0, 0.0, 0.0]).value
        self.left_hand_key = self.declare_parameter('left_hand_key', 'l').value
        self.right_hand_key = self.declare_parameter('right_hand_key', 'r').value
        left_topic = self.declare_parameter('left_target_topic', '/rc_ctrl/left_target').value
        right_topic = self.declare_parameter('right_target_topic', '/rc_ctrl/right_target').value
        button_topic = self.declare_parameter('button_topic', '/rc_ctrl/button').value
        publish_rate = self.declare_parameter('publish_rate', 125.0).value
        # 位姿一阶低通滤波系数（0~1，越大越跟手、越小越平滑；0 表示不过滤）
        self.smooth_alpha = float(self.declare_parameter('smooth_alpha', 0.4).value)
        # adb/logcat 读取参数（未设置 command 时使用）
        self.adb_tag = self.declare_parameter('adb_tag', 'wE9ryARX').value
        self.apk_activity = self.declare_parameter(
            'apk_activity', 'com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity').value
        self.start_apk = self.declare_parameter('start_apk', True).value

        # 发布者
        self.left_pub = self.create_publisher(PoseStamped, left_topic, 5)
        self.right_pub = self.create_publisher(PoseStamped, right_topic, 5)
        self.button_pub = self.create_publisher(Joy, button_topic, 5)

        # TF：广播「眼镜帧 -> 机器人基帧」静态变换，并用 tf2 做动态变换
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcast_static_tf()

        # 共享状态（读取线程写、发布线程读）
        self.lock = threading.Lock()
        self.latest_left = None
        self.latest_right = None
        self.filtered_left = None
        self.filtered_right = None
        self.latest_buttons = {}
        self._running = False
        self.proc = None

        # 启动子进程与读取线程
        self._start_reader()

        # 发布定时器（控制频率 125 Hz）
        self.timer = self.create_timer(1.0 / float(publish_rate), self.publish_callback)

        self.get_logger().info(
            'quest_reader started: glasses=%s -> base=%s, publishing %s (left) / %s (right) at %.1f Hz'
            % (self.glasses_frame, self.robot_base_frame, left_topic, right_topic,
               float(publish_rate)))

    def broadcast_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = self.glasses_frame
        t.transform.translation.x = float(self.tf_translation[0])
        t.transform.translation.y = float(self.tf_translation[1])
        t.transform.translation.z = float(self.tf_translation[2])
        t.transform.rotation.w = float(self.tf_rotation[0])
        t.transform.rotation.x = float(self.tf_rotation[1])
        t.transform.rotation.y = float(self.tf_rotation[2])
        t.transform.rotation.z = float(self.tf_rotation[3])
        self.tf_broadcaster.sendTransform(t)

    def _start_reader(self):
        if self.command:
            self._start_subprocess_reader()
        else:
            self._start_adb_reader()

    def _start_subprocess_reader(self):
        try:
            self.proc = subprocess.Popen(
                self.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1)
        except Exception as e:
            self.get_logger().error('无法启动子进程 %s: %s' % (self.command, e))
            return
        self._running = True
        self.reader_thread = threading.Thread(target=self._subprocess_reader_loop, daemon=True)
        self.reader_thread.start()

    def _start_adb_reader(self):
        if self.start_apk:
            shell_cmd = ('am start -n "%s" -a android.intent.action.MAIN '
                         '-c android.intent.category.LAUNCHER' % self.apk_activity)
            try:
                subprocess.run(['adb', 'shell', shell_cmd], capture_output=True, timeout=10.0)
                self.get_logger().info('已启动 APK: %s' % self.apk_activity)
            except Exception as e:
                self.get_logger().warning('启动 APK 失败: %s' % e)
        try:
            self.proc = subprocess.Popen(
                ['adb', 'logcat', '-T', '0'], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1)
        except Exception as e:
            self.get_logger().error('无法启动 adb logcat: %s' % e)
            return
        self._running = True
        self.reader_thread = threading.Thread(target=self._adb_reader_loop, daemon=True)
        self.reader_thread.start()

    def _subprocess_reader_loop(self):
        while self._running and self.proc is not None and self.proc.poll() is None:
            line = self.proc.stdout.readline()
            if not line:
                break
            self._ingest(line.strip())
        if self.proc is not None:
            rc = self.proc.poll()
            if rc is not None:
                self.get_logger().warning('子进程已退出，返回码 %d' % rc)

    def _adb_reader_loop(self):
        while self._running and self.proc is not None and self.proc.poll() is None:
            line = self.proc.stdout.readline()
            if not line:
                break
            line = line.strip()
            if self.adb_tag not in line:
                continue
            try:
                data = line.split(self.adb_tag + ': ')[1]
            except IndexError:
                continue
            self._ingest(data)
        if self.proc is not None:
            rc = self.proc.poll()
            if rc is not None:
                self.get_logger().warning('adb logcat 已退出，返回码 %d' % rc)

    def _filter_pose(self, prev, new):
        """对 (平移 3 向量, 四元数) 做一阶低通（EMA），抑制手柄跟踪噪声导致的抖动。

        四元数先做符号对齐（q 与 -q 代表同一姿态，避免分量平均退化），滤波后归一化。
        prev 为 None 时直接返回原始值（首帧无历史）。
        """
        if prev is None or self.smooth_alpha <= 0.0:
            return new
        a = self.smooth_alpha
        pt, pq = np.asarray(prev[0], float), np.asarray(prev[1], float)
        nt, nq = np.asarray(new[0], float), np.asarray(new[1], float)
        trans = pt + a * (nt - pt)
        if float(np.dot(pq, nq)) < 0.0:
            nq = -nq
        quat = pq + a * (nq - pq)
        n = float(np.linalg.norm(quat))
        if n > 1e-12:
            quat = quat / n
        return trans, quat

    def _ingest(self, data):
        transforms, buttons = parse_data(data)
        with self.lock:
            if self.left_hand_key in transforms:
                self.filtered_left = self._filter_pose(
                    self.filtered_left, matrix_to_pose(transforms[self.left_hand_key]))
                self.latest_left = self.filtered_left
            if self.right_hand_key in transforms:
                self.filtered_right = self._filter_pose(
                    self.filtered_right, matrix_to_pose(transforms[self.right_hand_key]))
                self.latest_right = self.filtered_right
            if buttons:
                self.latest_buttons = buttons

    def publish_callback(self):
        with self.lock:
            left = self.latest_left
            right = self.latest_right
            buttons = self.latest_buttons
        if left is None and right is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.glasses_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(
                'lookup_transform 失败: %s' % e, throttle_duration_sec=2.0)
            return
        if left is not None:
            self.left_pub.publish(self._to_base(left, transform))
        if right is not None:
            self.right_pub.publish(self._to_base(right, transform))
        if buttons is not None:
            self.button_pub.publish(buttons_to_joy(buttons))

        self.get_logger().debug('左右手位置按钮: left=%s, right=%s, buttons=%s' % (left, right, self.latest_buttons))

    def _to_base(self, pose, transform):
        trans, quat = pose
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.glasses_frame
        ps.pose.position.x = float(trans[0])
        ps.pose.position.y = float(trans[1])
        ps.pose.position.z = float(trans[2])
        ps.pose.orientation.w = quat[0]
        ps.pose.orientation.x = quat[1]
        ps.pose.orientation.y = quat[2]
        ps.pose.orientation.z = quat[3]
        out = do_transform_pose_stamped(ps, transform)
        out.header.frame_id = self.robot_base_frame
        return out

    def shutdown(self):
        self._running = False
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def main(args=None):
    rclpy.init(args=args)
    node = QuestReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
