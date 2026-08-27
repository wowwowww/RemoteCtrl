import time
from typing import Optional

from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sros_sdk_py import SrpClient
from sros_sdk_py.main_pb2 import SystemState
from sros_sdk_py.srp import RequestFailedError
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from .utils.ros_bridge import ros_to_srp_unit


class RemoteControlStateChecker:
  _srp_client: SrpClient

  def __init__(self, client: SrpClient):
    self._srp_client = client

  def get_remote_control_status(self) -> Optional[bool]:
    system_state = self._srp_client.get_current_system_state()
    if system_state is None:
      return None
    return system_state.operation_state == SystemState.OperationState.OPERATION_MANUAL

  def get_remote_control_oba_status(self) -> Optional[bool]:
    system_state = self._srp_client._system_state
    if system_state is None:
      return None
    return system_state.manual_control_oba_state

  def check_remote_control_status(self, enabled: bool) -> bool:
    status = self.get_remote_control_status()
    if status is None:
      return False
    return status == enabled

  def check_remote_control_oba_status(self, enabled: bool) -> bool:
    status = self.get_remote_control_oba_status()
    if status is None:
      return False
    return status == enabled


class RemoteController:
  _node: Node
  _srp_client: SrpClient
  _TIMEOUT_SEC = 5

  _service_group = MutuallyExclusiveCallbackGroup()
  _topic_group = MutuallyExclusiveCallbackGroup()

  _state_checker: RemoteControlStateChecker

  _remote_control_enabled: bool = False

  def __init__(self, node: Node, srp_client: SrpClient):
    self._node = node
    self._srp_client = srp_client
    self._state_checker = RemoteControlStateChecker(srp_client)

    self._button_topic = node.declare_parameter(
      'button_topic', '/rc_ctrl/button'
    ).value
    self._joy_deadzone = float(node.declare_parameter('joy_deadzone', 0.2).value)
    self._max_linear_speed = float(
      node.declare_parameter('joy_max_linear_speed', 0.5).value
    )
    self._max_angular_speed = float(
      node.declare_parameter('joy_max_angular_speed', 0.8).value
    )
    self._estop_button = int(node.declare_parameter('estop_button', 0).value)
    self._estop_release_button = int(
      node.declare_parameter('estop_release_button', 1).value
    )

    self._estop_button_prev = False
    self._estop_release_button_prev = False

    self._remote_control_enabled_srv = node.create_service(
      SetBool,
      'remote_control_enabled',
      self._handle_remote_control_enabled,
      callback_group=self._service_group,
    )

    self._remote_control_oba_enabled_srv = node.create_service(
      SetBool,
      'remote_control_oba_enabled',
      self._handle_remote_control_oba_enabled,
      callback_group=self._service_group,
    )

    self._cmd_vel_subscriber = node.create_subscription(
      Twist,
      'remote_control_cmd_vel',
      self._handle_cmd_vel,
      10,
      callback_group=self._topic_group,
    )

    self._button_subscriber = node.create_subscription(
      Joy,
      self._button_topic,
      self._handle_button,
      10,
      callback_group=self._topic_group,
    )

  def _joy_to_twist(self, msg: Joy) -> Twist:
    twist = Twist()
    if len(msg.axes) < 6:
      return twist

    left_js_x = float(msg.axes[4])
    left_js_y = float(msg.axes[5])
    dominant = max(abs(left_js_x), abs(left_js_y))
    if dominant <= self._joy_deadzone:
      return twist

    span = max(1e-6, 1.0 - self._joy_deadzone)
    scale = min(1.0, max(0.0, (dominant - self._joy_deadzone) / span))

    if abs(left_js_y) >= abs(left_js_x):
      twist.linear.x = scale * self._max_linear_speed * (
        1.0 if left_js_y >= 0.0 else -1.0
      )
    else:
      twist.angular.z = scale * self._max_angular_speed * (
        -1.0 if left_js_x >= 0.0 else 1.0
      )

    return twist

  def _button_pressed(self, msg: Joy, index: int) -> bool:
    return index < len(msg.buttons) and bool(msg.buttons[index])

  def _handle_estop_buttons(self, msg: Joy):
    estop_pressed = self._button_pressed(msg, self._estop_button)
    release_pressed = self._button_pressed(msg, self._estop_release_button)

    if estop_pressed and not self._estop_button_prev:
      self._trigger_emergency_stop()
    if release_pressed and not self._estop_release_button_prev:
      self._release_emergency_stop_and_enable()

    self._estop_button_prev = estop_pressed
    self._estop_release_button_prev = release_pressed

  def _trigger_emergency_stop(self):
    self._remote_control_enabled = False
    # 急停前先退出遥控，使 operation_state 切离 OPERATION_MANUAL。急停本身不会
    # 改变 operation_state；若保持 MANUAL，Y 解除后重新 enable_manual_control 会
    # 命中 070006（ALREADY_IN_MANUAL_CONTROL）被当作失败，速度门无法重新打开。
    # 未处于手动模式时退出遥控会返回 070001，属正常情况，忽略即可。
    try:
      self._srp_client.set_remote_control(False)
    except RequestFailedError as e:
      self._node.get_logger().warning(
        f'Exit remote control before emergency stop failed, error_code: {e.result_code}'
      )
    try:
      self._srp_client.emergency_stop()
      self._node.get_logger().info('Emergency stop triggered by button')
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Emergency stop failed, error_code: {e.result_code}'
      )

  def _release_emergency_stop_and_enable(self):
    try:
      self._srp_client.release_emergency_stop()
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Release emergency stop failed, error_code: {e.result_code}'
      )
      return

    if not self._wait_until_emergency_released():
      return

    self._set_remote_control(True)

  def _is_emergency_stopped(self):
    state = self._srp_client.get_current_system_state()
    if state is None:
      return None
    es_state = state.emergency_state
    State = SystemState.EmergencyState
    return (
      es_state != State.STATE_EMERGENCY_NA
      and es_state != State.STATE_EMERGENCY_NONE
    )

  def _wait_until_emergency_released(self, timeout: float = 5.0) -> bool:
    # RECOVERABLE(3) 表示急停触源已释放、可恢复，但 cancel_emergency 之后从
    # RECOVERABLE 清到 NONE(1) 需要电机/安全系统重新初始化，耗时可能超过 2 秒，
    # 因此这里放宽到 5 秒（与 _TIMEOUT_SEC 一致），避免误判为解除失败。
    start_time = time.monotonic()
    last_emergency_state = None
    while time.monotonic() - start_time < timeout:
      state = self._srp_client.get_current_system_state()
      if state is not None:
        last_emergency_state = state.emergency_state
      if self._is_emergency_stopped() is False:
        return True
      time.sleep(0.1)
    self._node.get_logger().error(
      'Timeout waiting for emergency stop to clear, '
      f'emergency_state stayed at {last_emergency_state}'
    )
    return False

  def _handle_button(self, msg: Joy):
    self._handle_estop_buttons(msg)
    self._handle_cmd_vel(self._joy_to_twist(msg))

  def _set_remote_control(self, target_enable: bool):
    try:
      self._srp_client.set_remote_control(target_enable)
      if target_enable:
        # 手动遥控时禁用避障（OBA），避免避障拦截速度指令
        self._srp_client.set_remote_control_oba(False)
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Remote control srv called failed, error_code: {e.result_code}'
      )
      return False, f'{e.result_code}'

    start_time = time.monotonic()

    while time.monotonic() - start_time < self._TIMEOUT_SEC:
      time.sleep(0.1)
      if self._state_checker.check_remote_control_status(target_enable):
        self._remote_control_enabled = target_enable
        self._node.get_logger().info(
          f'Remote control state transitioned to {target_enable} successfully.'
        )
        return True, f'Remote control successfully set to {target_enable}.'

    message = f'Timeout ({self._TIMEOUT_SEC}s) while waiting for remote control state to switch to {target_enable}'  # noqa: E501
    self._node.get_logger().error(f'Remote control command FAILED: {message}')

    return False, message

  def _handle_remote_control_enabled(
    self, request: SetBool.Request, response: SetBool.Response
  ):
    response.success, response.message = self._set_remote_control(request.data)
    return response

  def _handle_remote_control_oba_enabled(
    self, request: SetBool.Request, response: SetBool.Response
  ):
    target_enable = request.data

    try:
      self._srp_client.set_remote_control_oba(target_enable)
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Remote control oba srv called failed, error_code: {e.result_code}'
      )
      response.success = False
      response.message = f'{e.result_code}'
      return response

    start_time = time.monotonic()

    while time.monotonic() - start_time < self._TIMEOUT_SEC:
      time.sleep(0.1)
      if self._state_checker.check_remote_control_oba_status(target_enable):
        response.success = True
        response.message = f'Remote control oba successfully set to {target_enable}.'
        self._node.get_logger().info(
          f'Remote control oba state transitioned to {target_enable} successfully.'
        )
        return response

    response.success = False
    response.message = f'Timeout ({self._TIMEOUT_SEC}s) while waiting for remote control oba state to switch to {target_enable}'  # noqa: E501
    self._node.get_logger().error(
      f'Remote control oba command FAILED: {response.message}'
    )

    return response

  def _handle_cmd_vel(self, msg: Twist):
    if not self._remote_control_enabled:
      return

    linear_x = msg.linear.x  # m/s
    linear_y = msg.linear.y  # m/s
    angular_z = msg.angular.z  # rad/s
    try:
      self._srp_client.set_remote_control_speed(
        int(round(ros_to_srp_unit(linear_x))),
        int(round(ros_to_srp_unit(linear_y))),
        int(round(ros_to_srp_unit(angular_z))),
      )
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Set remote control speed failed, error_code: {e.result_code}'
      )
