import time
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.service import Service
from sros_sdk_py.client import SrpClient
from sros_sdk_py.main_pb2 import SystemState
from sros_sdk_py.srp import RequestFailedError
from std_srvs.srv import Trigger


class EStopStateChecker:
  @staticmethod
  def is_emergency_stopped(sys_state: SystemState) -> Optional[bool]:
    es_state = sys_state.emergency_state
    State = SystemState.EmergencyState
    return (
      es_state != State.STATE_EMERGENCY_NA and es_state != State.STATE_EMERGENCY_NONE
    )


class EStopSrv:
  _node: Node
  _srp_client: SrpClient
  _callback_group = ReentrantCallbackGroup()
  _stop_srv: Service
  _clear_stop_srv: Service

  def __init__(self, node: Node, srp_client: SrpClient):
    self._node = node
    self._srp_client = srp_client
    self._srv = self._node.create_service(
      Trigger,
      'emergency_stop',
      self._handle_estop_request,
      callback_group=self._callback_group,
    )
    self._srv = self._node.create_service(
      Trigger,
      'release_emergency_stop',
      self._handle_release_estop_request,
      callback_group=self._callback_group,
    )

  def _to_timeout(self, resp: Trigger.Response, symbol: str) -> Trigger.Response:
    resp.success = False
    resp.message = f'action {symbol} failed, reason: timeout waiting for {symbol}'
    return resp

  def _handle_estop_request(self, request: Trigger.Request, response: Trigger.Response):
    self._node.get_logger().info('Received EmergencyStop service request')

    should_execute = self._should_execute_cmd(True)

    self._node.get_logger().info(f'should_execute={should_execute}')

    if should_execute is None:
      return self._to_timeout(response, 'emergency stop')

    if not should_execute:
      response.success = True
      response.message = 'Emergency stop already active'
      return response

    try:
      self._srp_client.emergency_stop()
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Emergency stop srv called failed, error_code: {e.result_code}'
      )
      response.success = False
      response.message = f'{e.result_code}'
      return response

    success = self._wait_for_state(expect_estop=True)

    if success is None:
      return self._to_timeout(response, 'emergency stop')

    response.success = success
    response.message = (
      'Emergency stop successful'
      if success
      else 'Emergency stop failed, reason: "unknown error"'
    )
    return response

  def _handle_release_estop_request(
    self, request: Trigger.Request, response: Trigger.Response
  ):
    self._node.get_logger().info('Received MoveResume service request')

    should_execute = self._should_execute_cmd(False)

    if should_execute is None:
      return self._to_timeout(response, 'release emergency stop')

    if not should_execute:
      response.success = True
      response.message = 'Release emergency stop succeed (no emergency stop active)'
      return response

    try:
      self._srp_client.release_emergency_stop()
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Release emergency stop srv called failed, error_code: {e.result_code}'
      )
      response.success = False
      response.message = f'{e.result_code}'
      return response

    success = self._wait_for_state(expect_estop=False)

    if success is None:
      return self._to_timeout(response, 'release emergency stop')

    response.success = success
    response.message = (
      'Release emergency stop successful'
      if success
      else 'Release emergency stop failed, "unknown error"'
    )
    return response

  def _should_execute_cmd(self, target: bool, timeout: float = 2.0) -> Optional[bool]:
    start_time = time.time()
    while (time.time() - start_time) < timeout:
      system_state = self._srp_client.get_current_system_state()
      if not system_state:
        time.sleep(0.1)
        continue
      estop_active = EStopStateChecker.is_emergency_stopped(system_state)
      return estop_active != target
    return None

  def _wait_for_state(self, expect_estop: bool, timeout: float = 2.0) -> Optional[bool]:
    start_time = time.time()
    while (time.time() - start_time) < timeout:
      system_state = self._srp_client.get_current_system_state()
      if not system_state:
        time.sleep(0.1)
        continue

      is_emergency_stopped = EStopStateChecker.is_emergency_stopped(system_state)

      if expect_estop and is_emergency_stopped:
        return True
      elif not expect_estop and not is_emergency_stopped:
        return True

      time.sleep(0.1)

    return None
