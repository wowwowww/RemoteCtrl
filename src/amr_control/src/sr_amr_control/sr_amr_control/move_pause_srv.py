import time
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.service import Service
from sros_sdk_py.client import SrpClient
from sros_sdk_py.main_pb2 import MovementTask
from sros_sdk_py.srp import RequestFailedError
from std_srvs.srv import Trigger

from .state_publisher import MovementTaskStateCtrl


class MovePauseSrv:
  _node: Node
  _srp_client: SrpClient
  _callback_group = ReentrantCallbackGroup()
  _pause_srv: Service
  _resume_srv: Service

  def __init__(self, node: Node, srp_client: SrpClient):
    self._node = node
    self._srp_client = srp_client
    self._srv = self._node.create_service(
      Trigger,
      'move_pause',
      self._handle_pause_request,
      callback_group=self._callback_group,
    )
    self._srv = self._node.create_service(
      Trigger,
      'move_resume',
      self._handle_resume_request,
      callback_group=self._callback_group,
    )

  def _to_timeout(self, resp: Trigger.Response, symbol: str) -> Trigger.Response:
    resp.success = False
    resp.message = f'Move {symbol} failed, reason: timeout waiting for {symbol}'
    return resp

  def _handle_pause_request(self, request: Trigger.Request, response: Trigger.Response):
    self._node.get_logger().info('Received MovePause service request')

    should_execute = self._should_execute_cmd()

    self._node.get_logger().info(f'should_execute={should_execute}')

    if should_execute is None:
      return self._to_timeout(response, 'pause')

    if not should_execute:
      response.success = False
      response.message = 'Move paused failed (no movement to pause)'
      return response

    self._srp_client.move_pause()

    success = self._wait_for_state(expect_paused=True)

    if success is None:
      return self._to_timeout(response, 'pause')

    response.success = success
    response.message = (
      'Move paused successful'
      if success
      else 'Move paused failed, reason: "unknown error"'
    )
    return response

  def _handle_resume_request(
    self, request: Trigger.Request, response: Trigger.Response
  ):
    self._node.get_logger().info('Received MoveResume service request')

    should_execute = self._should_execute_cmd()

    if should_execute is None:
      return self._to_timeout(response, 'resume')

    if not should_execute:
      response.success = False
      response.message = 'Move resume failed (no movement to resume)'
      return response

    try:
      self._srp_client.move_resume()
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Move paused srv called failed, error_code: {e.result_code}'
      )
      response.success = False
      response.message = f'{e.result_code}'
      return response

    success = self._wait_for_state(expect_paused=False)

    if success is None:
      return self._to_timeout(response, 'resume')

    response.success = success
    response.message = (
      'Move resumed successful' if success else 'Move resumed failed, "unknown error"'
    )
    return response

  def _should_execute_cmd(self, timeout: float = 2.0) -> Optional[bool]:
    start_time = time.time()
    while (time.time() - start_time) < timeout:
      system_state = self._srp_client.get_current_system_state()
      if not system_state:
        time.sleep(0.1)
        continue
      if not system_state.HasField('movement_state'):
        return False

      movement_state = system_state.movement_state
      return MovementTaskStateCtrl.is_moving(movement_state)
    return None

  def _wait_for_state(
    self, expect_paused: bool, timeout: float = 2.0
  ) -> Optional[bool]:
    start_time = time.time()
    while (time.time() - start_time) < timeout:
      system_state = self._srp_client.get_current_system_state()
      if not (system_state and system_state.HasField('movement_state')):
        time.sleep(0.1)
        continue

      current_state = system_state.movement_state.state
      is_paused = current_state == MovementTask.TaskState.MT_PAUSED

      if expect_paused and is_paused:
        return True
      elif not expect_paused and not is_paused:
        return True

      time.sleep(0.1)

    return None
