import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sros_sdk_py.client import SrpClient
from sros_sdk_py.srp import RequestFailedError

from sr_amr_interfaces.action import StartCharging, StopCharging

# action_id=78: param0=1 → start charge, param0=2 → stop charge
_ACTION_ID_CHARGE = 78
_PARAM0_START = 1
_PARAM0_STOP = 2

# seconds per long-poll cycle; short enough for cancel responsiveness
_POLL_TIMEOUT_SEC = 1.0
_DEFAULT_TIMEOUT_SEC = 60.0


class ChargeActionServer:
  _node: Node
  _srp_client: SrpClient
  _callback_group = ReentrantCallbackGroup()
  _action_no: int = 0

  def __init__(self, node: Node, srp_client: SrpClient):
    self._node = node
    self._srp_client = srp_client
    self._start_server = ActionServer(
      self._node,
      StartCharging,
      'start_charging',
      self._execute_start,
      goal_callback=self._handle_goal,
      cancel_callback=self._handle_cancel,
      callback_group=self._callback_group,
    )
    self._stop_server = ActionServer(
      self._node,
      StopCharging,
      'stop_charging',
      self._execute_stop,
      goal_callback=self._handle_goal,
      cancel_callback=self._handle_cancel,
      callback_group=self._callback_group,
    )

  def _handle_goal(self, goal_handle):
    return GoalResponse.ACCEPT

  def _handle_cancel(self, cancel_handle):
    return CancelResponse.ACCEPT

  def _next_action_no(self) -> int:
    self._action_no += 1
    return self._action_no

  # ------------------------------------------------------------------ #
  # StartCharging                                                        #
  # ------------------------------------------------------------------ #

  def _execute_start(self, goal_handle: ServerGoalHandle) -> StartCharging.Result:
    goal: StartCharging.Goal = goal_handle.request
    result = StartCharging.Result()

    action_no = self._next_action_no()

    # Encode limit into param1: minute limit takes priority over percent limit.
    # param1=0 means "no limit" (charge until full / manually stopped).
    param1 = 0
    if goal.use_minute_limit:
      param1 = int(goal.charging_minute)
    elif goal.use_percent_limit:
      param1 = int(goal.charging_percent)

    self._node.get_logger().info(
      f'StartCharging: action_no={action_no} '
      f'use_minute_limit={goal.use_minute_limit} charging_minute={goal.charging_minute} '  # noqa: E501
      f'use_percent_limit={goal.use_percent_limit} '
      f'charging_percent={goal.charging_percent}'
    )

    try:
      status = self._srp_client.execute_action_task(
        action_no, _ACTION_ID_CHARGE, _PARAM0_START, param1
      )
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'StartCharging action_no={action_no} rejected by SRP, '
        f'error_code={e.result_code}'
      )
      result.result = False
      result.error_code = str(e.result_code)
      goal_handle.abort()
      return result

    revision = status['revision'] if status else 0
    return self._poll_until_done(
      goal_handle, action_no, revision, result, _DEFAULT_TIMEOUT_SEC
    )

  # ------------------------------------------------------------------ #
  # StopCharging                                                          #
  # ------------------------------------------------------------------ #

  def _execute_stop(self, goal_handle: ServerGoalHandle) -> StopCharging.Result:
    result = StopCharging.Result()
    action_no = self._next_action_no()

    self._node.get_logger().info(f'StopCharging: action_no={action_no}')

    try:
      status = self._srp_client.execute_action_task(
        action_no, _ACTION_ID_CHARGE, _PARAM0_STOP, 0
      )
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'StopCharging action_no={action_no} rejected by SRP, error_code={e.result_code} '  # noqa: E501
      )
      result.result = False
      result.error_code = str(e.result_code)
      goal_handle.abort()
      return result

    revision = status['revision'] if status else 0
    return self._poll_until_done(
      goal_handle, action_no, revision, result, _DEFAULT_TIMEOUT_SEC
    )

  # ------------------------------------------------------------------ #
  # Shared long-poll loop                                                #
  # ------------------------------------------------------------------ #

  def _poll_until_done(
    self,
    goal_handle: ServerGoalHandle,
    action_no: int,
    revision: int,
    result,
    timeout_sec: float,
  ):
    deadline = time.monotonic() + timeout_sec
    while True:
      if goal_handle.is_cancel_requested:
        self._node.get_logger().info(f'Charge action_no={action_no} cancel requested')
        try:
          self._srp_client.cancel_action_task()
        except RequestFailedError as e:
          self._node.get_logger().error(
            f'cancel_action_task failed for action_no={action_no}, '
            f'error_code={e.result_code}'
          )
        result.result = False
        result.error_code = 'Canceled by client'
        goal_handle.canceled()
        return result

      update = self._srp_client.wait_action_update(
        action_no, timeout=_POLL_TIMEOUT_SEC, after_revision=revision
      )
      action = update.get('action')

      # timed out this cycle with no change → check overall deadline, then loop
      if update.get('timeout') or action is None:
        if time.monotonic() >= deadline:
          self._node.get_logger().error(
            f'Charge action_no={action_no} timed out after {timeout_sec}s'
          )
          result.result = False
          result.error_code = 'Timeout'
          goal_handle.abort()
          return result
        continue

      revision = action['revision']
      status = action['status']
      self._node.get_logger().debug(f'Charge action_no={action_no} status={status}')

      if status == 'SUCCEEDED':
        result.result = True
        goal_handle.succeed()
        self._node.get_logger().info(f'Charge action_no={action_no} SUCCEEDED')
        return result

      if status in ('FAILED', 'REJECTED'):
        result.result = False
        result.error_code = str(action.get('result_code', ''))
        self._node.get_logger().error(
          f'Charge action_no={action_no} {status}: '
          f'result_code={action.get("result_code")} '
          f'result_str={action.get("result_str")}'
        )
        goal_handle.abort()
        return result

      if status == 'CANCELED':
        result.result = False
        result.error_code = 'Canceled'
        goal_handle.canceled()
        return result
