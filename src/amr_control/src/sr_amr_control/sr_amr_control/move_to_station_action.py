import time

from action_msgs.srv._cancel_goal import CancelGoal_Request
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from sros_sdk_py.client import SrpClient
from sros_sdk_py.main_pb2 import MovementTask, SystemState, TaskResult
from sros_sdk_py.srp import RequestFailedError

from sr_amr_interfaces.action import MoveToStation

from .utils.internal_state import InternalState


class MoveToStationAction:
  _node: Node
  _srp_client: SrpClient
  _action_server: ActionServer
  _internal_state: InternalState

  def __init__(self, node: Node, srp_client: SrpClient, internal_state: InternalState):
    self._node = node
    self._srp_client = srp_client
    self._action_server = ActionServer(
      self._node,
      MoveToStation,
      'move_to_station',
      self._execute,
      goal_callback=self._handle_goal,
      cancel_callback=self._handle_cancel,
    )
    self._internal_state = internal_state

  def _handle_goal(self, goal_handle: MoveToStation.Goal):
    station_id = goal_handle.station_id
    if station_id <= 0:
      self._node.get_logger().info(
        f'Received a invalid station ID: {station_id}',
      )
      return GoalResponse.REJECT
    else:
      self._node.get_logger().info(
        f'Received goal request with station ID: {station_id}'
      )
      return GoalResponse.ACCEPT

  def _handle_cancel(self, cancel_request: CancelGoal_Request):
    self._node.get_logger().info('Received request to cancel goal')
    return CancelResponse.ACCEPT

  def _execute(self, goal_handle: ServerGoalHandle):
    goal: MoveToStation.Goal = goal_handle.request

    feedback = MoveToStation.Feedback()
    result = MoveToStation.Result()

    movement_no = self._internal_state.generate_movement_no()
    station_id = goal.station_id

    self._node.get_logger().info(
      f'Starting movement task {movement_no}, target station {station_id}'
    )

    try:
      self._srp_client.move_to_station(movement_no, station_id)
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Movement task {movement_no} to station {station_id} failed to start, error code: {e.result_code}'  # noqa: E501
      )
      result.result = False
      result.error_code = str(e.result_code)
      goal_handle.abort()
      return result

    feedback.cur_path_no = 0
    feedback.obstacle_paused = False
    feedback.manual_paused = False

    while True:
      if goal_handle.is_cancel_requested:
        self._node.get_logger().info(f'Cancelling movement task {movement_no}')
        try:
          self._srp_client.cancel_movement_task()
          self._node.get_logger().info(f'Movement task {movement_no} cancelled')
        except RequestFailedError as e:
          self._node.get_logger().error(
            f'Failed to cancel movement task {movement_no}, error code: {e.result_code}'  # noqa: E501
          )
        goal_handle.canceled()
        result.result = False
        result.error_code = 'Canceled by client'
        break
      self._srp_client.fetch_system_state()
      time.sleep(0.1)
      system_state = self._srp_client.get_current_system_state()
      if system_state and system_state.HasField('movement_state'):
        movement_state = system_state.movement_state
        if (
          movement_state.state == MovementTask.TaskState.MT_FINISHED
          and movement_state.no == movement_no
        ):
          if movement_state.result == TaskResult.TASK_RESULT_OK:
            result.result = True
            goal_handle.succeed()
            self._node.get_logger().info(
              f'Movement task {movement_no} succeed, target station -> {goal.station_id}'  # noqa: E501
            )
            break
          else:
            result.result = False
            error_code = movement_state.failed_code
            self._node.get_logger().error(
              f'Movement task {movement_no} failed, target station -> {goal.station_id}, error code: {error_code}'  # noqa: E501
            )
            result.error_code = str(error_code)
            goal_handle.abort()
            break
        else:
          obstacle_paused = (
            system_state.sys_state == SystemState.SYS_STATE_TASK_NAV_PAUSED
          )
          manual_paused = (
            system_state.sys_state == SystemState.SYS_STATE_TASK_MANUAL_PAUSED
          )
          if (
            obstacle_paused != feedback.obstacle_paused
            or movement_state.cur_path_no != feedback.cur_path_no
            or manual_paused != feedback.manual_paused
          ):
            feedback.cur_path_no = movement_state.cur_path_no
            feedback.obstacle_paused = obstacle_paused
            feedback.manual_paused = manual_paused
            goal_handle.publish_feedback(feedback)
    return result
