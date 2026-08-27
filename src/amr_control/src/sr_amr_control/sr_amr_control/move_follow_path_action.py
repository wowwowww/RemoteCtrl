import math
import time
from typing import List

from action_msgs.srv._cancel_goal import CancelGoal_Request
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from sros_sdk_py import SrpClient
from sros_sdk_py.main_pb2 import MovementTask, SystemState, TaskResult
from sros_sdk_py.path import Path
from sros_sdk_py.srp import RequestFailedError

from sr_amr_interfaces.action import MoveFollowPath
from sr_amr_interfaces.msg import Path as MsgPath

from .utils.internal_state import InternalState
from .utils.ros_bridge import ros_to_srp_unit


class MoveFollowPathAction:
  _node: Node
  _srp_client: SrpClient
  _state: InternalState

  def __init__(self, node: Node, srp_client: SrpClient, state: InternalState):
    self._node = node
    self._srp_client = srp_client
    self._state = state
    self._action_server = ActionServer(
      self._node,
      MoveFollowPath,
      'move_follow_path',
      self._execute,
      goal_callback=self._handle_goal,
      cancel_callback=self._handle_cancel,
    )

  def _handle_goal(self, goal_handle: MoveFollowPath.Goal):
    self._node.get_logger().info(f'Received -> {len(goal_handle.paths)} paths')
    if goal_handle.paths:
      return GoalResponse.ACCEPT
    else:
      return GoalResponse.REJECT

  def _handle_cancel(self, cancel_request: CancelGoal_Request):
    self._node.get_logger().info('Received request to cancel goal')
    return CancelResponse.ACCEPT

  def _get_paths_from_request(self, request: MoveFollowPath.Goal) -> List[Path]:
    paths = []
    for msg_p in request.paths:
      if msg_p.type == MsgPath.PATH_LINE:
        path = Path.create_line_path(
          int(ros_to_srp_unit(msg_p.sx)),
          int(ros_to_srp_unit(msg_p.sy)),
          int(ros_to_srp_unit(msg_p.ex)),
          int(ros_to_srp_unit(msg_p.ey)),
          direction=msg_p.direction,
          orientation=int(math.radians(msg_p.orientation) * 1000),
          limit_v=int(msg_p.limit_v * 1000),
        )
      elif msg_p.type == MsgPath.PATH_BEZIER:
        path = Path.create_bezier_path(
          int(ros_to_srp_unit(msg_p.sx)),
          int(ros_to_srp_unit(msg_p.sy)),
          int(ros_to_srp_unit(msg_p.cx)),
          int(ros_to_srp_unit(msg_p.cy)),
          int(ros_to_srp_unit(msg_p.dx)),
          int(ros_to_srp_unit(msg_p.dy)),
          int(ros_to_srp_unit(msg_p.ex)),
          int(ros_to_srp_unit(msg_p.ey)),
          direction=msg_p.direction,
          limit_v=int(ros_to_srp_unit(msg_p.limit_v)),
        )
      elif msg_p.type == MsgPath.PATH_ROTATE:
        path = Path.create_rotate_path(
          int(ros_to_srp_unit(math.radians(msg_p.rotate_angle))),
          int(ros_to_srp_unit(math.radians(msg_p.limit_w))),
        )
      else:
        assert False, f'Unsupported path type: {msg_p.type}'

      paths.append(path)
    return paths

  def _execute(self, goal_handle: ServerGoalHandle) -> MoveFollowPath.Result:
    feedback = MoveFollowPath.Feedback()
    result = MoveFollowPath.Result()

    movement_no = self._state.generate_movement_no()
    self._node.get_logger().info(f'Starting movement task -> {movement_no}')
    paths = self._get_paths_from_request(goal_handle.request)

    try:
      self._srp_client.move_follow_path(movement_no, paths)
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Movement task {movement_no} failed to start, error code: {e.result_code}'  # noqa: E501
      )
      result.result = False
      result.error_code = str(e.result_code)
      goal_handle.abort()
      return result

    feedback.cur_path_no = 0
    feedback.obstacle_paused = False

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
            self._node.get_logger().info(f'Movement task {movement_no} succeed')
            break
          else:
            result.result = False
            error_code = movement_state.failed_code
            self._node.get_logger().error(
              f'Movement task {movement_no} failed, error code: {error_code}'
            )
            result.error_code = str(error_code)
            goal_handle.abort()
            break
        else:
          obstacle_paused = (
            system_state.sys_state == SystemState.SYS_STATE_TASK_PATH_PAUSED
          )
          if (
            obstacle_paused != feedback.obstacle_paused
            or movement_state.cur_path_no != feedback.cur_path_no
          ):
            feedback.cur_path_no = movement_state.cur_path_no
            feedback.obstacle_paused = obstacle_paused
            goal_handle.publish_feedback(feedback)
    return result
