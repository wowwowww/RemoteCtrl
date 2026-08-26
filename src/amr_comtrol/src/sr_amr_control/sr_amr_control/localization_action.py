from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from sros_sdk_py.client import SrpClient
from sros_sdk_py.main_pb2 import SystemState
from sros_sdk_py.srp import RequestFailedError

from sr_amr_interfaces.action import (
  LocateByPose,
  LocateByStation,
  SetMapWithInitialPose,
)

from .utils.ros_bridge import ros_to_srp_unit, yaw_from_quaternion

_DEFAULT_TIMEOUT_SEC = 60.0


class LocalizationActionServer:
  _node: Node
  _srp_client: SrpClient

  def __init__(self, node: Node, srp_client: SrpClient):
    self._node = node
    self._srp_client = srp_client

    self._locate_by_pose_server = ActionServer(
      self._node,
      LocateByPose,
      'locate_by_pose',
      self._execute_locate_by_pose,
      goal_callback=self._handle_goal,
      cancel_callback=self._handle_cancel,
    )

    self._locate_by_station_server = ActionServer(
      self._node,
      LocateByStation,
      'locate_by_station',
      self._execute_locate_by_station,
      goal_callback=self._handle_goal,
      cancel_callback=self._handle_cancel,
    )

    self._set_map_with_pose_server = ActionServer(
      self._node,
      SetMapWithInitialPose,
      'set_map_with_initial_pose',
      self._execute_set_map_with_initial_pose,
      goal_callback=self._handle_goal,
      cancel_callback=self._handle_cancel,
    )

  def _handle_goal(self, goal_request):
    del goal_request
    return GoalResponse.ACCEPT

  def _handle_cancel(self, cancel_request):
    del cancel_request
    return CancelResponse.ACCEPT

  @staticmethod
  def _timeout_or_default(timeout_sec: float) -> float:
    return timeout_sec if timeout_sec > 0 else _DEFAULT_TIMEOUT_SEC

  def is_already_locating_or_located(self) -> bool:
    system_state: SystemState = self._srp_client.fetch_system_state()  # type: ignore[assignment]
    return (
      system_state.location_state == SystemState.LocationState.LOCATION_STATE_RUNNING
      or system_state.location_state
      == SystemState.LocationState.LOCATION_STATE_INITIALING
    )

  def _execute_locate_by_pose(
    self,
    goal_handle: ServerGoalHandle,
  ):
    goal: LocateByPose.Goal = goal_handle.request
    result = LocateByPose.Result()
    feedback = LocateByPose.Feedback()

    if goal_handle.is_cancel_requested:
      goal_handle.canceled()
      result.success = False
      result.message = 'Canceled by client'
      return result

    if self.is_already_locating_or_located():
      result.success = False
      result.message = 'Robot is already performing localization or has been localized'
      goal_handle.abort()
      return result

    x = ros_to_srp_unit(goal.initial_pose.position.x)
    y = ros_to_srp_unit(goal.initial_pose.position.y)
    yaw = yaw_from_quaternion(goal.initial_pose.orientation)
    timeout_sec = self._timeout_or_default(goal.timeout_sec)

    message = (
      f'Locating by pose: x={goal.initial_pose.position.x:.3f}m, '
      f'y={goal.initial_pose.position.y:.3f}m, yaw={yaw:.3f}rad, timeout={timeout_sec:.1f}s'  # noqa: E501
    )

    self._node.get_logger().info(message)
    feedback.feedback_message = message
    goal_handle.publish_feedback(feedback)

    try:
      ok = self._srp_client.locate_by_pose(
        x=x,
        y=y,
        yaw=yaw,
        force_set_initial_pose=goal.force_set_initial_pose,
        timeout_sec=timeout_sec,
      )
      if ok:
        result.success = True
        result.message = 'Localization by pose succeeded'
        goal_handle.succeed()
      else:
        result.success = False
        result.message = 'Localization by pose failed'
        goal_handle.abort()
    except RequestFailedError as e:
      result.success = False
      result.message = str(e.result_code)
      goal_handle.abort()
    except Exception as e:
      result.success = False
      result.message = str(e)
      goal_handle.abort()

    return result

  def _execute_locate_by_station(
    self,
    goal_handle: ServerGoalHandle,
  ):
    goal = goal_handle.request
    result = LocateByStation.Result()
    feedback = LocateByStation.Feedback()

    if goal.station_id <= 0:
      result.success = False
      result.message = 'station_id must be greater than 0'
      goal_handle.abort()
      return result

    if goal_handle.is_cancel_requested:
      goal_handle.canceled()
      result.success = False
      result.message = 'Canceled by client'
      return result

    timeout_sec = self._timeout_or_default(goal.timeout_sec)
    feedback.feedback_message = (
      f'Locating by station: station_id={goal.station_id}, timeout={timeout_sec:.1f}s'
    )
    goal_handle.publish_feedback(feedback)

    if self.is_already_locating_or_located():
      result.success = False
      result.message = 'Robot is already performing localization or has been localized'
      goal_handle.abort()
      return result

    try:
      ok = self._srp_client.locate_by_station(  # type: ignore[attr-defined]
        station_id=int(goal.station_id),
        force_locate=goal.force_locate,
        timeout_sec=timeout_sec,
      )
      if ok:
        result.success = True
        result.message = 'Localization by station succeeded'
        goal_handle.succeed()
      else:
        result.success = False
        result.message = 'Localization by station failed'
        goal_handle.abort()
    except RequestFailedError as e:
      result.success = False
      result.message = str(e.result_code)
      goal_handle.abort()
    except Exception as e:
      result.success = False
      result.message = str(e)
      goal_handle.abort()

    return result

  def _execute_set_map_with_initial_pose(
    self,
    goal_handle: ServerGoalHandle,
  ):
    goal = goal_handle.request
    result = SetMapWithInitialPose.Result()
    feedback = SetMapWithInitialPose.Feedback()

    map_name = goal.map_name.strip()
    if map_name == '':
      result.success = False
      result.message = 'map_name cannot be empty'
      goal_handle.abort()
      return result

    if goal_handle.is_cancel_requested:
      goal_handle.canceled()
      result.success = False
      result.message = 'Canceled by client'
      return result

    x = float(ros_to_srp_unit(goal.initial_pose.position.x))
    y = float(ros_to_srp_unit(goal.initial_pose.position.y))
    yaw = float(yaw_from_quaternion(goal.initial_pose.orientation))
    timeout_sec = self._timeout_or_default(goal.timeout_sec)

    feedback.feedback_message = (
      f'Switching map to {map_name}, timeout={timeout_sec:.1f}s'
    )
    goal_handle.publish_feedback(feedback)

    try:
      ok = self._srp_client.set_map_with_initial_pose(
        map_name=map_name,
        x=x,
        y=y,
        yaw=yaw,
        force_set_initial_pose=goal.force_set_initial_pose,
        timeout_sec=timeout_sec,
      )
      if ok:
        result.success = True
        result.message = f'Map switched to {map_name}'
        goal_handle.succeed()
      else:
        result.success = False
        result.message = f'SetMapWithInitialPose failed for map {map_name}'
        goal_handle.abort()
    except RequestFailedError as e:
      result.success = False
      result.message = str(e.result_code)
      goal_handle.abort()
    except Exception as e:
      result.success = False
      result.message = str(e)
      goal_handle.abort()

    return result
