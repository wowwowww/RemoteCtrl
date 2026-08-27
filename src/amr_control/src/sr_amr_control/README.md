# StandardRobots AMR Control Package

Chinese documentation: [README-zh.md](./README-zh.md)

This package provides the ROS2 node for controlling the chassis of the StandardRobots AMR platform(Also known as SROS, Standard Robots Operating System).

Please be aware that all of the units used in this package are in metric system (meters, radians, etc.).

```bash
colcon build --packages-up-to sr_amr_control
```

You should source the install/setup.bash after building the package to use the defined nodes in other packages.

```bash
source install/setup.bash
```

You can use the following command to run the AMR control node:

```bash
# Run the node
ros2 run sr_amr_control control_node --ros-args -p connect_ip:=<IP_ADDRESS>
# Or using launch file, namespace `sr_amr_control`
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=<IP_ADDRESS>
# Disable odom integration when map localization is more reliable
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=<IP_ADDRESS> odom_use_integration:=false
```

optional arguments:

- `frame_id`: The frame ID for the published topics and TF (default: "base_link")
- `parent_frame_id`: The parent frame ID for the TF (default: "map")
- `odom_frame_id`: The odometry frame ID (default: "odom")
- `odom_topic`: The published odometry topic name (default: "/odom")
- `odom_use_integration`: Whether to integrate odom pose from `system_state` velocity (`true`) or directly follow map pose (`false`) (default: `true`)
- `lidar_points_frame_id`: Frame ID of incoming lidar protobuf points (`loc_laser_points`) before reconstruction, usually `map`/`world` (default: "map")
- `lidar`: Enable lidar scan reporting at startup (default: `false`)
- `connect_username`: The username for authentication with the AMR, (default value [MASKED])
- `connect_passwd`: The password for authentication with the AMR, (default value [MASKED])

## TF behavior

The node publishes TF in the following chain:

- `map -> odom`
- `odom -> base_link` (or your configured `frame_id`)

`odom_use_integration` controls how `odom -> base_link` is produced:

- `true`: Integrate `linear_velocity_x`, `linear_velocity_y`, and `angular_velocity` from `system_state`.
- `false`: Skip integration and directly follow localization pose for `odom -> base_link`, while publishing `map -> odom` as identity.

Compensation mechanism (when `odom_use_integration=true`):

- The node first integrates velocity to get `odom -> base_link`.
- It then uses `system_state.location_pose` (pose in `map`) to back-calculate `map -> odom`.
- This keeps the chain consistent: `map->odom * odom->base_link` approximately matches `location_pose` (`map->base_link`).
- As a result, `base_link` in the `map` frame stays close to localization, while drift is mainly reflected in the `odom` frame.

Notes:

- If localization jumps (for example during relocalization), `map -> odom` will adjust accordingly.
- At 3.33 Hz update rate, small errors can still appear due to timestamp alignment and discretization.

Recommended setting:

- If your localization is more reliable than velocity integration, use `odom_use_integration:=false`.

## Messages

### Actions

- `move_to_station`: `sr_amr_interfaces/MoveToStation` Move the AMR to a specified station
- `move_follow_path`: `sr_amr_interfaces/MoveFollowPath` Move the AMR following specified path(s)
- `start_charging`: `sr_amr_interfaces/StartCharging` Start charging the AMR with optional limits
- `stop_charging`: `sr_amr_interfaces/StopCharging` Stop charging the AMR
- `locate_by_pose`: `sr_amr_interfaces/LocateByPose` Set initial pose and start localization
- `locate_by_station`: `sr_amr_interfaces/LocateByStation` Localize by station ID
- `set_map_with_initial_pose`: `sr_amr_interfaces/SetMapWithInitialPose` Switch map with initial pose

### Services

- `move_pause`: `std_srvs/Trigger` Pause the current movement
- `move_resume`: `std_srvs/Trigger` Resume the paused movement
- `remote_control_enabled`: `std_srvs/SetBool` Enable/Disable the remote control of the AMR
- `remote_control_oba_enabled`: `std_srvs/SetBool` Enable/Disable the obstacle avoidance of remote controlling the AMR
- `lidar_enabled`: `std_srvs/SetBool` Enable/Disable lidar scan reporting
- `emergency_stop`: `std_srvs/Trigger` Emergency stop the AMR
- `release_emergency_stop`: `std_srvs/Trigger` Release the emergency stop state of the AMR

### Topics

Publishers:

- `system_state`: `sr_amr_interfaces/SystemState` `3.33hz` Publish the system status of the AMR
- `battery_state`: `sr_amr_interfaces/BatteryState` `3.33hz` Publish the battery status of the AMR
- `/odom`: `nav_msgs/Odometry` `3.33hz` Publish odometry aligned with `odom -> base_link` TF
- `front/scan`: `sensor_msgs/LaserScan` Publish the front lidar scan when reporting is enabled.
- `rear/scan`: `sensor_msgs/LaserScan` Publish the rear lidar scan when reporting is enabled.

Note: `loc_laser_points` from SROS protobuf are processed obstacle points (typically in `map`/`world`), not raw lidar scans. This node reconstructs `sensor_msgs/LaserScan` by transforming those points into each lidar frame and binning with `atan2`.

Subscribers:

- `remote_control_cmd_vel`: `geometry_msgs/Twist` Subscribe to the remote control velocity commands

## Examples

actions:

```bash
# Move to a station
ros2 action send_goal /move_to_station sr_amr_interfaces/MoveToStation "{station_id: 1}"
# Move following a path
ros2 action send_goal /move_follow_path sr_amr_interfaces/MoveFollowPath "{paths: [{type: 1, sx: 0, sy: 0, ex: 1, ey: 0}]}"
# Move following a path, and backwards direction
ros2 action send_goal /move_follow_path sr_amr_interfaces/MoveFollowPath "{paths: [{type: 1, sx: 0, sy: 0, ex: 1, ey: 0, direction: 2}]}"
# Move following a path, and orientation follow +90 degrees
ros2 action send_goal /move_follow_path sr_amr_interfaces/MoveFollowPath "{paths: [{type: 1, sx: 0, sy: 0, ex: 1, ey: 0, direction: 16, orientation: 90}]}"
# Rotate in place +90 degrees
ros2 action send_goal /move_follow_path sr_amr_interfaces/MoveFollowPath "{paths: [{type: 4, rotate_angle: 90, limit_w: 45}]}"

# Start charging
ros2 action send_goal /start_charging sr_amr_interfaces/action/StartCharging "{use_minute_limit: false, charging_minute: 0, use_percent_limit: false, charging_percent: 0}"
# Stop charging
ros2 action send_goal /stop_charging sr_amr_interfaces/action/StopCharging "{}"

# Localize by pose (position in meters, orientation as quaternion)
ros2 action send_goal /locate_by_pose sr_amr_interfaces/action/LocateByPose "{initial_pose: {position: {x: 1.2, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, force_set_initial_pose: false, timeout_sec: 60.0}"
# Localize by station
ros2 action send_goal /locate_by_station sr_amr_interfaces/action/LocateByStation "{station_id: 1, force_locate: false, timeout_sec: 60.0}"
# Switch map with initial pose
ros2 action send_goal /set_map_with_initial_pose sr_amr_interfaces/action/SetMapWithInitialPose "{map_name: 'Skyler', initial_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, force_set_initial_pose: false, timeout_sec: 120.0}"
```

services:

```bash
# Pause the current movement
ros2 service call /move_pause std_srvs/srv/Trigger
# Resume the paused movement
ros2 service call /move_resume std_srvs/srv/Trigger
# Enable remote control
ros2 service call /remote_control_enabled std_srvs/srv/SetBool "{data: true}"
# Disable remote control
ros2 service call /remote_control_enabled std_srvs/srv/SetBool "{data: false}"
# Enable obstacle avoidance for remote control
ros2 service call /remote_control_oba_enabled std_srvs/srv/SetBool "{data: true}"
# Disable obstacle avoidance for remote control
ros2 service call /remote_control_oba_enabled std_srvs/srv/SetBool "{data: false}"
# Enable lidar scan reporting
ros2 service call /lidar_enabled std_srvs/srv/SetBool "{data: true}"
# Disable lidar scan reporting
ros2 service call /lidar_enabled std_srvs/srv/SetBool "{data: false}"
# Emergency stop the AMR
ros2 service call /emergency_stop std_srvs/srv/Trigger
# Clear the emergency stop state of the AMR
ros2 service call /release_emergency_stop std_srvs/srv/Trigger
```

topic subscribers:

```bash
# remote control cmd_vel
ros2 topic pub /remote_control_cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

topic publishers:

```bash
# inspect lidar scan reporting
ros2 topic echo /front/scan
ros2 topic echo /rear/scan
```

## Error codes

You may got error code when calling services or sending action goals.

For ROS2 standard services, such as `std_srvs/Trigger` and `std_srvs/SetBool`, the error code will be in the `message` field of the response when `success` is `False`.

For `sr_amr_interfaces` actions, check action result fields directly.
For example, charging actions use `result/error_code`, while localization actions use `success/message`.

You can refer to the error codes defined in the [error_code.csv](../resources/error_code.csv) file for more details.

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](../LICENSE) file for details.
