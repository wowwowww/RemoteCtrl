# PGI-140-80 TIO driver

The PGI driver is integrated into each `jaka_driver/rc_ctrl_node` process so it
uses the same JAKA SDK login as the arm servo loop. This is intentional: a
second process trying to log in to the same JAKA controller can leave a stale
SDK session and cannot safely share the robot connection.

## Wiring and protocol

The PGI-140-80 is connected to the JAKA S5 TIO RS485 pair and its 24 V supply.
The driver uses Modbus-RTU, default `115200 8N1`, slave ID `1`, and function
code `0x06` writes:

| Register | Meaning | Range |
| --- | --- | --- |
| `0x0100` | initialization | `1` or `0xA5` |
| `0x0101` | force | `20..100` percent |
| `0x0103` | target position | `0..1000` per mille |
| `0x0104` | speed | `1..100` percent |

`gripper_rs485_channel=0` configures TIO RS485H (DO1/DO2, TIO channel 1);
`1` configures TIO RS485L (AIN1/AIN2, TIO channel 2). The default is `1`,
matching the PGI-compatible TIO wiring used by the local JAKA driver. The
driver enables TIO voltage output at 24 V by default. Confirm the actual TIO
wiring and supply before powering hardware. TIO pin/RS485 configuration is
performed immediately after `login_in`, before robot power and servo mode are
enabled; some controllers reject pin-mode changes after servo mode starts.
If the TIO channel has already been configured in the JAKA App and the
controller rejects runtime configuration, use `gripper_configure_tio:=false`.
The saved configuration must still match the selected channel, `115200 8N1`,
Modbus RTU mode, and the PGI slave ID.

## Quest mapping

`quest_reader` publishes `sensor_msgs/Joy` on `/rc_ctrl/button`:

* `axes[2]` is `leftTrig`, `axes[3]` is `rightTrig`.
* A trigger at or above `gripper_trigger_threshold` closes its gripper.
* Releasing below the threshold opens it.
* The default mode also accepts the corresponding digital `LTr`/`RTr` state
  when the controller does not publish an analog trigger value.

For a digital-only front-button report, set `gripper_use_button:=true`. The default
button index is automatically `10` (`LTr`) for the left arm and `11` (`RTr`)
for the right arm; override it with `gripper_button_index:=N` if the controller
reports a different layout. The stable digital order is
`X,Y,A,B,LThU,RThU,LJ,RJ,LG,RG,LTr,RTr`.

## Launch example

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jaka_driver dual_arm_teleop.launch.py \
  robot_left_ip:=192.168.71.37 robot_right_ip:=192.168.71.36 \
  gripper_force:=60 gripper_speed:=40
```

Useful overrides include `gripper_open_position`, `gripper_closed_position`,
`gripper_rs485_channel`, `gripper_slave_id`, `gripper_baudrate`,
`gripper_configure_tio`, `gripper_initialize_command` and
`gripper_enable_tio_power`. Keep
`gripper_initialize:=true` after changing
fingers or the saved PGI travel calibration; initialization can move the jaws
for up to about three seconds.

The driver only sends a new position on a button state transition, so it does
not compete with the 125 Hz arm servo stream. On normal shutdown it sends an
open command before logging out of the JAKA controller.
