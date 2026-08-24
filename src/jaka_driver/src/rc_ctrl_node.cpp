#include <iostream>
#include <string>
#include <map>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/pgi140_gripper.hpp"

using namespace std;

// SDK return status -> readable message
map<int, string> mapErr = {
    {2,   "ERR_FUCTION_CALL_ERROR"},
    {-1,  "ERR_INVALID_HANDLER"},
    {-2,  "ERR_INVALID_PARAMETER"},
    {-3,  "ERR_COMMUNICATION_ERR"},
    {-4,  "ERR_KINE_INVERSE_ERR"},
    {-5,  "ERR_EMERGENCY_PRESSED"},
    {-6,  "ERR_NOT_POWERED"},
    {-7,  "ERR_NOT_ENABLED"},
    {-8,  "ERR_DISABLE_SERVOMODE"},
    {-9,  "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"}
};

// Single-arm teleop node. The JAKA SDK holds ONE global connection per process
// (libjakaAPI.so's `robot` shared_ptr), so a process can only drive a single
// robot. This node controls exactly one arm, chosen by the `arm_name` parameter;
// the launch file starts two instances (one per arm).
JAKAZuRobot robot;

// Controller pose: translation (mm) + orientation (quaternion). Orientation is
// kept as a quaternion end-to-end so the per-cycle incremental rotation is
// computed correctly; RPY is derived only for the final servo_p delta.
struct CtrlPose {
    CartesianTran tran;   // mm
    Quaternion   quat;    // s == w (SDK scalar == ROS w)
};

// Teleop state: whether the grip is currently held, the controller pose captured
// at the press instant, and the latest controller pose.
bool gripped = false;
CtrlPose ctrl_origin;   // controller pose at grip press (origin)
CtrlPose ctrl_prev;     // last controller pose we commanded
geometry_msgs::msg::PoseStamped::SharedPtr latest;
// Whether the arm connected successfully at startup (login_in == 0). A failed
// arm is skipped for power_on/enable/servo.
bool arm_ok = false;
std::string arm_name = "left";
int grip_axis = 0;           // index into Joy.axes: 0 = leftGrip, 1 = rightGrip
int gripper_axis = 2;        // index trigger: 2 = leftTrig, 3 = rightTrig
bool gripper_use_button = false;
int gripper_button_index = 0;
double gripper_trigger_threshold = 0.5;
std::unique_ptr<pgi140::Pgi140Gripper> gripper;

// Quaternion is (s, x, y, z) with s the scalar, matching ROS (w, x, y, z).
Quaternion quat_conjugate(const Quaternion &q)
{
    // Inverse of a unit quaternion == conjugate (negate the vector part).
    Quaternion r;
    r.s = q.s;
    r.x = -q.x;
    r.y = -q.y;
    r.z = -q.z;
    return r;
}

// Hamilton product a * b  ("apply b, then a").
Quaternion quat_mul(const Quaternion &a, const Quaternion &b)
{
    Quaternion r;
    r.s = a.s * b.s - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.s * b.x + a.x * b.s + a.y * b.z - a.z * b.y;
    r.y = a.s * b.y - a.x * b.z + a.y * b.s + a.z * b.x;
    r.z = a.s * b.z + a.x * b.y - a.y * b.x + a.z * b.s;
    return r;
}

// Convert a PoseStamped (position in meters + quaternion) into a controller pose
// (translation in mm + quaternion). No RPY conversion here — the RPY delta is
// computed from quaternions in servo_cartesian().
void pose_to_ctrl(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                  CtrlPose &pose)
{
    // ROS uses meters, the JAKA SDK uses millimeters.
    pose.tran.x = msg->pose.position.x * 1000.0;
    pose.tran.y = msg->pose.position.y * 1000.0;
    pose.tran.z = msg->pose.position.z * 1000.0;

    pose.quat.s = msg->pose.orientation.w;
    pose.quat.x = msg->pose.orientation.x;
    pose.quat.y = msg->pose.orientation.y;
    pose.quat.z = msg->pose.orientation.z;
}

// Per-cycle Cartesian servo limits. servo_p is fed one increment per 8 ms
// interpolation cycle, so SERVO_MAX_TRAN_STEP / 0.008 ~= 125 mm/s and
// SERVO_MAX_ROT_STEP / 0.008 ~= 36 deg/s.
constexpr double SERVO_TRAN_DEADBAND = 0.5;   // mm: ignore smaller translation deltas
constexpr double SERVO_ROT_DEADBAND = 0.005;  // rad (~0.29 deg): ignore smaller rotation deltas
constexpr double SERVO_MAX_TRAN_STEP = 5.0;   // mm per cycle (~125 mm/s)
constexpr double SERVO_MAX_ROT_STEP = 0.05;  // rad per cycle (~36 deg/s)

// Drive one arm to track the controller's displacement since grip press, in
// Cartesian space. servo_p is sent one per-cycle INCR delta — the same
// feed-forward pattern as the working demo (servo_p(&cart, INCR) with a fixed
// increment). The grip-press pose is the origin: because INCR accumulates these
// increments, the arm ends up exactly at "origin + offset", returns to start
// when the controller returns to its origin, and holds when we stop sending.
// No IK and no read-back, so nothing can diverge/oscillate.
//
// The incremental ROTATION is computed from quaternions (q_now * q_prev^-1) and
// only then converted to RPY: subtracting two absolute RPYs does not yield the
// incremental rotation (RPY is non-linear and gimbal-lock singular).
void servo_cartesian(JAKAZuRobot &robot,
                     const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                     const CtrlPose &ctrl_origin,
                     CtrlPose &ctrl_prev,
                     const char *arm_name)
{
    CtrlPose ctrl_now;
    pose_to_ctrl(msg, ctrl_now);

    CartesianPose incr;
    // Per-cycle translation increment (vector subtraction is exact).
    incr.tran.x = ctrl_now.tran.x - ctrl_prev.tran.x;
    incr.tran.y = ctrl_now.tran.y - ctrl_prev.tran.y;
    incr.tran.z = ctrl_now.tran.z - ctrl_prev.tran.z;

    // Per-cycle orientation increment: q_incr = q_now * q_prev^-1, a
    // near-identity rotation that converts back to RPY without singularities.
    Quaternion q_incr = quat_mul(ctrl_now.quat, quat_conjugate(ctrl_prev.quat));
    RotMatrix rot_incr;
    if (robot.quaternion_to_rot_matrix(&q_incr, &rot_incr) != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] failed to convert quaternion to rot matrix", arm_name);
        return;
    }
    if (robot.rot_matrix_to_rpy(&rot_incr, &incr.rpy) != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] failed to convert rot matrix to RPY", arm_name);
        return;
    }

    // Deadband: hold in place instead of chasing sub-mm / sub-degree handle noise.
    double max_tran = std::max(std::fabs(incr.tran.x),
                      std::max(std::fabs(incr.tran.y), std::fabs(incr.tran.z)));
    double max_rot = std::max(std::fabs(incr.rpy.rx),
                     std::max(std::fabs(incr.rpy.ry), std::fabs(incr.rpy.rz)));
    if (max_tran < SERVO_TRAN_DEADBAND && max_rot < SERVO_ROT_DEADBAND)
        return;

    // Velocity clamp: scale translation and rotation separately, each to its own
    // per-cycle cap, preserving direction. A fast handle swing moves the arm at
    // the capped speed instead of commanding an overspeed Cartesian move.
    if (max_tran > SERVO_MAX_TRAN_STEP)
    {
        double s = SERVO_MAX_TRAN_STEP / max_tran;
        incr.tran.x *= s;
        incr.tran.y *= s;
        incr.tran.z *= s;
    }
    if (max_rot > SERVO_MAX_ROT_STEP)
    {
        double s = SERVO_MAX_ROT_STEP / max_rot;
        incr.rpy.rx *= s;
        incr.rpy.ry *= s;
        incr.rpy.rz *= s;
    }
    incr.rpy.rx=-incr.rpy.rx;
    incr.rpy.ry=-incr.rpy.ry;
    incr.rpy.rz=-incr.rpy.rz;

    int ret = robot.servo_p(&incr, MoveMode::INCR);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] servo_p failed: %s", arm_name, mapErr[ret].c_str());
        return;
    }

    // Track what we actually commanded so the next cycle's delta is measured
    // from the same reference the robot is using.
    ctrl_prev = ctrl_now;

    // DEBUG: throttled print of the total translation offset and the per-cycle
    // incremental rotation (the quaternion-derived RPY delta).
    static int dbg_count = 0;
    if (++dbg_count % 25 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] offset=(%.1f,%.1f,%.1f)mm incr=(%.2f,%.2f,%.2f)mm rincr=(%.4f,%.4f,%.4f)rad",
                    arm_name,
                    ctrl_now.tran.x - ctrl_origin.tran.x,
                    ctrl_now.tran.y - ctrl_origin.tran.y,
                    ctrl_now.tran.z - ctrl_origin.tran.z,
                    incr.tran.x, incr.tran.y, incr.tran.z,
                    incr.rpy.rx, incr.rpy.ry, incr.rpy.rz);
    }
}

void servo_hold(JAKAZuRobot &robot)
{
    CartesianPose hold;
    hold.tran.x = 0.0;
    hold.tran.y = 0.0;
    hold.tran.z = 0.0;
    hold.rpy.rx = 0.0;
    hold.rpy.ry = 0.0;
    hold.rpy.rz = 0.0;
    int ret = robot.servo_p(&hold, MoveMode::INCR);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] servo_p HOLD failed: %s", arm_name.c_str(), mapErr[ret].c_str());
    }
}
// Update the grip state for one arm. On the rising edge (grip crosses 0.5) we
// capture the controller pose and the arm's current TCP as the two origins; on
// release we stop commanding so the arm holds its last position.
void handle_grip(bool &gripped, float value,
                 const geometry_msgs::msg::PoseStamped::SharedPtr &latest,
                 CtrlPose &ctrl_origin,
                 CtrlPose &ctrl_prev,
                 const char *arm_name)
{
    bool now = value > 0.5f;
    if (now && !gripped)
    {
        if (latest)
        {
            pose_to_ctrl(latest, ctrl_origin);
            ctrl_prev = ctrl_origin;
            gripped = true;
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] gripped, ctrl_origin=(%.1f,%.1f,%.1f)mm",
                        arm_name,
                        ctrl_origin.tran.x, ctrl_origin.tran.y, ctrl_origin.tran.z);
        }
    }
    else if (!now)
    {
        gripped = false;
    }
}

void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!arm_ok)
        return;
    latest = msg;
    if (gripped)
    {
        servo_cartesian(robot, msg, ctrl_origin, ctrl_prev, arm_name.c_str());
    }
    else//if not gripped
    {
        servo_hold(robot);
    }
}

void button_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float grip = msg->axes.size() > static_cast<size_t>(grip_axis)
                     ? msg->axes[grip_axis] : 0.0f;
    if (arm_ok)
        handle_grip(gripped, grip, latest,
                    ctrl_origin, ctrl_prev, arm_name.c_str());

    if (gripper != nullptr)
    {
        bool pressed = false;
        if (gripper_use_button)
        {
            pressed = msg->buttons.size() > static_cast<size_t>(gripper_button_index) &&
                      msg->buttons[gripper_button_index] != 0;
        }
        else
        {
            float trigger = msg->axes.size() > static_cast<size_t>(gripper_axis)
                              ? msg->axes[gripper_axis] : 0.0f;
            pressed = trigger >= static_cast<float>(gripper_trigger_threshold);
            // The Quest APK normally supplies the analog value, but some
            // controller firmware reports only the digital LTr/RTr state.
            // Accept either representation unless an alternate button-only
            // mode was explicitly requested.
            if (!pressed && msg->buttons.size() > static_cast<size_t>(gripper_button_index)) {
                pressed = msg->buttons[gripper_button_index] != 0;
            }
        }

        if (pressed != gripper->closed())
        {
            std::string error;
            if (!gripper->set_closed(pressed, &error))
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] PGI gripper command failed: %s",
                            arm_name.c_str(), error.c_str());
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                            "[%s] PGI gripper %s",
                            arm_name.c_str(), pressed ? "closed" : "opened");
            }
        }
    }
}

// Bring one arm fully online: login -> power_on -> enable -> servo. Mirrors the
// jaka_driver reference sequence (wait 8 s after power_on and 4 s after enable
// so the robot actually reaches the powered/enabled state before servo). Returns
// true only if every step succeeded; on failure the arm is left un-driven and
// the callbacks skip it.
bool bring_up_arm(JAKAZuRobot &robot, const std::string &ip, const char *arm)
{
    int ret = robot.login_in(ip.c_str());
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s login FAILED (%s, ret=%d: %s)",
                    arm, ip.c_str(), ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s login OK (%s)", arm, ip.c_str());

    // Same SDK setup the reference node applies right after login.
    robot.set_status_data_update_time_interval(100);
    robot.set_block_wait_timeout(120);

    ret = robot.power_on();
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s power_on FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s power_on OK", arm);
    std::this_thread::sleep_for(std::chrono::seconds(8));

    ret = robot.enable_robot();
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s enable_robot FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s enable_robot OK", arm);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // Keep collision detection ON but raise the threshold 25N -> 75N. Level 0
    // (disable) would let the arm push through obstacles — unsafe in VR teleop.
    robot.set_collision_level(3);

    // Hard Cartesian velocity/acceleration/jerk limits for servo_p (mm/s,
    // mm/s^2, mm/s^3) plus orientation limits (deg/s, deg/s^2, deg/s^3):
    // a safety bound on top of the per-cycle software clamp.
    robot.servo_move_use_carte_NLF(200.0, 2000.0, 20000.0,
                                   90.0, 900.0, 9000.0);

    ret = robot.servo_move_enable(true);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s servo_move_enable FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    BOOL in_servo = FALSE;
    robot.is_in_servomove(&in_servo);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s servo enabled (is_in_servomove=%d)", arm, in_servo);
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "C");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rc_ctrl_node");

    // Which arm this process controls. The JAKA SDK limits us to one connection
    // per process, so one instance drives exactly one arm and the launch file
    // starts two instances (arm_name left / right).
    arm_name = node->declare_parameter<string>("arm_name", "left");
    bool is_right = (arm_name == "right");
    string default_ip = is_right ? "192.168.71.36" : "192.168.71.37";
    string default_topic = is_right ? "/rc_ctrl/right_target" : "/rc_ctrl/left_target";
    int default_grip_axis = is_right ? 1 : 0;

    // Read connection/topic parameters (defaults match RC_ctrl.launch.py).
    string ip = node->declare_parameter<string>("robot_ip", default_ip);
    string target_topic = node->declare_parameter<string>("target_topic", default_topic);
    string button_topic = node->declare_parameter<string>("button_topic", "/rc_ctrl/button");
    grip_axis = node->declare_parameter<int>("grip_axis", default_grip_axis);

    // PGI-140-80 is driven through the Quest index-trigger axis by default:
    // Joy axes[2] = leftTrig and axes[3] = rightTrig. A Joy button index can
    // be selected for controllers whose front button is reported digitally.
    gripper_axis = node->declare_parameter<int>(
        "gripper_axis", is_right ? 3 : 2);
    gripper_use_button = node->declare_parameter<bool>("gripper_use_button", false);
    const int configured_button_index = node->declare_parameter<int>(
        "gripper_button_index", -1);
    // Joy.buttons follows quest_reader's stable order; LTr=10 and RTr=11.
    gripper_button_index = configured_button_index >= 0 ? configured_button_index :
        (is_right ? 11 : 10);
    gripper_trigger_threshold = node->declare_parameter<double>(
        "gripper_trigger_threshold", 0.5);
    if (!std::isfinite(gripper_trigger_threshold) ||
        gripper_trigger_threshold < 0.0 || gripper_trigger_threshold > 1.0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] invalid gripper_trigger_threshold; using 0.5",
                    arm_name.c_str());
        gripper_trigger_threshold = 0.5;
    }

    pgi140::GripperConfig gripper_config;
    gripper_config.rs485_channel = node->declare_parameter<int>("gripper_rs485_channel", 0);
    gripper_config.slave_id = node->declare_parameter<int>("gripper_slave_id", 1);
    gripper_config.baudrate = node->declare_parameter<int>("gripper_baudrate", 115200);
    gripper_config.force_percent = node->declare_parameter<int>("gripper_force", 50);
    gripper_config.speed_percent = node->declare_parameter<int>("gripper_speed", 50);
    gripper_config.open_position = node->declare_parameter<int>("gripper_open_position", 0);
    gripper_config.closed_position = node->declare_parameter<int>(
        "gripper_closed_position", 1000);
    gripper_config.initialize = node->declare_parameter<bool>("gripper_initialize", true);
    gripper_config.initialize_command = node->declare_parameter<int>(
        "gripper_initialize_command", 0x01);
    gripper_config.initialize_delay_ms = node->declare_parameter<int>(
        "gripper_initialize_delay_ms", 2000);
    gripper_config.enable_tio_power = node->declare_parameter<bool>(
        "gripper_enable_tio_power", true);
    gripper_config.tio_voltage = node->declare_parameter<int>(
        "gripper_tio_voltage", 0);

    arm_ok = bring_up_arm(robot, ip, arm_name.c_str());

    if (arm_ok)
    {
        gripper = std::make_unique<pgi140::Pgi140Gripper>(robot, gripper_config);
        std::string error;
        if (!gripper->initialize(&error))
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI gripper disabled: %s", arm_name.c_str(), error.c_str());
            gripper.reset();
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI-140-80 ready: RS485 channel=%d slave=%d force=%d%% speed=%d%% "
                        "open=%d closed=%d trigger axis=%d%s button=%d",
                        arm_name.c_str(), gripper_config.rs485_channel, gripper_config.slave_id,
                        gripper_config.force_percent, gripper_config.speed_percent,
                        gripper_config.open_position, gripper_config.closed_position,
                        gripper_axis, gripper_use_button ? " (digital button)" : "",
                        gripper_use_button ? gripper_button_index : -1);
        }
    }

    // Subscribe to the handle target topic and the shared button topic.
    auto target_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic, 10, target_callback);
    auto button_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        button_topic, 10, button_callback);

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "rc_ctrl_node started: arm=%s, ip=%s, tracking %s (grip axis %d)",
                arm_name.c_str(), ip.c_str(), target_topic.c_str(), grip_axis);

    rclcpp::spin(node);

    // Graceful shutdown. Only touch the arm if it actually connected: calling
    // servo_move_enable/login_out on a disconnected arm blocks until the SDK
    // gives up, which hangs Ctrl-C and forces a SIGKILL — and a SIGKILL leaves
    // a stale session on the controller, blocking the next login.
    if (arm_ok)
    {
        if (gripper != nullptr && gripper->closed())
        {
            std::string error;
            if (!gripper->set_closed(false, &error))
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] failed to open PGI gripper on shutdown: %s",
                            arm_name.c_str(), error.c_str());
            }
        }
        robot.servo_move_enable(false);
        robot.login_out();
    }
    rclcpp::shutdown();
    return 0;
}
