#include <iostream>
#include <string>
#include <map>
#include <chrono>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"

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

JAKAZuRobot left_robot;
JAKAZuRobot right_robot;

// Per-arm teleop state: whether the grip is currently held, the controller/arm
// poses captured at the press instant, and the latest controller pose.
bool left_gripped = false, right_gripped = false;
CartesianPose left_ctrl_origin, right_ctrl_origin;  // controller pose at grip press (origin)
CartesianPose left_ctrl_prev, right_ctrl_prev;      // last controller pose we commanded
geometry_msgs::msg::PoseStamped::SharedPtr latest_left, latest_right;
// Whether each arm connected successfully at startup (login_in == 0). A failed
// arm is skipped for power_on/enable/servo so its SDK errors don't hide the
// other, healthy arm.
bool left_ok = false, right_ok = false;

// Convert a PoseStamped (position in meters + quaternion) into a JAKA
// CartesianPose (translation in mm + RPY in radians).
bool pose_to_cartesian(JAKAZuRobot &robot,
                       const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                       CartesianPose &pose)
{
    // ROS uses meters, the JAKA SDK uses millimeters.
    pose.tran.x = msg->pose.position.x * 1000.0;
    pose.tran.y = msg->pose.position.y * 1000.0;
    pose.tran.z = msg->pose.position.z * 1000.0;

    Quaternion quat;
    quat.s = msg->pose.orientation.w;
    quat.x = msg->pose.orientation.x;
    quat.y = msg->pose.orientation.y;
    quat.z = msg->pose.orientation.z;

    RotMatrix rot;
    Rpy rpy;
    if (robot.quaternion_to_rot_matrix(&quat, &rot) != 0)
    {
        return false;
    }
    if (robot.rot_matrix_to_rpy(&rot, &rpy) != 0)
    {
        return false;
    }

    pose.rpy.rx = rpy.rx;
    pose.rpy.ry = rpy.ry;
    pose.rpy.rz = rpy.rz;

    return true;
}

// Per-cycle Cartesian servo limits. servo_p is fed one increment per 8 ms
// interpolation cycle, so SERVO_MAX_TRAN_STEP / 0.008 ~= 125 mm/s and
// SERVO_MAX_ROT_STEP / 0.008 ~= 36 deg/s.
constexpr double SERVO_TRAN_DEADBAND = 0.5;   // mm: ignore smaller translation deltas
constexpr double SERVO_ROT_DEADBAND = 0.005;  // rad (~0.29 deg): ignore smaller rotation deltas
constexpr double SERVO_MAX_TRAN_STEP = 1.0;   // mm per cycle (~125 mm/s)
constexpr double SERVO_MAX_ROT_STEP = 0.005;  // rad per cycle (~36 deg/s)

// Wrap an angle difference to [-pi, pi] so orientation deltas don't jump by 2pi
// when the controller pose crosses the +/-pi boundary.
double wrap_pi(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

// Drive one arm to track the controller's displacement since grip press, in
// Cartesian space. servo_p is sent one per-cycle INCR delta — the same
// feed-forward pattern as the working demo (servo_p(&cart, INCR) with a fixed
// increment). The grip-press pose is the origin: because INCR accumulates these
// increments, the arm ends up exactly at "origin + offset", returns to start
// when the controller returns to its origin, and holds when we stop sending.
// No IK and no read-back, so nothing can diverge/oscillate.
void servo_cartesian(JAKAZuRobot &robot,
                     const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                     const CartesianPose &ctrl_origin,
                     CartesianPose &ctrl_prev,
                     const char *arm_name)
{
    CartesianPose ctrl_now;
    if (!pose_to_cartesian(robot, msg, ctrl_now))
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] failed to convert pose to CartesianPose", arm_name);
        return;
    }

    // Total offset from the grip-press origin — the semantic "where the arm
    // should be". Printed for debugging; not sent directly.
    CartesianPose offset;
    offset.tran.x = ctrl_now.tran.x - ctrl_origin.tran.x;
    offset.tran.y = ctrl_now.tran.y - ctrl_origin.tran.y;
    offset.tran.z = ctrl_now.tran.z - ctrl_origin.tran.z;
    offset.rpy.rx = wrap_pi(ctrl_now.rpy.rx - ctrl_origin.rpy.rx);
    offset.rpy.ry = wrap_pi(ctrl_now.rpy.ry - ctrl_origin.rpy.ry);
    offset.rpy.rz = wrap_pi(ctrl_now.rpy.rz - ctrl_origin.rpy.rz);

    // Per-cycle increment = offset_now - offset_prev. INCR accumulates these, so
    // the running sum equals `offset` and the arm tracks the origin-relative
    // displacement without ever double-counting pending motion.
    CartesianPose incr;
    incr.tran.x = ctrl_now.tran.x - ctrl_prev.tran.x;
    incr.tran.y = ctrl_now.tran.y - ctrl_prev.tran.y;
    incr.tran.z = ctrl_now.tran.z - ctrl_prev.tran.z;
    incr.rpy.rx = wrap_pi(ctrl_now.rpy.rx - ctrl_prev.rpy.rx);
    incr.rpy.ry = wrap_pi(ctrl_now.rpy.ry - ctrl_prev.rpy.ry);
    incr.rpy.rz = wrap_pi(ctrl_now.rpy.rz - ctrl_prev.rpy.rz);

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

    // DEBUG: throttled print of the total offset and the per-cycle increment.
    static int dbg_count = 0;
    if (++dbg_count % 25 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] offset=(%.1f,%.1f,%.1f)mm rpy=(%.3f,%.3f,%.3f)rad incr=(%.2f,%.2f,%.2f)mm rincr=(%.4f,%.4f,%.4f)rad",
                    arm_name,
                    offset.tran.x, offset.tran.y, offset.tran.z,
                    offset.rpy.rx, offset.rpy.ry, offset.rpy.rz,
                    incr.tran.x, incr.tran.y, incr.tran.z,
                    incr.rpy.rx, incr.rpy.ry, incr.rpy.rz);
    }
}

// Update the grip state for one arm. On the rising edge (grip crosses 0.5) we
// capture the controller pose and the arm's current TCP as the two origins; on
// release we stop commanding so the arm holds its last position.
void handle_grip(bool &gripped, float value,
                 const geometry_msgs::msg::PoseStamped::SharedPtr &latest,
                 JAKAZuRobot &robot,
                 CartesianPose &ctrl_origin,
                 CartesianPose &ctrl_prev,
                 const char *arm_name)
{
    bool now = value > 0.5f;
    if (now && !gripped)
    {
        if (latest && pose_to_cartesian(robot, latest, ctrl_origin))
        {
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

void left_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!left_ok)
        return;
    latest_left = msg;
    if (left_gripped)
    {
        servo_cartesian(left_robot, msg, left_ctrl_origin, left_ctrl_prev, "left");
    }
}

void right_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!right_ok)
        return;
    latest_right = msg;
    if (right_gripped)
    {
        servo_cartesian(right_robot, msg, right_ctrl_origin, right_ctrl_prev, "right");
    }
}

void button_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float left_grip = msg->axes.size() > 0 ? msg->axes[0] : 0.0f;
    float right_grip = msg->axes.size() > 1 ? msg->axes[1] : 0.0f;
    if (left_ok)
        handle_grip(left_gripped, left_grip, latest_left, left_robot,
                    left_ctrl_origin, left_ctrl_prev, "left");
    if (right_ok)
        handle_grip(right_gripped, right_grip, latest_right, right_robot,
                    right_ctrl_origin, right_ctrl_prev, "right");
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
    auto node = rclcpp::Node::make_shared("dual_arm_ctrl");

    // Read connection and topic parameters (defaults match RC_ctrl.launch.py).
    string left_ip = node->declare_parameter<string>("robot_left_ip", "192.168.71.37");
    string right_ip = node->declare_parameter<string>("robot_right_ip", "192.168.71.36");
    string left_topic = node->declare_parameter<string>("left_target_topic", "/rc_ctrl/left_target");
    string right_topic = node->declare_parameter<string>("right_target_topic", "/rc_ctrl/right_target");
    string button_topic = node->declare_parameter<string>("button_topic", "/rc_ctrl/button");
    // Bring each arm fully online. Done sequentially (not login-both-then-setup)
    // so an arm isn't left idle between login and power_on while the other arm's
    // login blocks — that idle gap is what made power_on fail with
    // ERR_COMMUNICATION_ERR right after a successful login.
    left_ok = bring_up_arm(left_robot, left_ip, "left");
    right_ok = bring_up_arm(right_robot, right_ip, "right");

    // Subscribe to the two handle target topics.
    auto left_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        left_topic, 10, left_target_callback);
    auto right_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        right_topic, 10, right_target_callback);
    auto button_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        button_topic, 10, button_callback);

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "dual_arm_ctrl started, tracking %s (left) and %s (right)",
                left_topic.c_str(), right_topic.c_str());

    rclcpp::spin(node);

    // Graceful shutdown. Only touch arms that actually connected: calling
    // servo_move_enable/login_out on a disconnected arm blocks until the SDK
    // gives up, which hangs Ctrl-C and forces a SIGKILL — and a SIGKILL leaves
    // a stale session on the controller, blocking the next login.
    if (left_ok)
    {
        left_robot.servo_move_enable(false);
        left_robot.login_out();
    }
    if (right_ok)
    {
        right_robot.servo_move_enable(false);
        right_robot.login_out();
    }
    rclcpp::shutdown();
    return 0;
}
