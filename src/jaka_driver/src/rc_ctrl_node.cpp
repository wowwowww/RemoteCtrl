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
CartesianPose left_ctrl_origin, right_ctrl_origin;
CartesianPose left_robot_origin, right_robot_origin;
JointValue left_prev_joint, right_prev_joint;
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
    pose.tran.x = msg->pose.position.x * 100.0;
    pose.tran.y = msg->pose.position.y * 100.0;
    pose.tran.z = msg->pose.position.z * 100.0;

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

// Per-cycle servo limits. The joint servo is fed one increment per 8 ms
// interpolation cycle, so SERVO_MAX_STEP / 0.008 s ~= 0.5 rad/s max joint speed.
constexpr double SERVO_DEADBAND = 0.0002;  // rad: ignore smaller deltas (kills handle-noise jitter)
constexpr double SERVO_MAX_STEP = 0.004;   // rad per cycle (~0.5 rad/s)

// Drive one arm to track the controller's displacement since grip press.
//
// servo_p (Cartesian servo) returned success but did not move the arm on this
// controller, so we servo in JOINT space instead — the mode the working
// servoj_demo uses. Each cycle we:
//   1. build the absolute Cartesian target = arm TCP origin + controller delta,
//   2. solve it to joint angles with the SDK IK (seeded from the current joints
//      so the solver stays on the same branch),
//   3. servo_j by the per-cycle joint increment in INCR mode.
void servo_ik(JAKAZuRobot &robot,
              const geometry_msgs::msg::PoseStamped::SharedPtr msg,
              const CartesianPose &ctrl_origin,
              const CartesianPose &robot_origin,
              JointValue &prev_joint,
              const char *arm_name)
{
    CartesianPose ctrl_now;
    if (!pose_to_cartesian(robot, msg, ctrl_now))
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] failed to convert pose to CartesianPose", arm_name);
        return;
    }

    // Absolute Cartesian target: TCP pose captured at press + controller delta.
    CartesianPose target;
    target.tran.x = robot_origin.tran.x + (ctrl_now.tran.x - ctrl_origin.tran.x);
    target.tran.y = robot_origin.tran.y + (ctrl_now.tran.y - ctrl_origin.tran.y);
    target.tran.z = robot_origin.tran.z + (ctrl_now.tran.z - ctrl_origin.tran.z);
    target.rpy.rx = robot_origin.rpy.rx + (ctrl_now.rpy.rx - ctrl_origin.rpy.rx);
    target.rpy.ry = robot_origin.rpy.ry + (ctrl_now.rpy.ry - ctrl_origin.rpy.ry);
    target.rpy.rz = robot_origin.rpy.rz + (ctrl_now.rpy.rz - ctrl_origin.rpy.rz);

    // Solve to joint space, seeded with the joints we last commanded. We do NOT
    // read back get_joint_position() here: servo_j(INCR) adds its increment to
    // the robot's internal servo reference, so measuring the delta against a
    // read-back value double-counts the pending motion and makes the command
    // diverge/oscillate every cycle — the source of the violent jitter.
    JointValue joint_target;
    if (robot.kine_inverse(&prev_joint, &target, &joint_target) != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] kine_inverse failed for target=(%.1f,%.1f,%.1f)mm",
                    arm_name, target.tran.x, target.tran.y, target.tran.z);
        return;
    }

    JointValue incr;
    double max_abs = 0.0;
    for (int i = 0; i < 6; i++)
    {
        incr.jVal[i] = joint_target.jVal[i] - prev_joint.jVal[i];
        double a = std::fabs(incr.jVal[i]);
        if (a > max_abs)
            max_abs = a;
    }

    // Deadband: hold in place instead of chasing sub-degree handle noise.
    if (max_abs < SERVO_DEADBAND)
        return;

    // Velocity clamp: scale the whole increment down to SERVO_MAX_STEP while
    // preserving its direction, so a fast handle move can't command overspeed.
    if (max_abs > SERVO_MAX_STEP)
    {
        double s = SERVO_MAX_STEP / max_abs;
        for (int i = 0; i < 6; i++)
            incr.jVal[i] *= s;
    }

    int ret = robot.servo_j(&incr, MoveMode::INCR);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] servo_j failed: %s", arm_name, mapErr[ret].c_str());
        return;
    }

    // Track what we actually commanded so the next cycle's delta is measured
    // from the same reference the robot is using.
    for (int i = 0; i < 6; i++)
        prev_joint.jVal[i] += incr.jVal[i];

    // DEBUG: throttled print of raw controller pose, the Cartesian target, and
    // the per-cycle joint increment.
    static int dbg_count = 0;
    if (++dbg_count % 25 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] raw=(%.3f,%.3f,%.3f)m target=(%.1f,%.1f,%.1f)mm jincr=(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)rad",
                    arm_name,
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    target.tran.x, target.tran.y, target.tran.z,
                    incr.jVal[0], incr.jVal[1], incr.jVal[2], incr.jVal[3], incr.jVal[4], incr.jVal[5]);
    }
}

// Update the grip state for one arm. On the rising edge (grip crosses 0.5) we
// capture the controller pose and the arm's current TCP as the two origins; on
// release we stop commanding so the arm holds its last position.
void handle_grip(bool &gripped, float value,
                 const geometry_msgs::msg::PoseStamped::SharedPtr &latest,
                 JAKAZuRobot &robot,
                 CartesianPose &ctrl_origin,
                 CartesianPose &robot_origin,
                 JointValue &prev_joint,
                 const char *arm_name)
{
    bool now = value > 0.5f;
    if (now && !gripped)
    {
        if (latest && pose_to_cartesian(robot, latest, ctrl_origin) &&
            robot.get_tcp_position(&robot_origin) == 0 &&
            robot.get_joint_position(&prev_joint) == 0)
        {
            gripped = true;
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] gripped, ctrl_origin=(%.1f,%.1f,%.1f)mm robot_origin=(%.1f,%.1f,%.1f)mm",
                        arm_name,
                        ctrl_origin.tran.x, ctrl_origin.tran.y, ctrl_origin.tran.z,
                        robot_origin.tran.x, robot_origin.tran.y, robot_origin.tran.z);
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
        servo_ik(left_robot, msg, left_ctrl_origin, left_robot_origin, left_prev_joint, "left");
    }
}

void right_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!right_ok)
        return;
    latest_right = msg;
    if (right_gripped)
    {
        servo_ik(right_robot, msg, right_ctrl_origin, right_robot_origin, right_prev_joint, "right");
    }
}

void button_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float left_grip = msg->axes.size() > 0 ? msg->axes[0] : 0.0f;
    float right_grip = msg->axes.size() > 1 ? msg->axes[1] : 0.0f;
    if (left_ok)
        handle_grip(left_gripped, left_grip, latest_left, left_robot,
                    left_ctrl_origin, left_robot_origin, left_prev_joint, "left");
    if (right_ok)
        handle_grip(right_gripped, right_grip, latest_right, right_robot,
                    right_ctrl_origin, right_robot_origin, right_prev_joint, "right");
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

    // Hard joint-space velocity/acceleration/jerk limits (deg/s, deg/s^2,
    // deg/s^3): a safety bound on top of the per-cycle software clamp.
    robot.servo_move_use_joint_NLF(30.0, 300.0, 3000.0);

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
