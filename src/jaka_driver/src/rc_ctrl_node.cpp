#include <iostream>
#include <string>
#include <map>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

// Track the target pose of one handle with absolute Cartesian servo.
void track_pose(JAKAZuRobot &robot,
                const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                const char *arm_name)
{
    CartesianPose pose;
    if (!pose_to_cartesian(robot, msg, pose))
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] failed to convert pose to CartesianPose", arm_name);
        return;
    }

    int ret = robot.servo_p(&pose, MoveMode::ABS);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] servo_p failed: %s", arm_name, mapErr[ret].c_str());
    }
}

void left_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    track_pose(left_robot, msg, "left");
}

void right_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    track_pose(right_robot, msg, "right");
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

    // Connect and bring up both arms.
    left_robot.login_in(left_ip.c_str());
    right_robot.login_in(right_ip.c_str());
    left_robot.power_on();
    right_robot.power_on();
    left_robot.enable_robot();
    right_robot.enable_robot();

    // Smooth the servo commands while tracking the handles.
    left_robot.servo_speed_foresight(15, 0.03);
    right_robot.servo_speed_foresight(15, 0.03);

    left_robot.servo_move_enable(true);
    right_robot.servo_move_enable(true);

    // Subscribe to the two handle target topics.
    auto left_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        left_topic, 10, left_target_callback);
    auto right_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        right_topic, 10, right_target_callback);

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "dual_arm_ctrl started, tracking %s (left) and %s (right)",
                left_topic.c_str(), right_topic.c_str());

    rclcpp::spin(node);

    left_robot.servo_move_enable(false);
    right_robot.servo_move_enable(false);
    rclcpp::shutdown();
    return 0;
}
