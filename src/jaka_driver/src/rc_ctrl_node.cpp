#include <iostream>
#include <string>
#include <map>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory>
#include <mutex>

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
//
// `gripped` is the cross-thread gate between the background re-arm thread and the
// target callback: re-arm writes ctrl_origin/ctrl_prev BEFORE setting gripped=true,
// and target_callback only reads them AFTER observing gripped==true, so the atomic
// store/load establishes the needed happens-before (no data race). While re-arming
// (gripped false) the target callback does nothing, so the SDK is never touched
// concurrently by servo_p and the re-arm sequence.
std::atomic<bool> gripped{false};
std::atomic<bool> rearming{false};
// Latest grip state, mirrored here so the re-arm thread can check whether the
// grip is STILL held when it finishes (a press-then-release during re-arm must
// not leave the arm tracking).
std::atomic<bool> grip_now{false};
std::thread rearm_thread;
bool debug = false;
CtrlPose ctrl_origin;   // controller pose at grip press (origin)
CtrlPose ctrl_prev;     // last controller pose we commanded
// Latest target pose (stored by value under a mutex: it is written by the target
// callback every cycle and read by the re-arm thread at grip press).
geometry_msgs::msg::PoseStamped latest_pose;
std::mutex latest_mtx;
// Whether the arm connected successfully at startup (login_in == 0). A failed
// arm is skipped for power_on/enable/servo.
bool arm_ok = false;
std::string arm_name = "left";
int grip_axis = 0;           // index into Joy.axes: 0 = leftGrip, 1 = rightGrip
int gripper_axis = 2;        // index trigger: 2 = leftTrig, 3 = rightTrig
bool gripper_use_button = false;
int gripper_button_index = 0;
bool gripper_linear = true; // true = linear (analog axes), false = digital (button)
double gripper_trigger_deadband = 0.02; // minimal change to send new Grip()
double prev_gripper_trigger = 0.0;
std::unique_ptr<pgi140::Pgi140Gripper> gripper;
bool use_rpy_ctrl = false;  // false = position-only teleop, true = position + attitude
bool enable_y_limit = true; // true = clamp arm TCP y to its workspace range
bool arm_is_right = false;  // mirror of arm_name == "right"
double arm_tcp_y_at_grip = 0.0; // arm's absolute TCP y captured at grip press (mm)
bool arm_tcp_valid = false;     // whether the grip-time TCP read succeeded

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
void pose_to_ctrl(const geometry_msgs::msg::PoseStamped &msg,
                  CtrlPose &pose)
{
    // ROS uses meters, the JAKA SDK uses millimeters.
    pose.tran.x = msg.pose.position.x * 1000.0;
    pose.tran.y = msg.pose.position.y * 1000.0;
    pose.tran.z = msg.pose.position.z * 1000.0;

    pose.quat.s = msg.pose.orientation.w;
    pose.quat.x = msg.pose.orientation.x;
    pose.quat.y = msg.pose.orientation.y;
    pose.quat.z = msg.pose.orientation.z;
}

// Per-cycle Cartesian servo limits. servo_p is fed one increment per 8 ms
// interpolation cycle, so SERVO_MAX_TRAN_STEP / 0.008 ~= 125 mm/s and
// SERVO_MAX_ROT_STEP / 0.008 ~= 36 deg/s.
constexpr double SERVO_TRAN_DEADBAND = 0.5;   // mm: ignore smaller translation deltas
constexpr double SERVO_ROT_DEADBAND = 0.005;  // rad (~0.29 deg): ignore smaller rotation deltas
constexpr double SERVO_MAX_TRAN_STEP = 5.0;   // mm per cycle (~125 mm/s)
constexpr double SERVO_MAX_ROT_STEP = 0.05;  // rad per cycle (~36 deg/s)
constexpr double Y_LIMIT_MM = 110.0;         // workspace y boundary: left > -110, right < +110

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
    pose_to_ctrl(*msg, ctrl_now);

    // Workspace y-limit: an absolute one-way barrier on the arm's TCP y so the
    // two arms never cross the shared center line. The commanded absolute
    // position is "TCP at grip + handle offset since grip"; we clamp the handle
    // target BEFORE computing the increment, so the arm stops exactly at the
    // boundary and is never commanded a reverse (pull-back) motion. No per-cycle
    // read-back, so there is no staleness and no bounce.
    if (enable_y_limit && arm_tcp_valid)
    {
        double pred_y = arm_tcp_y_at_grip + (ctrl_now.tran.y - ctrl_origin.tran.y);
        if (arm_is_right && pred_y > Y_LIMIT_MM)
            ctrl_now.tran.y = ctrl_origin.tran.y + (Y_LIMIT_MM - arm_tcp_y_at_grip);
        else if (!arm_is_right && pred_y < -Y_LIMIT_MM)
            ctrl_now.tran.y = ctrl_origin.tran.y + (-Y_LIMIT_MM - arm_tcp_y_at_grip);
    }

    CartesianPose incr;
    // Per-cycle translation increment (vector subtraction is exact).
    incr.tran.x = ctrl_now.tran.x - ctrl_prev.tran.x;
    incr.tran.y = ctrl_now.tran.y - ctrl_prev.tran.y;
    incr.tran.z = ctrl_now.tran.z - ctrl_prev.tran.z;

    // Attitude control is optional: when use_rpy_ctrl is false the handle only
    // drives translation and the arm holds its current orientation (rpy == 0).
    double max_rot = 0.0;
    incr.rpy.rx = 0.0;
    incr.rpy.ry = 0.0;
    incr.rpy.rz = 0.0;
    if (use_rpy_ctrl)
    {
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

        max_rot = std::max(std::fabs(incr.rpy.rx),
                 std::max(std::fabs(incr.rpy.ry), std::fabs(incr.rpy.rz)));
        if (max_rot > SERVO_MAX_ROT_STEP)
        {
            double s = SERVO_MAX_ROT_STEP / max_rot;
            incr.rpy.rx *= s;
            incr.rpy.ry *= s;
            incr.rpy.rz *= s;
        }
        incr.rpy.rx = -incr.rpy.rx;
        incr.rpy.ry = -incr.rpy.ry;
        incr.rpy.rz = -incr.rpy.rz;
    }

    // Deadband: hold in place instead of chasing sub-mm / sub-degree handle noise.
    double max_tran = std::max(std::fabs(incr.tran.x),
                      std::max(std::fabs(incr.tran.y), std::fabs(incr.tran.z)));
    if (max_tran < SERVO_TRAN_DEADBAND && (!use_rpy_ctrl || max_rot < SERVO_ROT_DEADBAND))
        return;

    // Velocity clamp: scale translation to its per-cycle cap, preserving
    // direction. A fast handle swing moves the arm at the capped speed instead
    // of commanding an overspeed Cartesian move.
    if (max_tran > SERVO_MAX_TRAN_STEP)
    {
        double s = SERVO_MAX_TRAN_STEP / max_tran;
        incr.tran.x *= s;
        incr.tran.y *= s;
        incr.tran.z *= s;
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

// Re-arm the robot if protection dropped it out of servo. Called on every grip
// press so a re-grip recovers from a protective/collision stop. Checks the
// states top-down (servo -> enable -> power-on) and drives them bottom-up
// (power_on -> enable_robot -> servo_move_enable(true)), only performing the
// steps actually needed:
//   - already in servo -> nothing to do;
//   - collision / error  -> collision_recover() + clear_error();
//   - powered off        -> power_on (+8 s settle);
//   - disabled           -> enable_robot (+4 s settle);
//   - otherwise          -> just re-enter servo.
// Mirrors bring_up_arm(), but skips login and the steps that are still intact.
bool ensure_arm_ready(JAKAZuRobot &robot)
{
    if (!arm_ok)
        return false;

    // Fast path: still in servo, nothing to recover.
    BOOL in_servo = FALSE;
    if (robot.is_in_servomove(&in_servo) == 0 && in_servo)
        return true;

    // Clear a protective stop / collision / motion-abnormal error first, so the
    // controller will accept power_on/enable/servo again.
    BOOL in_collision = FALSE;
    if (robot.is_in_collision(&in_collision) == 0 && in_collision)
    {
        int r = robot.collision_recover();
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] collision_recover ret=%d", arm_name.c_str(), r);
    }
    int r = robot.clear_error();
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] clear_error ret=%d", arm_name.c_str(), r);

    // Read power-on / enabled flags. get_robot_status_simple is the lightweight
    // source for both (errcode / powered_on / enabled).
    RobotStatus_simple status{};
    if (robot.get_robot_status_simple(&status) != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] get_robot_status_simple failed; cannot re-arm", arm_name.c_str());
        return false;
    }

    if (status.powered_on == 0)
    {
        int ret = robot.power_on();
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] re-arm power_on FAILED (ret=%d: %s)", arm_name.c_str(),
                        ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm power_on OK", arm_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(8));
        // Power-on leaves the robot disabled; refresh the flags before deciding
        // whether enable_robot is still needed.
        if (robot.get_robot_status_simple(&status) != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] get_robot_status_simple failed after power_on", arm_name.c_str());
            return false;
        }
    }

    if (status.enabled == 0)
    {
        int ret = robot.enable_robot();
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] re-arm enable_robot FAILED (ret=%d: %s)", arm_name.c_str(),
                        ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm enable_robot OK", arm_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }

    int ret = robot.servo_move_enable(true);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] re-arm servo_move_enable FAILED (ret=%d: %s)", arm_name.c_str(),
                    ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm servo enabled", arm_name.c_str());
    return true;
}

// Re-arm the robot in a background thread. The slow ensure_arm_ready sequence
// (power_on/enable/servo with multi-second settles) must NOT run in the executor
// thread, or it freezes target_callback for 12+ s. It also must NOT run
// concurrently with servo_p — the JAKA SDK holds one connection per process — so
// target_callback stays idle (gripped false) until this finishes.
void rearm_worker()
{
    bool ok = ensure_arm_ready(robot);
    if (ok && grip_now)
    {
        CtrlPose origin;
        {
            std::lock_guard<std::mutex> lk(latest_mtx);
            pose_to_ctrl(latest_pose, origin);
        }
        // Capture the arm's absolute TCP y once at grip press so the workspace
        // y-limit is enforced against the actual position (grip TCP + handle
        // offset) without a stale per-cycle read-back.
        CartesianPose tcp_at_grip;
        arm_tcp_valid = (robot.get_tcp_position(&tcp_at_grip) == 0);
        if (arm_tcp_valid)
            arm_tcp_y_at_grip = tcp_at_grip.tran.y;
        // Write origin/prev BEFORE publishing gripped=true so the atomic gate
        // makes these writes visible to target_callback.
        ctrl_origin = origin;
        ctrl_prev = origin;
        gripped = true;
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] gripped, ctrl_origin=(%.1f,%.1f,%.1f)mm",
                    arm_name.c_str(),
                    ctrl_origin.tran.x, ctrl_origin.tran.y, ctrl_origin.tran.z);
    }
    rearming = false;
}

// Update the grip state for one arm. On the rising edge (grip crosses 0.5) we
// kick off a background re-arm + origin capture; on release we send one explicit
// hold and stop tracking. While not gripped the target callback sends nothing
// (stopping INCR already holds the arm), so no per-cycle hold is needed.
void handle_grip(float value)
{
    bool now = value > 0.5f;
    grip_now = now;  // mirror current grip so the re-arm thread can check it
    if (now && !gripped && !rearming)
    {
        // Reap a previous, already-finished re-arm thread before overwriting the
        // std::thread (assigning to a joinable std::thread would terminate).
        if (rearm_thread.joinable())
            rearm_thread.join();
        rearming = true;
        rearm_thread = std::thread(rearm_worker);
    }
    else if (!now && gripped)
    {
        servo_hold(robot);
        gripped = false;
    }
}

void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!arm_ok)
        return;
    {
        std::lock_guard<std::mutex> lk(latest_mtx);
        latest_pose = *msg;
    }
    if (gripped)
    {
        servo_cartesian(robot, msg, ctrl_origin, ctrl_prev, arm_name.c_str());
    }
    // Not gripped: send nothing. Stopping the INCR stream already holds the arm;
    // the release edge in handle_grip sends one explicit servo_hold.
    if(debug)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] target_callback: %f,%f,%f",
                    arm_name.c_str(), msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    }
}

void button_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float grip = msg->axes.size() > static_cast<size_t>(grip_axis)
                     ? msg->axes[grip_axis] : 0.0f;
    if (arm_ok)
        handle_grip(grip);

    if (gripper != nullptr)
    {
        // Two gripper control modes:
        // - "digital": binary control via a Joy button -> set_closed(bool)
        // - "linear" : analog control via a Joy axis -> Grip(float)
        if (!gripper_linear)
        {
            bool pressed = msg->buttons.size() > static_cast<size_t>(gripper_button_index) &&
                           msg->buttons[gripper_button_index] != 0;
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
        else // linear (analog) control
        {
            float trigger_raw = msg->axes.size() > static_cast<size_t>(gripper_axis)
                                  ? msg->axes[gripper_axis] : 0.0f;
            // Normalize/clamp to [0,1]. Many controllers already report 0..1.
            float trigger = trigger_raw;
            if (trigger < 0.0f) trigger = 0.0f;
            if (trigger > 1.0f) trigger = 1.0f;
            if (std::fabs(trigger - prev_gripper_trigger) >= gripper_trigger_deadband||trigger>0.95)
            {
                std::string error;
                if (!gripper->Grip(trigger, &error))
                {
                    RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper Grip(%.3f) failed: %s",
                                arm_name.c_str(), trigger, error.c_str());
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper Grip(%.3f)",
                                arm_name.c_str(), trigger);
                }
                prev_gripper_trigger = trigger;
            }
        }
    }
}

// Bring one arm fully online: login -> configure TIO -> power_on -> enable ->
// servo. Mirrors the jaka_driver reference sequence (wait 8 s after power_on
// and 4 s after enable so the robot reaches the powered/enabled state before
// servo). Returns true only if every step succeeded; on failure the arm is
// left un-driven and the callbacks skip it.
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

    // Configure the TIO while the controller is logged in but before the arm
    // enters servo mode. JAKA controllers may reject pin-mode changes once
    // servo motion is active.
    if (gripper != nullptr)
    {
        std::string error;
        if (!gripper->initialize(&error))
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI gripper disabled: %s", arm_name.c_str(), error.c_str());
            gripper.reset();
        }
        else
        {
            const auto & config = gripper->config();
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI-140-80 ready: RS485 channel=%d slave=%d force=%d%% speed=%d%% "
                        "open=%d closed=%d trigger_axis=%d mode=%s button=%d deadband=%.3f",
                        arm_name.c_str(), config.rs485_channel, config.slave_id,
                        config.force_percent, config.speed_percent,
                        config.open_position, config.closed_position,
                        gripper_axis, (gripper_linear ? "linear" : "digital"),
                        (gripper_linear ? -1 : gripper_button_index),
                        gripper_trigger_deadband);
        }
    }

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

    //滤波设置
    //非线性滤波
    ret = robot.servo_move_use_carte_NLF(1500.0, 2500.0, 2800.0,1120.0, 1180.0, 1720.0);
    //速度前瞻滤波
    //ret = robot.servo_speed_foresight(10, 0.5);
    if(ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s servo_speed_foresight FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }


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
    arm_is_right = is_right;
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
    gripper_linear = node->declare_parameter<bool>("gripper_linear", true);
    gripper_trigger_deadband = node->declare_parameter<double>("gripper_trigger_deadband", 0.02);
    // Backwards compatibility: if the old boolean is set, prefer digital mode.
    if (gripper_use_button) {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] parameter gripper_use_button is present; using digital mode",
                    arm_name.c_str());
        gripper_linear = false;
    }
    // gripper_trigger_threshold removed; using deadband for linear control

    pgi140::GripperConfig gripper_config;
    gripper_config.rs485_channel = node->declare_parameter<int>("gripper_rs485_channel", 1);
    gripper_config.slave_id = node->declare_parameter<int>("gripper_slave_id", 1);
    gripper_config.baudrate = node->declare_parameter<int>("gripper_baudrate", 115200);
    gripper_config.force_percent = node->declare_parameter<int>("gripper_force", 20);
    gripper_config.speed_percent = node->declare_parameter<int>("gripper_speed", 100);
    gripper_config.open_position = node->declare_parameter<int>("gripper_open_position", 1000);
    gripper_config.closed_position = node->declare_parameter<int>(
        "gripper_closed_position", 0);
    gripper_config.initialize = node->declare_parameter<bool>("gripper_initialize", true);
    gripper_config.initialize_command = node->declare_parameter<int>(
        "gripper_initialize_command", 0x01);
    gripper_config.initialize_delay_ms = node->declare_parameter<int>(
        "gripper_initialize_delay_ms", 2000);
    gripper_config.configure_tio = node->declare_parameter<bool>(
        "gripper_configure_tio", false);
    gripper_config.enable_tio_power = node->declare_parameter<bool>(
        "gripper_enable_tio_power", true);
    gripper_config.tio_voltage = node->declare_parameter<int>(
        "gripper_tio_voltage", 0);

    
   debug = node->declare_parameter<bool>("debug", false);
   use_rpy_ctrl = node->declare_parameter<bool>("use_rpy_ctrl", false);
   enable_y_limit = node->declare_parameter<bool>("enable_y_limit", true);


    gripper = std::make_unique<pgi140::Pgi140Gripper>(robot, gripper_config);
    arm_ok = bring_up_arm(robot, ip, arm_name.c_str());
    if (!arm_ok)
    {
        gripper.reset();
    }

    // Subscribe to the handle target topic and the shared button topic.
    // keep_last(1): teleop 控制循环只关心最新一帧，depth=10 会在消费端稍慢时
    // 积压旧位姿、引入递增延迟。与 quest_reader 端 QoSProfile(depth=1) 兼容。
    auto target_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic, rclcpp::QoS(1), target_callback);
    auto button_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        button_topic, rclcpp::QoS(1), button_callback);

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "rc_ctrl_node started: arm=%s, ip=%s, tracking %s (grip axis %d)",
                arm_name.c_str(), ip.c_str(), target_topic.c_str(), grip_axis);

    rclcpp::spin(node);

    // Wait for an in-flight background re-arm to finish before touching the SDK
    // below (an active re-arm can block up to ~12 s on its power_on/enable
    // settles). Joining first avoids servo_move_enable/login_out racing it.
    if (rearm_thread.joinable())
        rearm_thread.join();

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
