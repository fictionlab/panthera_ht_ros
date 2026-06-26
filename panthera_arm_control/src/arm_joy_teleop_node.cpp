#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "panthera_interfaces/msg/end_pose_euler.hpp"
#include "panthera_interfaces/msg/pos_cmd.hpp"
#include "panthera_interfaces/srv/gripper_control.hpp"
#include "panthera_interfaces/srv/go_zero.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

using JoyMsg = sensor_msgs::msg::Joy;
using EndPoseEulerMsg = panthera_interfaces::msg::EndPoseEuler;
using PosCmdMsg = panthera_interfaces::msg::PosCmd;
using GripperControlSrv = panthera_interfaces::srv::GripperControl;
using GoZeroSrv = panthera_interfaces::srv::GoZero;

class ArmJoyTeleopNode : public rclcpp::Node
{
public:
    ArmJoyTeleopNode()
        : Node("arm_joy_teleop_node")
    {
        declareParameters();
        readParameters();

        joy_sub_ = this->create_subscription<JoyMsg>(
            joy_topic_, 1,
            std::bind(&ArmJoyTeleopNode::joyCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<EndPoseEulerMsg>(
            pose_topic_, 1,
            std::bind(&ArmJoyTeleopNode::poseCallback, this, std::placeholders::_1));

        pos_cmd_pub_ = this->create_publisher<PosCmdMsg>(pos_cmd_topic_, 1);
        gripper_client_ = this->create_client<GripperControlSrv>(gripper_service_);
        go_zero_client_ = this->create_client<GoZeroSrv>(go_zero_service_);

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&ArmJoyTeleopNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Arm Joy Teleop Node Started");
    }

private:
    struct PoseRPY
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double roll{0.0};
        double pitch{0.0};
        double yaw{0.0};
    };

    void declareParameters()
    {
        this->declare_parameter<std::string>("joy_topic", "/joy");
        this->declare_parameter<std::string>("pose_topic", "end_pose_euler");
        this->declare_parameter<std::string>("pos_cmd_topic", "/pos_cmd");
        this->declare_parameter<std::string>("gripper_service", "gripper_control");
        this->declare_parameter<std::string>("go_zero_service", "go_zero_srv");

        this->declare_parameter<double>("command_rate_hz", 60.0);
        this->declare_parameter<double>("linear_speed_mps", 0.045);
        this->declare_parameter<double>("angular_speed_rps", 0.2);
        this->declare_parameter<double>("deadzone", 0.1);

        this->declare_parameter<int>("pos_cmd_mode1", 0);
        this->declare_parameter<int>("pos_cmd_mode2", 0);
        this->declare_parameter<double>("axis_filter_alpha", 0.25);
        this->declare_parameter<double>("trigger_deadzone", 0.08);
        this->declare_parameter<double>("motion_epsilon", 0.002);
        this->declare_parameter<std::string>("trigger_mode", "signed");

        this->declare_parameter<int>("axis_x", 1);           // left stick vertical
        this->declare_parameter<int>("axis_y", 0);           // left stick horizontal
        this->declare_parameter<int>("axis_roll", 3);        // right stick horizontal
        this->declare_parameter<int>("axis_pitch", 4);       // right stick vertical
        this->declare_parameter<int>("axis_yaw", 6);         // dpad horizontal
        this->declare_parameter<int>("axis_z_up", 5);        // right trigger
        this->declare_parameter<int>("axis_z_down", 2);      // left trigger

        this->declare_parameter<int>("deadman_button", 5);         // RB: world-frame deadman
        this->declare_parameter<int>("local_deadman_button", 4);   // LB: local-frame deadman
        this->declare_parameter<int>("gripper_open_button", 3);   // Y
        this->declare_parameter<int>("gripper_close_button", 0);  // A
        this->declare_parameter<int>("home_button", 1);           // B
    }

    void readParameters()
    {
        joy_topic_ = this->get_parameter("joy_topic").as_string();
        pose_topic_ = this->get_parameter("pose_topic").as_string();
        pos_cmd_topic_ = this->get_parameter("pos_cmd_topic").as_string();
        gripper_service_ = this->get_parameter("gripper_service").as_string();
        go_zero_service_ = this->get_parameter("go_zero_service").as_string();

        command_rate_hz_ = this->get_parameter("command_rate_hz").as_double();
        linear_speed_mps_ = this->get_parameter("linear_speed_mps").as_double();
        angular_speed_rps_ = this->get_parameter("angular_speed_rps").as_double();
        deadzone_ = this->get_parameter("deadzone").as_double();

        pos_cmd_mode1_ = this->get_parameter("pos_cmd_mode1").as_int();
        pos_cmd_mode2_ = this->get_parameter("pos_cmd_mode2").as_int();
        axis_filter_alpha_ = clamp(this->get_parameter("axis_filter_alpha").as_double(), 0.01, 1.0);
        trigger_deadzone_ = clamp(this->get_parameter("trigger_deadzone").as_double(), 0.0, 0.49);
        motion_epsilon_ = clamp(this->get_parameter("motion_epsilon").as_double(), 1e-5, 0.1);
        trigger_mode_ = this->get_parameter("trigger_mode").as_string();
        std::transform(trigger_mode_.begin(), trigger_mode_.end(), trigger_mode_.begin(), ::tolower);
        if (trigger_mode_ != "signed" && trigger_mode_ != "unsigned") {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid trigger_mode '%s'. Using 'signed'.",
                        trigger_mode_.c_str());
            trigger_mode_ = "signed";
        }

        axis_x_ = this->get_parameter("axis_x").as_int();
        axis_y_ = this->get_parameter("axis_y").as_int();
        axis_roll_ = this->get_parameter("axis_roll").as_int();
        axis_pitch_ = this->get_parameter("axis_pitch").as_int();
        axis_yaw_ = this->get_parameter("axis_yaw").as_int();
        axis_z_up_ = this->get_parameter("axis_z_up").as_int();
        axis_z_down_ = this->get_parameter("axis_z_down").as_int();

        deadman_button_ = this->get_parameter("deadman_button").as_int();
        local_deadman_button_ = this->get_parameter("local_deadman_button").as_int();
        gripper_open_button_ = this->get_parameter("gripper_open_button").as_int();
        gripper_close_button_ = this->get_parameter("gripper_close_button").as_int();
        home_button_ = this->get_parameter("home_button").as_int();
    }

    static double clamp(double v, double vmin, double vmax)
    {
        return std::max(vmin, std::min(v, vmax));
    }

    static double wrapToPi(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double applyDeadzone(double value, double deadzone)
    {
        return (std::fabs(value) >= deadzone) ? value : 0.0;
    }

    static double getAxisValue(const JoyMsg & joy, int axis_index)
    {
        if (axis_index < 0 || axis_index >= static_cast<int>(joy.axes.size())) {
            return 0.0;
        }
        return joy.axes[axis_index];
    }

    static bool getButtonValue(const JoyMsg & joy, int button_index)
    {
        if (button_index < 0 || button_index >= static_cast<int>(joy.buttons.size())) {
            return false;
        }
        return joy.buttons[button_index] != 0;
    }

    static double lowPass(double previous, double input, double alpha)
    {
        return previous + alpha * (input - previous);
    }

    // Converts trigger axis value to 0..1.
    // Signed mode: released=1, pressed=-1.
    // Unsigned mode: released=0, pressed=1.
    double triggerToUnit(double raw) const
    {
        if (trigger_mode_ == "signed") {
            return clamp((1.0 - raw) * 0.5, 0.0, 1.0);
        }
        return clamp(raw, 0.0, 1.0);
    }

    void resetFilteredAxes()
    {
        filt_x_ = 0.0;
        filt_y_ = 0.0;
        filt_z_ = 0.0;
        filt_roll_ = 0.0;
        filt_pitch_ = 0.0;
        filt_yaw_ = 0.0;
    }

    void syncTargetToCurrentPose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        target_pose_ = current_pose_;
    }

    void joyCallback(const JoyMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        last_joy_ = *msg;
        joy_received_ = true;
    }

    void poseCallback(const EndPoseEulerMsg::SharedPtr msg)
    {
        PoseRPY current;
        current.x = msg->x;
        current.y = msg->y;
        current.z = msg->z;
        current.roll = msg->roll;
        current.pitch = msg->pitch;
        current.yaw = msg->yaw;

        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = current;
        if (!target_initialized_) {
            target_pose_ = current_pose_;
            target_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Teleop target initialized from current end-effector pose");
        }
        pose_received_ = true;
    }

    void controlLoop()
    {
        if (!joy_received_ || !pose_received_ || !target_initialized_) {
            return;
        }

        JoyMsg joy;
        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            joy = last_joy_;
        }

        const bool world_deadman_pressed = getButtonValue(joy, deadman_button_);
        const bool local_deadman_pressed = getButtonValue(joy, local_deadman_button_);
        const bool deadman_pressed = world_deadman_pressed || local_deadman_pressed;
        const double dt = 1.0 / std::max(command_rate_hz_, 1.0);

        if (deadman_pressed && !deadman_prev_) {
            // Clutch behavior: start from measured pose when deadman is re-engaged.
            syncTargetToCurrentPose();
            resetFilteredAxes();
        }

        if (!deadman_pressed) {
            // Keep last target; this avoids abrupt stop when deadman is released.
            resetFilteredAxes();
        }
        deadman_prev_ = deadman_pressed;

        const double x_in = applyDeadzone(getAxisValue(joy, axis_x_), deadzone_);
        const double y_in = applyDeadzone(getAxisValue(joy, axis_y_), deadzone_);
        const double roll_in = applyDeadzone(-getAxisValue(joy, axis_roll_), deadzone_);
        const double pitch_in = applyDeadzone(getAxisValue(joy, axis_pitch_), deadzone_);
        const double yaw_in = applyDeadzone(getAxisValue(joy, axis_yaw_), deadzone_);

        const double z_up = applyDeadzone(triggerToUnit(getAxisValue(joy, axis_z_up_)), trigger_deadzone_);
        const double z_down = applyDeadzone(triggerToUnit(getAxisValue(joy, axis_z_down_)), trigger_deadzone_);
        const double z_in = applyDeadzone(z_up - z_down, deadzone_);

        const bool open_gripper = getButtonValue(joy, gripper_open_button_);
        const bool close_gripper = getButtonValue(joy, gripper_close_button_);
        const bool home_pressed = getButtonValue(joy, home_button_);
        const bool open_gripper_edge = open_gripper && !gripper_open_prev_;
        const bool close_gripper_edge = close_gripper && !gripper_close_prev_;

        if (home_pressed && !home_button_prev_) {
            sendGoZeroRequest();
            resetFilteredAxes();
            syncTargetToCurrentPose();
        }
        home_button_prev_ = home_pressed;
        gripper_open_prev_ = open_gripper;
        gripper_close_prev_ = close_gripper;

        filt_x_ = lowPass(filt_x_, x_in, axis_filter_alpha_);
        filt_y_ = lowPass(filt_y_, y_in, axis_filter_alpha_);
        filt_z_ = lowPass(filt_z_, z_in, axis_filter_alpha_);
        filt_roll_ = lowPass(filt_roll_, roll_in, axis_filter_alpha_);
        filt_pitch_ = lowPass(filt_pitch_, pitch_in, axis_filter_alpha_);
        filt_yaw_ = lowPass(filt_yaw_, yaw_in, axis_filter_alpha_);

        const bool has_motion_cmd =
            std::fabs(filt_x_) > motion_epsilon_ || std::fabs(filt_y_) > motion_epsilon_ || std::fabs(filt_z_) > motion_epsilon_ ||
            std::fabs(filt_roll_) > motion_epsilon_ || std::fabs(filt_pitch_) > motion_epsilon_ || std::fabs(filt_yaw_) > motion_epsilon_;
        const bool has_gripper_cmd = open_gripper_edge || close_gripper_edge;

        if ((!deadman_pressed || !has_motion_cmd) && !has_gripper_cmd) {
            return;
        }

        PoseRPY next_target;
        PoseRPY frame_pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            next_target = target_pose_;
            frame_pose = current_pose_;
        }

        double tx = filt_x_;
        double ty = filt_y_;
        double tz = filt_z_;

        if (local_deadman_pressed) {
            // Local-frame steering: rotate the stick translation vector by the
            // current end-effector orientation into the base frame.
            const double cr = std::cos(frame_pose.roll);
            const double sr = std::sin(frame_pose.roll);
            const double cp = std::cos(frame_pose.pitch);
            const double sp = std::sin(frame_pose.pitch);
            const double cy = std::cos(frame_pose.yaw);
            const double sy = std::sin(frame_pose.yaw);

            const double r00 = cy * cp;
            const double r01 = cy * sp * sr - sy * cr;
            const double r02 = cy * sp * cr + sy * sr;
            const double r10 = sy * cp;
            const double r11 = sy * sp * sr + cy * cr;
            const double r12 = sy * sp * cr - cy * sr;
            const double r20 = -sp;
            const double r21 = cp * sr;
            const double r22 = cp * cr;

            const double lx = tx;
            const double ly = ty;
            const double lz = tz;
            tx = r00 * lx + r01 * ly + r02 * lz;
            ty = r10 * lx + r11 * ly + r12 * lz;
            tz = r20 * lx + r21 * ly + r22 * lz;
        }

        next_target.x += tx * linear_speed_mps_ * dt;
        next_target.y += ty * linear_speed_mps_ * dt;
        next_target.z += tz * linear_speed_mps_ * dt;
        next_target.roll = wrapToPi(next_target.roll + filt_roll_ * angular_speed_rps_ * dt);
        next_target.pitch = wrapToPi(next_target.pitch + filt_pitch_ * angular_speed_rps_ * dt);
        next_target.yaw = wrapToPi(next_target.yaw + filt_yaw_ * angular_speed_rps_ * dt);

        PosCmdMsg cmd_msg;
        cmd_msg.x = next_target.x;
        cmd_msg.y = next_target.y;
        cmd_msg.z = next_target.z;
        cmd_msg.roll = next_target.roll;
        cmd_msg.pitch = next_target.pitch;
        cmd_msg.yaw = next_target.yaw;
        cmd_msg.gripper = -1.0;
        cmd_msg.mode1 = static_cast<uint8_t>(std::clamp(pos_cmd_mode1_, 0, 255));
        cmd_msg.mode2 = static_cast<uint8_t>(std::clamp(pos_cmd_mode2_, 0, 255));
        pos_cmd_pub_->publish(cmd_msg);

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            target_pose_ = next_target;
        }

        if (open_gripper_edge || close_gripper_edge) {
            sendGripperCommand(open_gripper_edge ? "open" : "close");
        }
    }

    void sendGoZeroRequest()
    {
        if (!go_zero_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Go-zero service %s not ready", go_zero_service_.c_str());
            return;
        }

        auto request = std::make_shared<GoZeroSrv::Request>();
        go_zero_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Home return requested");
    }

    void sendGripperCommand(const std::string & action)
    {
        if (!gripper_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Gripper service %s not ready", gripper_service_.c_str());
            return;
        }

        auto request = std::make_shared<GripperControlSrv::Request>();
        request->action = action;
        gripper_client_->async_send_request(request);
    }

    std::string joy_topic_;
    std::string pose_topic_;
    std::string pos_cmd_topic_;
    std::string gripper_service_;
    std::string go_zero_service_;

    double command_rate_hz_{60.0};
    double linear_speed_mps_{0.045};
    double angular_speed_rps_{0.2};
    double deadzone_{0.1};
    double axis_filter_alpha_{0.25};
    double trigger_deadzone_{0.08};
    double motion_epsilon_{0.002};
    std::string trigger_mode_{"signed"};
    int pos_cmd_mode1_{0};
    int pos_cmd_mode2_{0};

    int axis_x_{1};
    int axis_y_{0};
    int axis_roll_{3};
    int axis_pitch_{4};
    int axis_yaw_{6};
    int axis_z_up_{5};
    int axis_z_down_{2};

    int deadman_button_{5};
    int local_deadman_button_{4};
    int gripper_open_button_{3};
    int gripper_close_button_{0};
    int home_button_{1};

    bool joy_received_{false};
    bool pose_received_{false};
    bool target_initialized_{false};
    bool deadman_prev_{false};
    bool home_button_prev_{false};
    bool gripper_open_prev_{false};
    bool gripper_close_prev_{false};

    double filt_x_{0.0};
    double filt_y_{0.0};
    double filt_z_{0.0};
    double filt_roll_{0.0};
    double filt_pitch_{0.0};
    double filt_yaw_{0.0};

    JoyMsg last_joy_;
    PoseRPY current_pose_;
    PoseRPY target_pose_;

    std::mutex joy_mutex_;
    std::mutex pose_mutex_;

    rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
    rclcpp::Subscription<EndPoseEulerMsg>::SharedPtr pose_sub_;
    rclcpp::Publisher<PosCmdMsg>::SharedPtr pos_cmd_pub_;
    rclcpp::Client<GripperControlSrv>::SharedPtr gripper_client_;
    rclcpp::Client<GoZeroSrv>::SharedPtr go_zero_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmJoyTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}