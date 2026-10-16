#include "ecucomm.hpp"
#include <algorithm>
#include <cstddef>
#include <cmath>

serial::Serial ser;
namespace ecucomm
{
    namespace
    {
        double mapJoyAxis(double value)
        {
            const double deadzone = 0.1;
            value = std::max(-1.0, std::min(1.0, value));
            if (std::abs(value) <= deadzone)
            {
                return 0.0;
            }
            return std::copysign((std::abs(value) - deadzone) / (1.0 - deadzone), value);
        }
    }

    ecucomm::ecucomm(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::string node_name)
        : nh_(nh),
          nh_private_(nh_private),
          node_name_(node_name)
    {
        initialize();
    }

    // 初始化nh_private
    void ecucomm::initialize()
    {
        nh_private_.param("wheel_separation", wheel_separation_, 0.6);
        if (!std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0)
        {
            ROS_WARN("Invalid wheel_separation; using 0.6 m.");
            wheel_separation_ = 0.6;
        }
        nh_private_.param("cmd_vel_timeout", cmd_vel_timeout_, 1.0);
        if (!std::isfinite(cmd_vel_timeout_) || cmd_vel_timeout_ <= 0.0)
        {
            ROS_WARN("Invalid cmd_vel_timeout; using 1.0 s.");
            cmd_vel_timeout_ = 1.0;
        }
        ROS_INFO("cmd_vel timeout: %.3f s.", cmd_vel_timeout_);

        nh_private_.param<std::string>("joy_topic", joy_topic_, "joy");
        nh_private_.param("estop_button", estop_button_, 11);
        nh_private_.param("reset_button", reset_button_, 10);
        if (estop_button_ < 0 || reset_button_ < 0 || estop_button_ == reset_button_)
        {
            ROS_WARN("Invalid joystick button indices; using estop=11, reset=10.");
            estop_button_ = 11;
            reset_button_ = 10;
        }
        ROS_INFO("Joystick control: topic=%s, estop_button=%d, reset_button=%d.", joy_topic_.c_str(), estop_button_, reset_button_);
        ROS_INFO("Control mode: MANUAL. buttons[0]=AUTO, buttons[1]=MANUAL.");

        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &ecucomm::cmdVelCb, this);
        joy_sub_ = nh_.subscribe(joy_topic_, 10, &ecucomm::joyCb, this);
        // 使用实际时间触发发送，仿真时间暂停时仍能检查超时并暂停发送。
        loop_timer_ = nh_.createWallTimer(ros::WallDuration(0.01), &ecucomm::timerCb, this);
    }

    void ecucomm::timerCb(const ros::WallTimerEvent &)
    {
        // 尚未收到消息，或已经超时：保持静默，不发送串口指令。
        if (emergency_stop_ || !has_cmd_vel_)
        {
            return;
        }

        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - last_cmd_vel_time_)
                                   .count();
        if (elapsed >= cmd_vel_timeout_)
        {
            has_cmd_vel_ = false;
            joy_active_ = false;
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            ROS_WARN("No velocity input received for %.3f s; serial command transmission paused.", elapsed);
            return;
        }

        ROS_INFO_ONCE("Motor command sender started.");
        sendmsgs();
    }

    void ecucomm::cmdVelCb(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (manual_mode_)
        {
            return;
        }
        setVelocity(msg->linear.x, -msg->angular.z, "cmd_vel");
    }

    // 两个速度来源共用差速换算；角速度在调用前完成方向校正。
    void ecucomm::setVelocity(double linear_velocity, double angular_velocity, const char *source)
    {
        // 锁定期间丢弃速度消息，不更新指令或接收时间。
        if (emergency_stop_)
        {
            return;
        }

        last_cmd_vel_time_ = std::chrono::steady_clock::now();
        has_cmd_vel_ = true;

        if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity))
        {
            ROS_WARN_THROTTLE(1.0,
                              "Invalid %s: linear.x=%g, angular.z=%g; stopping both wheels.",
                              source, linear_velocity, angular_velocity);
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            return;
        }

        // 差速换算，输入角速度已经完成方向校正。
        const double leftv = linear_velocity - angular_velocity * wheel_separation_ / 2.0;
        const double rightv = linear_velocity + angular_velocity * wheel_separation_ / 2.0;

        // 轮速 1.4 m/s 对应控制量 1000，负速度对应负控制量。
        const double max_wheel_velocity = 1.4;
        const double max_motor_command = 1000.0;

        const double left_command = leftv / max_wheel_velocity * max_motor_command;
        const double right_command = rightv / max_wheel_velocity * max_motor_command;

        // 使用两轮原始控制量组合输出，避免赋值后覆盖原值。
        LeftWheel_speed = left_command - right_command;
        RightWheek_speed = right_command + left_command;
        const double mixed_left_command = LeftWheel_speed;
        const double mixed_right_command = RightWheek_speed;

        if (!std::isfinite(LeftWheel_speed) || !std::isfinite(RightWheek_speed))
        {
            ROS_WARN_THROTTLE(1.0,
                              "%s exceeds the supported numeric range: linear.x=%g, angular.z=%g, "
                              "mixed_left=%g, mixed_right=%g; stopping both wheels.",
                              source, linear_velocity, angular_velocity, mixed_left_command, mixed_right_command);
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            return;
        }

        // 组合后超过控制量范围时，同比例缩小两个通道的指令。
        const double peak_command = std::max(std::abs(LeftWheel_speed), std::abs(RightWheek_speed));
        if (peak_command > max_motor_command)
        {
            LeftWheel_speed = LeftWheel_speed / peak_command * max_motor_command;
            RightWheek_speed = RightWheek_speed / peak_command * max_motor_command;
        }

        ROS_INFO_THROTTLE(1.0,
                          "%s: linear.x=%.3f m/s, angular.z=%.3f rad/s, wheel_separation=%.3f m | "
                          "wheel_velocity: left=%.3f m/s, right=%.3f m/s | "
                          "raw_command: left=%.1f, right=%.1f | "
                          "mixed_command: left=%.1f, right=%.1f | "
                          "final_command: left=%.1f, right=%.1f, limited=%s",
                          source, linear_velocity, angular_velocity, wheel_separation_,
                          leftv, rightv, left_command, right_command,
                          mixed_left_command, mixed_right_command, LeftWheel_speed, RightWheek_speed,
                          peak_command > max_motor_command ? "yes" : "no");
    }

    void ecucomm::joyCb(const sensor_msgs::Joy::ConstPtr &msg)
    {
        if (msg->buttons.size() < 2 ||
            static_cast<std::size_t>(estop_button_) >= msg->buttons.size() ||
            static_cast<std::size_t>(reset_button_) >= msg->buttons.size())
        {
            // 按键数据不完整时锁定发送，避免越界或错误解锁。
            emergency_stop_ = true;
            has_cmd_vel_ = false;
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            joy_received_ = false;
            joy_active_ = false;
            reset_button_was_pressed_ = false;
            ROS_WARN_THROTTLE(1.0,
                              "Joy has only %zu buttons; estop=%d, reset=%d. Serial transmission locked.",
                              msg->buttons.size(), estop_button_, reset_button_);
            return;
        }

        const bool estop_pressed = msg->buttons[estop_button_] != 0;
        const bool reset_pressed = msg->buttons[reset_button_] != 0;
        const bool auto_pressed = msg->buttons[0] != 0;
        const bool manual_pressed = msg->buttons[1] != 0;
        // 首条有效消息只记录按键状态，避免把初始按住状态误判为新的按键动作。
        const bool reset_pressed_edge = joy_received_ && reset_pressed && !reset_button_was_pressed_;
        const bool auto_pressed_edge = joy_received_ && auto_pressed && !auto_button_was_pressed_;
        const bool manual_pressed_edge = joy_received_ && manual_pressed && !manual_button_was_pressed_;
        joy_received_ = true;
        reset_button_was_pressed_ = reset_pressed;
        auto_button_was_pressed_ = auto_pressed;
        manual_button_was_pressed_ = manual_pressed;

        // 急停优先；两键同时按下，或一直按住重置键，都不能解除急停。
        if (estop_pressed)
        {
            if (!emergency_stop_)
            {
                ROS_WARN("Joystick estop pressed; serial transmission locked.");
            }
            emergency_stop_ = true;
            has_cmd_vel_ = false;
            joy_active_ = false;
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            return;
        }
        else if (reset_pressed_edge)
        {
            emergency_stop_ = false;
            manual_mode_ = true;
            has_cmd_vel_ = false;
            joy_active_ = false;
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            ROS_INFO("Joystick reset pressed; unlocked in MANUAL mode, waiting for joystick input.");
            return;
        }
        if (emergency_stop_)
        {
            return;
        }

        // 急停、解锁优先于模式切换；两种模式键同时按下时手动优先。
        if (manual_pressed_edge || auto_pressed_edge)
        {
            manual_mode_ = manual_pressed;
            has_cmd_vel_ = false;
            joy_active_ = false;
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            ROS_INFO("Control mode: %s; waiting for a new %s input.",
                     manual_mode_ ? "MANUAL" : "AUTO", manual_mode_ ? "joystick" : "cmd_vel");
            return;
        }
        if (!manual_mode_)
        {
            return;
        }

        if (msg->axes.size() < 2 ||
            !std::isfinite(msg->axes[0]) || !std::isfinite(msg->axes[1]))
        {
            emergency_stop_ = true;
            has_cmd_vel_ = false;
            joy_active_ = false;
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            ROS_WARN_THROTTLE(1.0, "Invalid joystick axes; serial transmission locked.");
            return;
        }

        // axes[0] 左右，axes[1] 前后；正负方向使用相同的死区及线性映射。
        const double angular_velocity = mapJoyAxis(msg->axes[0]);
        const double linear_velocity = mapJoyAxis(msg->axes[1]);
        ROS_INFO_THROTTLE(1.0,
                          "Joy axes: turn=%.3f, forward=%.3f | mapped: angular=%.3f, linear=%.3f.",
                          msg->axes[0], msg->axes[1], angular_velocity, linear_velocity);

        // 初始回中状态不发送；开始操控后回中则发送零速度。
        if (!joy_active_ && linear_velocity == 0.0 && angular_velocity == 0.0)
        {
            return;
        }
        joy_active_ = true;
        // 与 cmd_vel 使用相同的转向方向校正，避免差速换算中重复取反。
        setVelocity(linear_velocity, -angular_velocity, "joy");
    }

    // 发送消息
    void ecucomm::sendmsgs()
    {
        // 在唯一的串口发送入口拦截，急停期间连零指令也不发送。
        if (emergency_stop_)
        {
            return;
        }

        std::string left_str;
        left_str = std::to_string(LeftWheel_speed);
        int position = left_str.find_first_of(".");
        left_str = left_str.substr(0, position);
        std::string right_str;
        right_str = std::to_string(RightWheek_speed);
        position = right_str.find_first_of(".");
        right_str = right_str.substr(0, position);

        std::string send_string;
        send_string = "!M " + left_str + " " + right_str + "\r";
        ROS_INFO_STREAM_THROTTLE(1.0, "Serial command: !M " << left_str << " " << right_str);
        ser.write(send_string);
    }

}

int main(int argc, char **argv)
{
    std::string node_name = "ecucomm";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // 创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    // 设置要打开的串口名称
    ser.setPort("/dev/ttyUSB0");
    // 设置串口通信的波特率
    ser.setBaudrate(115200);
    // 串口设置timeout
    ser.setTimeout(to);
    try
    {
        // 打开串口
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    // 判断串口是否打开成功
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    ecucomm::ecucomm sender(nh, nh_private, node_name);
    ROS_INFO("Initialized sender node.");
    ros::spin();
}
