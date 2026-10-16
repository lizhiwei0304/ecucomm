#include "ecucomm.hpp"
#include <algorithm>
#include <cmath>

serial::Serial ser;
namespace ecucomm
{
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

        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &ecucomm::cmdVelCb, this);
        // 使用实际时间触发发送，仿真时间暂停时仍能检查超时并暂停发送。
        loop_timer_ = nh_.createWallTimer(ros::WallDuration(0.01), &ecucomm::timerCb, this);
    }

    void ecucomm::timerCb(const ros::WallTimerEvent &)
    {
        // 尚未收到消息，或已经超时：保持静默，不发送串口指令。
        if (!has_cmd_vel_)
        {
            return;
        }

        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - last_cmd_vel_time_)
                                   .count();
        if (elapsed >= cmd_vel_timeout_)
        {
            has_cmd_vel_ = false;
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            ROS_WARN("No cmd_vel received for %.3f s; serial command transmission paused.", elapsed);
            return;
        }

        ROS_INFO_ONCE("Motor command sender started.");
        sendmsgs();
    }

    void ecucomm::cmdVelCb(const geometry_msgs::Twist::ConstPtr &msg)
    {
        last_cmd_vel_time_ = std::chrono::steady_clock::now();
        has_cmd_vel_ = true;

        const double linear_velocity = msg->linear.x;
        const double angular_velocity = -msg->angular.z;
        if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity))
        {
            ROS_WARN_THROTTLE(1.0,
                              "Invalid cmd_vel: linear.x=%g, angular.z=%g; stopping both wheels.",
                              linear_velocity, angular_velocity);
            LeftWheel_speed = 0;
            RightWheek_speed = 0;
            return;
        }

        // 差速底盘：正角速度为向左转，右轮比左轮快。
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
                              "cmd_vel exceeds the supported numeric range: linear.x=%g, angular.z=%g, "
                              "mixed_left=%g, mixed_right=%g; stopping both wheels.",
                              linear_velocity, angular_velocity, mixed_left_command, mixed_right_command);
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
                          "cmd_vel: linear.x=%.3f m/s, angular.z=%.3f rad/s, wheel_separation=%.3f m | "
                          "wheel_velocity: left=%.3f m/s, right=%.3f m/s | "
                          "raw_command: left=%.1f, right=%.1f | "
                          "mixed_command: left=%.1f, right=%.1f | "
                          "final_command: left=%.1f, right=%.1f, limited=%s",
                          linear_velocity, angular_velocity, wheel_separation_,
                          leftv, rightv, left_command, right_command,
                          mixed_left_command, mixed_right_command, LeftWheel_speed, RightWheek_speed,
                          peak_command > max_motor_command ? "yes" : "no");
    }

    // 发送消息
    void ecucomm::sendmsgs()
    {
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
