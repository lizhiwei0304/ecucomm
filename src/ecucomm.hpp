//
// Created by lizhiwei on 20-06-18.
//

#ifndef ECUCOMM_HPP
#define ECUCOMM_HPP
#include <ros/ros.h>
#include <serial/serial.h>
#include <chrono>
#include <string>
#include <geometry_msgs/Twist.h>

namespace ecucomm
{
  class ecucomm
  {
  public:
    ecucomm(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
            std::string node_name);

    // ros设置
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::WallTimer loop_timer_;
    std::string node_name_{"ecucomm_node"};
    // 接收底盘线速度（m/s）和角速度（rad/s）。
    ros::Subscriber cmd_vel_sub_;
    void initialize();
    void timerCb(const ros::WallTimerEvent &event);
    void cmdVelCb(const geometry_msgs::Twist::ConstPtr &msg);
    void sendmsgs();
    double LeftWheel_speed = 0;
    double RightWheek_speed = 0;
    // 超过此时长未收到 cmd_vel 就暂停发送，使用单调时钟计时，单位为秒。
    double cmd_vel_timeout_ = 1.0;
    bool has_cmd_vel_ = false;
    std::chrono::steady_clock::time_point last_cmd_vel_time_{};
    // 左右轮中心间距，单位为米。
    double wheel_separation_ = 0.55;
  };
} // namespace ecucomm

#endif
