// Copyright (c) 2024 RealMan Intelligent Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/armstate.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SlightMoveDemo : public rclcpp::Node
{
public:
  SlightMoveDemo()
  : Node("slight_move_demo"),
    started_at_(std::chrono::steady_clock::now()),
    command_started_at_(started_at_)
  {
    declare_parameter<bool>("execute_motion", false);
    declare_parameter<int>("joint_index", 1);
    declare_parameter<double>("delta_rad", 0.008726646259971648);  // 0.5 degree
    declare_parameter<int>("speed", 5);
    declare_parameter<bool>("return_to_start", true);
    declare_parameter<double>("state_timeout_sec", 10.0);
    declare_parameter<double>("motion_timeout_sec", 30.0);

    get_parameter("execute_motion", execute_motion_);
    get_parameter("joint_index", joint_index_);
    get_parameter("delta_rad", delta_rad_);
    get_parameter("speed", speed_);
    get_parameter("return_to_start", return_to_start_);
    get_parameter("state_timeout_sec", state_timeout_sec_);
    get_parameter("motion_timeout_sec", motion_timeout_sec_);

    state_request_pub_ = create_publisher<std_msgs::msg::Empty>(
      "rm_driver/get_current_arm_state_cmd", rclcpp::ParametersQoS());
    move_pub_ = create_publisher<rm_ros_interfaces::msg::Movej>(
      "rm_driver/movej_cmd", rclcpp::ParametersQoS());
    state_sub_ = create_subscription<rm_ros_interfaces::msg::Armstate>(
      "rm_driver/get_current_arm_state_result", rclcpp::ParametersQoS(),
      std::bind(&SlightMoveDemo::state_callback, this, _1));
    result_sub_ = create_subscription<std_msgs::msg::Bool>(
      "rm_driver/movej_result", rclcpp::ParametersQoS(),
      std::bind(&SlightMoveDemo::result_callback, this, _1));
    timer_ = create_wall_timer(250ms, std::bind(&SlightMoveDemo::timer_callback, this));

    if (!parameters_are_safe()) {
      stage_ = Stage::ERROR;
      return;
    }

    if (!execute_motion_) {
      stage_ = Stage::DISABLED;
      RCLCPP_WARN(
        get_logger(),
        "Slight motion is disabled. Set execute_motion:=true only with an operator, "
        "a clear workspace, and the hardware E-stop ready.");
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Slight motion is armed: joint=%d, delta=%.6f rad, speed=%d, return_to_start=%s",
      joint_index_, delta_rad_, speed_, return_to_start_ ? "true" : "false");
  }

private:
  enum class Stage
  {
    WAITING_FOR_STATE,
    MOVING_OUT,
    RETURNING,
    COMPLETE,
    DISABLED,
    ERROR
  };

  bool parameters_are_safe()
  {
    constexpr double max_delta_rad = 0.017453292519943295;  // 1 degree
    if (joint_index_ < 1 || joint_index_ > 7) {
      RCLCPP_ERROR(get_logger(), "joint_index must be in the range 1..7");
      return false;
    }
    if (!std::isfinite(delta_rad_) || std::abs(delta_rad_) > max_delta_rad || delta_rad_ == 0.0) {
      RCLCPP_ERROR(get_logger(), "delta_rad must be non-zero and no greater than 1 degree");
      return false;
    }
    if (speed_ < 1 || speed_ > 20) {
      RCLCPP_ERROR(get_logger(), "speed must be in the conservative range 1..20");
      return false;
    }
    if (!std::isfinite(state_timeout_sec_) || state_timeout_sec_ < 1.0 ||
      state_timeout_sec_ > 60.0)
    {
      RCLCPP_ERROR(get_logger(), "state_timeout_sec must be in the range 1..60");
      return false;
    }
    if (!std::isfinite(motion_timeout_sec_) || motion_timeout_sec_ < 1.0 ||
      motion_timeout_sec_ > 120.0)
    {
      RCLCPP_ERROR(get_logger(), "motion_timeout_sec must be in the range 1..120");
      return false;
    }
    return true;
  }

  void timer_callback()
  {
    const auto now = std::chrono::steady_clock::now();
    if (stage_ == Stage::WAITING_FOR_STATE) {
      const double elapsed = std::chrono::duration<double>(now - started_at_).count();
      if (elapsed > state_timeout_sec_) {
        stage_ = Stage::ERROR;
        RCLCPP_ERROR(
          get_logger(), "No valid arm state received within %.1f seconds; no motion was sent",
          state_timeout_sec_);
        return;
      }
      if (state_request_pub_->get_subscription_count() == 0) {
        return;
      }
      state_request_pub_->publish(std_msgs::msg::Empty());
      return;
    }

    if (stage_ == Stage::MOVING_OUT || stage_ == Stage::RETURNING) {
      const double elapsed = std::chrono::duration<double>(now - command_started_at_).count();
      if (elapsed > motion_timeout_sec_) {
        stage_ = Stage::ERROR;
        RCLCPP_ERROR(
          get_logger(), "MoveJ result timed out after %.1f seconds; inspect the robot before retrying",
          motion_timeout_sec_);
      }
    }
  }

  void state_callback(const rm_ros_interfaces::msg::Armstate::SharedPtr msg)
  {
    if (stage_ != Stage::WAITING_FOR_STATE) {
      return;
    }
    if (msg->err != 0 || msg->err_len != 0) {
      RCLCPP_ERROR(
        get_logger(), "Arm state reports an error (err=%u, err_len=%u); no motion was sent",
        static_cast<unsigned int>(msg->err), static_cast<unsigned int>(msg->err_len));
      stage_ = Stage::ERROR;
      return;
    }
    if ((msg->dof != 6 && msg->dof != 7) || msg->joint.size() < msg->dof) {
      RCLCPP_ERROR(
        get_logger(), "Invalid arm state: dof=%u, joint_count=%zu",
        static_cast<unsigned int>(msg->dof), msg->joint.size());
      stage_ = Stage::ERROR;
      return;
    }
    if (joint_index_ > msg->dof) {
      RCLCPP_ERROR(
        get_logger(), "joint_index=%d exceeds the connected arm dof=%u",
        joint_index_, static_cast<unsigned int>(msg->dof));
      stage_ = Stage::ERROR;
      return;
    }
    if (move_pub_->get_subscription_count() == 0) {
      return;
    }

    start_joint_.assign(msg->joint.begin(), msg->joint.begin() + msg->dof);
    for (const float joint : start_joint_) {
      if (!std::isfinite(joint)) {
        RCLCPP_ERROR(
          get_logger(), "Arm state contains a non-finite joint value; no motion was sent");
        stage_ = Stage::ERROR;
        return;
      }
    }

    auto target_joint = start_joint_;
    target_joint[static_cast<std::size_t>(joint_index_ - 1)] += static_cast<float>(delta_rad_);
    publish_move(target_joint);
    stage_ = Stage::MOVING_OUT;
    command_started_at_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Sent the bounded slight-move command");
  }

  void result_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (stage_ != Stage::MOVING_OUT && stage_ != Stage::RETURNING) {
      return;
    }
    if (!msg->data) {
      stage_ = Stage::ERROR;
      RCLCPP_ERROR(get_logger(), "MoveJ failed; the example will not send another motion command");
      return;
    }

    if (stage_ == Stage::MOVING_OUT && return_to_start_) {
      publish_move(start_joint_);
      stage_ = Stage::RETURNING;
      command_started_at_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(), "Slight move succeeded; returning to the captured start position");
      return;
    }

    stage_ = Stage::COMPLETE;
    RCLCPP_INFO(get_logger(), "Slight-move example completed successfully");
  }

  void publish_move(const std::vector<float> & joint)
  {
    rm_ros_interfaces::msg::Movej command;
    command.joint = joint;
    command.speed = static_cast<uint8_t>(speed_);
    command.block = true;
    command.trajectory_connect = 0;
    command.dof = static_cast<uint8_t>(joint.size());
    move_pub_->publish(command);
  }

  bool execute_motion_ = false;
  int joint_index_ = 1;
  double delta_rad_ = 0.008726646259971648;
  int speed_ = 5;
  bool return_to_start_ = true;
  double state_timeout_sec_ = 10.0;
  double motion_timeout_sec_ = 30.0;
  Stage stage_ = Stage::WAITING_FOR_STATE;
  std::chrono::steady_clock::time_point started_at_;
  std::chrono::steady_clock::time_point command_started_at_;
  std::vector<float> start_joint_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr state_request_pub_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr move_pub_;
  rclcpp::Subscription<rm_ros_interfaces::msg::Armstate>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlightMoveDemo>());
  rclcpp::shutdown();
  return 0;
}
