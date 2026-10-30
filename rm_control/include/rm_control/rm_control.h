// Copyright 2024 realman-robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_CONTROL__RM_CONTROL_H_
#define RM_CONTROL__RM_CONTROL_H_

#include <memory>
#include <string>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

// RM Robot msg
// #include "rm_ros_interfaces/msg/jointpos75.hpp"

class Rm_Control: public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle < FollowJointTrajectory >;

  explicit Rm_Control(std::string name);
  ~Rm_Control() {
  }

  void timer_callback();

private:
  rm_ros_interfaces::msg::Jointpos joint_msg;
  // rm_ros_interfaces::msg::Jointpos75 joint7_msg;
  int arm_type_ = 75;
  bool follow_ = false;
  // 实例化样条
  rclcpp_action::Server < FollowJointTrajectory > ::SharedPtr action_server_;

  // 声明话题发布者
  rclcpp::Publisher < rm_ros_interfaces::msg::Jointpos > ::SharedPtr joint_pos_publisher;
  // rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos75>::SharedPtr joint_pos_publisher_75;

  // 声明话题订阅者
  rclcpp::Subscription < std_msgs::msg::Empty > ::SharedPtr Get_Move_Stop_Cmd;

  rclcpp::TimerBase::SharedPtr State_Timer;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr < const FollowJointTrajectory::Goal > goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr < GoalHandleFJT > goal_handle);
  void execute_move(const std::shared_ptr < GoalHandleFJT > goal_handle);
  void handle_accepted(const std::shared_ptr < GoalHandleFJT > goal_handle);
  void get_move_stop_callback(std_msgs::msg::Empty::SharedPtr msg);
};

#endif  // RM_CONTROL__RM_CONTROL_H_
