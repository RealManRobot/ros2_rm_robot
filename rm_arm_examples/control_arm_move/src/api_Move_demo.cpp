//
// Created by ubuntu on 23-11-28.
//
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/movejp.hpp"
#include "rm_ros_interfaces/msg/movel.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "rm_ros_interfaces/msg/movec.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
bool movej_state = false;
bool movej_p_state = false;
bool movel_state = false;
bool movec_state = false;
bool first_run = true;
/****************************************创建类************************************/ 
class MoveDemoPub: public rclcpp::Node
{
  public:
    MoveDemoPub();                                                                          //构造函数
    void looppub_timer_callback();                                                          //move运动规划函数

    
  
  private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movej_p_publisher_;       //声明发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;          //声明发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_publisher_;          //声明发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movec>::SharedPtr movec_publisher_; 
    rclcpp::TimerBase::SharedPtr loop_pub_Timer;                                           //定时发布器
    rm_ros_interfaces::msg::Movej movej_way;
    int arm_dof_ = 6;
};

/****************************************创建类************************************/ 
class MoveDemoSub: public rclcpp::Node
{
  public:
    MoveDemoSub();                                                                          //构造函数
    void MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                            //结果回调函数
    void MoveJDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                            //结果回调函数
    void MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                             //结果回调函数
    void MoveCDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                             //结果回调函数
    
  
  private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_p_subscription_;            //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_subscription_;              //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_subscription_;              //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movec_subscription_;              //声明订阅器
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveDemoSub::MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    movej_p_state = msg->data;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Movej_p succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"*******Movej_p Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveDemoSub::MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    movel_state = msg->data;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******MoveL succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"*******MoveL Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveDemoSub::MoveJDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    movej_state = msg->data;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Movej succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"*******Movej Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveDemoSub::MoveCDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    movec_state = msg->data;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******MoveC succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"*******MoveC Failed\n");
    }
}   
/***********************************************end**************************************************/

/*******************************************movejp运动函数****************************************/
void MoveDemoPub::looppub_timer_callback()
{
  if(first_run == true)
  {
  
    if(arm_dof_ == 6)
    {
      movej_way.joint[0] = -0.360829;
      movej_way.joint[1] = 0.528468;
      movej_way.joint[2] = 1.326293;
      movej_way.joint[3] = -0.000454;
      movej_way.joint[4] = 1.221748;
      movej_way.joint[5] = 0.000052;
      movej_way.speed = 20;
      movej_way.dof = 6;
    }
    if(arm_dof_ == 7)
    {
      movej_way.joint[0] = 0.176278;
      movej_way.joint[1] = 0.0;
      movej_way.joint[2] = 0.3543;
      movej_way.joint[3] = 0.53;
      movej_way.joint[4] = 0.00873;
      movej_way.joint[5] = 0.3595;
      movej_way.joint[6] = 0.3595;
      movej_way.speed = 20;
      movej_way.dof = 7;
    }
    movej_way.block = true;
    this->movej_publisher_->publish(movej_way);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    first_run = false;
  }
  if(movej_state == true)
  {
    rm_ros_interfaces::msg::Movejp moveJ_P_TargetPose;
    moveJ_P_TargetPose.pose.position.x = -0.355816;
    moveJ_P_TargetPose.pose.position.y = -0.000013;
    moveJ_P_TargetPose.pose.position.z = 0.222814;
    moveJ_P_TargetPose.pose.orientation.x = 0.995179;
    moveJ_P_TargetPose.pose.orientation.y = -0.094604;
    moveJ_P_TargetPose.pose.orientation.z = -0.025721;
    moveJ_P_TargetPose.pose.orientation.w = 0.002349;
    moveJ_P_TargetPose.speed = 20;
    moveJ_P_TargetPose.block = true;
    this->movej_p_publisher_->publish(moveJ_P_TargetPose);
    movej_state = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
  if(movej_p_state == true)
  {
  rm_ros_interfaces::msg::Movel moveL_TargetPose;
  moveL_TargetPose.pose.position.x = -0.255816;
  moveL_TargetPose.pose.position.y = -0.000013;
  moveL_TargetPose.pose.position.z = 0.222814;
  moveL_TargetPose.pose.orientation.x = 0.995179;
  moveL_TargetPose.pose.orientation.y = -0.094604;
  moveL_TargetPose.pose.orientation.z = -0.025721;
  moveL_TargetPose.pose.orientation.w = 0.002349;
  moveL_TargetPose.speed = 20;
  moveL_TargetPose.block = true;
  this->movel_publisher_->publish(moveL_TargetPose);
  movej_p_state = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
  if(movel_state == true)
  {
  rm_ros_interfaces::msg::Movec moveC_TargetPose;
  moveC_TargetPose.pose_mid.position.x = -0.307239;
  moveC_TargetPose.pose_mid.position.y = 0.150903;
  moveC_TargetPose.pose_mid.position.z = 0.222814;
  moveC_TargetPose.pose_mid.orientation.x = 0.995179;
  moveC_TargetPose.pose_mid.orientation.y = -0.094604;
  moveC_TargetPose.pose_mid.orientation.z = -0.025721;
  moveC_TargetPose.pose_mid.orientation.w = 0.002349;
  moveC_TargetPose.pose_end.position.x = -0.357239;
  moveC_TargetPose.pose_end.position.y = 0.000903;
  moveC_TargetPose.pose_end.position.z = 0.222814;
  moveC_TargetPose.pose_end.orientation.x = 0.995179;
  moveC_TargetPose.pose_end.orientation.y = -0.094604;
  moveC_TargetPose.pose_end.orientation.z = -0.025721;
  moveC_TargetPose.pose_end.orientation.w = 0.002349;
  moveC_TargetPose.speed = 20;
  moveC_TargetPose.loop = 1;
  moveC_TargetPose.trajectory_connect = 0;
  moveC_TargetPose.block = true;
  this->movec_publisher_->publish(moveC_TargetPose);
  movel_state = false;
  }
  
}
/***********************************************end**************************************************/


/***********************************构造函数，初始化发布器订阅器****************************************/
MoveDemoPub::MoveDemoPub():rclcpp::Node("Move_demo_pub_node")
{
  this->declare_parameter<int>("arm_dof", arm_dof_);
  this->get_parameter("arm_dof", arm_dof_);
  RCLCPP_INFO (this->get_logger(),"arm_dof is %d\n",arm_dof_);
  if(arm_dof_ == 6)
  {movej_way.joint.resize(6);}
  else if(arm_dof_ == 7)
  {movej_way.joint.resize(7);}
  movej_p_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movejp>("/rm_driver/movej_p_cmd", rclcpp::ParametersQoS());
  movel_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movel>("/rm_driver/movel_cmd", rclcpp::ParametersQoS());
  movej_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movej>("/rm_driver/movej_cmd", rclcpp::ParametersQoS());
  movec_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movec>("/rm_driver/movec_cmd", rclcpp::ParametersQoS());
  loop_pub_Timer = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&MoveDemoPub::looppub_timer_callback,this));
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
MoveDemoSub::MoveDemoSub():rclcpp::Node("Move_demo_sub_node")
{
  movej_p_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_p_result", rclcpp::ParametersQoS(), std::bind(&MoveDemoSub::MoveJPDemo_Callback, this,_1));
  movel_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movel_result", rclcpp::ParametersQoS(), std::bind(&MoveDemoSub::MoveLDemo_Callback, this,_1));
  movej_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_result", rclcpp::ParametersQoS(), std::bind(&MoveDemoSub::MoveJDemo_Callback, this,_1));
  movec_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movec_result", rclcpp::ParametersQoS(), std::bind(&MoveDemoSub::MoveCDemo_Callback, this,_1));
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node_sub = std::make_shared<MoveDemoSub>();
  auto node_pub = std::make_shared<MoveDemoPub>();
  executor.add_node(node_pub);
  executor.add_node(node_sub);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
