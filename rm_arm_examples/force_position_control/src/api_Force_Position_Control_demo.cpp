//
// Created by ubuntu on 24-7-11.
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
#include "rm_ros_interfaces/msg/setforceposition.hpp"
#include "rm_ros_interfaces/msg/movec.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
bool set_force_postion_state = false;
bool movej_p_state = false;
bool movel_state = false;
bool stop_force_postion_state = false;
bool first_run = true;
/****************************************创建类************************************/ 
class ForcePositionControlDemoSub: public rclcpp::Node
{
  public:
    ForcePositionControlDemoSub();                                                                         //构造函数
    void ForcePositionControl_demo();                                                                   //力位混合运动规划函数
    void MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                 //结果回调函数
    void SetForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                        //结果回调函数
    void MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                  //结果回调函数
    void StopForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                       //结果回调函数
    
  private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_p_subscription_;                         //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_subscription_;                           //声明订阅器
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_force_postion_subscription_;               //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_force_postion_subscription_;              //声明订阅器

};

class ForcePositionControlDemoPub: public rclcpp::Node
{
  public:
    ForcePositionControlDemoPub();                                                                      //构造函数
    void ForcePositionControl_demo();                                                                   //力位混合运动规划函数
    void looppub_timer_callback();                                                                      //move运动规划函数
    
  private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movej_p_publisher_;                    //声明发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;                       //声明发布器
    
    rclcpp::Publisher<rm_ros_interfaces::msg::Setforceposition>::SharedPtr set_force_postion_publisher_;//声明发布器
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_force_postion_publisher_;                    //声明发布器
    rclcpp::TimerBase::SharedPtr loop_pub_Timer;                                                        //定时发布器

};

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ForcePositionControlDemoSub::MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    movej_p_state = true;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Movej_p succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"*******Movej_p Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ForcePositionControlDemoSub::MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
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
void ForcePositionControlDemoSub::SetForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    set_force_postion_state = msg->data;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Set Force Postion succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"*******Set Force Postion Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ForcePositionControlDemoSub::StopForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    stop_force_postion_state = true;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Stop Force Postion succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"*******Stop Force Postion Failed\n");
    }
}   
/***********************************************end**************************************************/

/*******************************************力位混合运动函数****************************************/
void ForcePositionControlDemoPub::looppub_timer_callback()
{
  //moveJP到达指定位置 
  if(first_run ==true)
  {
    rm_ros_interfaces::msg::Movejp moveJ_P_TargetPose;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    first_run = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  //开启力位混合 
  if(movej_p_state==true)                    //等待moveJ_P到达
  {
    rm_ros_interfaces::msg::Setforceposition forceposition_data;
    forceposition_data.sensor = 1;
    forceposition_data.mode = 0;
    forceposition_data.direction = 2;
    forceposition_data.n = 5;
    // forceposition_data.block = true;
    this->set_force_postion_publisher_->publish(forceposition_data);
    movej_p_state = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  //moveL运动 
  if(set_force_postion_state==true)
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
    set_force_postion_state = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  //停止力位混合 
  if(movel_state==true)                     //等待movel到达
  {
    std_msgs::msg::Bool stop_force_postion_data;
    stop_force_postion_data.data = true;
    this->stop_force_postion_publisher_->publish(stop_force_postion_data);
    movel_state = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if(stop_force_postion_state==true)
  {
    RCLCPP_INFO (this->get_logger(),"*******All step run over\n");
    stop_force_postion_state = false;
  }
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
ForcePositionControlDemoPub::ForcePositionControlDemoPub():rclcpp::Node("Force_Position_Control_pub_node")
{
  movej_p_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movejp>("/rm_driver/movej_p_cmd", rclcpp::ParametersQoS());
  movel_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movel>("/rm_driver/movel_cmd", rclcpp::ParametersQoS());
  set_force_postion_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Setforceposition>("/rm_driver/set_force_postion_cmd", rclcpp::ParametersQoS());
  stop_force_postion_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/rm_driver/stop_force_postion_cmd", rclcpp::ParametersQoS());
  loop_pub_Timer = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&ForcePositionControlDemoPub::looppub_timer_callback,this));
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
ForcePositionControlDemoSub::ForcePositionControlDemoSub():rclcpp::Node("Force_Position_Control_sub_node")
{
  movej_p_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_p_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::MoveJPDemo_Callback, this,_1));
  movel_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movel_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::MoveLDemo_Callback, this,_1));
  set_force_postion_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/set_force_postion_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::SetForcePostionDemo_Callback, this,_1));
  stop_force_postion_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/stop_force_postion_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::StopForcePostionDemo_Callback, this,_1));
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node_sub = std::make_shared<ForcePositionControlDemoSub>();
  auto node_pub = std::make_shared<ForcePositionControlDemoPub>();
  executor.add_node(node_pub);
  executor.add_node(node_sub);
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
