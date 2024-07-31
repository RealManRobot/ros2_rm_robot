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
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class MoveJPDemo: public rclcpp::Node
{
  public:
    MoveJPDemo();                                                                                 //构造函数
    void movejp_demo();                                                                           //movejp运动规划函数
    void MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                    //结果回调函数
  
  private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movejp_publisher_;               //声明发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movejp_subscription_;                    //声明订阅器
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveJPDemo::MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******MoveJP succeeded\n");
    } else {
        RCLCPP_INFO (this->get_logger(),"*******MoveJP Failed\n");
    }
}   
/***********************************************end**************************************************/

/*******************************************获取位姿函数****************************************/
void MoveJPDemo::movejp_demo()
{

    rm_ros_interfaces::msg::Movejp moveJ_P_TargetPose;
    moveJ_P_TargetPose.pose.position.x = 0.33638298511505127;
    moveJ_P_TargetPose.pose.position.y = -0.0025470000691711903;
    moveJ_P_TargetPose.pose.position.z = 0.5196430087089539;
    moveJ_P_TargetPose.pose.orientation.x = 0.9973419904708862;
    moveJ_P_TargetPose.pose.orientation.y = -0.0021510000806301832;
    moveJ_P_TargetPose.pose.orientation.z = 0.0628110021352768;
    moveJ_P_TargetPose.pose.orientation.w = 0.03685300052165985;
    moveJ_P_TargetPose.speed = 20;
    moveJ_P_TargetPose.block = true;
    this->movejp_publisher_->publish(moveJ_P_TargetPose);
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
MoveJPDemo::MoveJPDemo():rclcpp::Node("Movejp_demo_node")
{

  movejp_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_p_result", rclcpp::ParametersQoS(), std::bind(&MoveJPDemo::MoveJPDemo_Callback, this,_1));
  movejp_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movejp>("/rm_driver/movej_p_cmd", rclcpp::ParametersQoS());
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  movejp_demo();
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveJPDemo>());
  rclcpp::shutdown();
  return 0;
}
