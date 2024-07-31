//
// Created by ubuntu on 23-11-28.
//
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class MoveJDemo: public rclcpp::Node
{
  public:
    MoveJDemo();                                                                                  //构造函数
    void movej_demo();                                                                            //发布MoveJ规划指令
    void MovejDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                     //结果回调函数
  
  private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr publisher_;                       //声明发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;                           //声明订阅器
    rm_ros_interfaces::msg::Movej movej_way;
    int arm_dof_ = 6;
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveJDemo::MovejDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Movej succeeded\n");
    } else {
        RCLCPP_INFO (this->get_logger(),"*******Movej Failed\n");
    }
}   
/***********************************************end**************************************************/

/*******************************************发布movej指令函数****************************************/
void MoveJDemo::movej_demo()
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
    this->publisher_->publish(movej_way);
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
MoveJDemo::MoveJDemo():rclcpp::Node("Movej_demo")
{
  this->declare_parameter<int>("arm_dof", arm_dof_);
  this->get_parameter("arm_dof", arm_dof_);
  RCLCPP_INFO (this->get_logger(),"arm_dof is %d\n",arm_dof_);
  if(arm_dof_ == 6)
  {movej_way.joint.resize(6);}
  else if(arm_dof_ == 7)
  {movej_way.joint.resize(7);}
  subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_result", rclcpp::ParametersQoS(), std::bind(&MoveJDemo::MovejDemo_Callback, this,_1));
  publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movej>("/rm_driver/movej_cmd", rclcpp::ParametersQoS());
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  movej_demo();
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveJDemo>());
  rclcpp::shutdown();
  return 0;
}
