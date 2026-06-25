//
// Created by ubuntu on 23-11-28.
//
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/armoriginalstate.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class GetArmState: public rclcpp::Node
{
  public:
    GetArmState();                                                                                   //构造函数
    void get_arm_state();                                                                            //改变工作坐标系函数
    void GetArmState_Callback(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg);                 //结果回调函数
  
  private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;                                   //声明发布器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armoriginalstate>::SharedPtr subscription_;         //声明订阅器
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void GetArmState::GetArmState_Callback(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg->dof == 7)
    {
      RCLCPP_INFO (this->get_logger(),"joint state is: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", msg->joint[0],msg->joint[1],msg->joint[2],msg->joint[3],msg->joint[4],msg->joint[5],msg->joint[6]);
    }
    else
    {
      RCLCPP_INFO (this->get_logger(),"joint state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n", msg->joint[0],msg->joint[1],msg->joint[2],msg->joint[3],msg->joint[4],msg->joint[5]);
    }
    RCLCPP_INFO (this->get_logger(),"pose state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n", msg->pose[0],msg->pose[1],msg->pose[2],msg->pose[3],msg->pose[4],msg->pose[5]);
    // RCLCPP_INFO (this->get_logger(),"arm_err is: %d\n",msg->arm_err);
    // RCLCPP_INFO (this->get_logger(),"sys_err is: %d\n",msg->sys_err);
}   
/***********************************************end**************************************************/

/*******************************************获取位姿函数****************************************/
void GetArmState::get_arm_state()
{
    std_msgs::msg::Empty get_state;
    this->publisher_->publish(get_state);
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
GetArmState::GetArmState():rclcpp::Node("get_state")
{
  subscription_ = this->create_subscription<rm_ros_interfaces::msg::Armoriginalstate>("/rm_driver/get_current_arm_original_state_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::GetArmState_Callback, this,_1));
  publisher_ = this->create_publisher<std_msgs::msg::Empty>("/rm_driver/get_current_arm_state_cmd", rclcpp::ParametersQoS());
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  get_arm_state();
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetArmState>());
  rclcpp::shutdown();
  return 0;
}
