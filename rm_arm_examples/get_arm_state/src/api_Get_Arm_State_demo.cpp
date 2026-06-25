//
// Created by ubuntu on 24-7-10.
//
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/armoriginalstate.hpp"
#include "rm_ros_interfaces/msg/armstate.hpp"
#include "rm_ros_interfaces/msg/armsoftversion.hpp"
#include "rm_ros_interfaces/msg/sixforce.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class GetArmState: public rclcpp::Node
{
  public:
    GetArmState();                                                                                           //构造函数
    void get_arm_state();                                                                                    //改变工作坐标系函数
    void GetArmOriginalState_Callback(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg);        //机械臂状态原始结果回调函数
    void GetArmState_Callback(const rm_ros_interfaces::msg::Armstate::SharedPtr msg);                        //机械臂状态结果回调函数
    void GetArmSoftwareVersion_Callback(const rm_ros_interfaces::msg::Armsoftversion::SharedPtr msg);        //控制器版本结果回调函数
    void ForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                          //六维力传感器结果回调函数
    void ZeroForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                      //六维力结果回调函数
    void WorkForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                      //六维力工作坐标结果回调函数
    void ToolForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                      //六维力工具坐标结果回调函数


  private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm_state_publisher_;                                 //机械臂关节状态发布器
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm_software_version_publisher_;                      //机械臂软件版本发布器
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr get_force_data_publisher_;                            //机械臂六维力状态发布器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armoriginalstate>::SharedPtr subscription_original_state;   //关节原始状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armstate>::SharedPtr subscription_arm_state;                //关节状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armsoftversion>::SharedPtr subscription_software_version;   //控制器版本订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_force_data;               //六维力传感器状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_zero_force_data;          //六维力系统受力状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_work_force_data;          //六维力工作坐标系受力状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_tool_force_data;          //六维力工具坐标系受力状态订阅器
};


/************************************************机械臂状态原始结果回调********************************************/ 
void GetArmState::GetArmOriginalState_Callback(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg)
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
}   
/*******************************************************end**********************************************************/

/******************************************************机械臂状态结果回调***********************************************/ 
void GetArmState::GetArmState_Callback(const rm_ros_interfaces::msg::Armstate::SharedPtr msg)
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
    RCLCPP_INFO (this->get_logger(),"pose state is: \n[position.x = %lf,\n position.y = %lf,\n position.z = %lf,\n orientation.x = %lf,\n orientation.y = %lf,\n orientation.z = %lf,\n orientation.w = %lf]\n", msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
}   
/*******************************************************end**********************************************************/
/************************************************机械臂传感器版本结果回调函数********************************************/ 
void GetArmState::GetArmSoftwareVersion_Callback(const rm_ros_interfaces::msg::Armsoftversion::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),"Planversion is %s\n ", msg->planversion.c_str());
    RCLCPP_INFO(this->get_logger(),"Productversion is %s\n ", msg->productversion.c_str());
}
/*******************************************************end**********************************************************/
/************************************************机械臂传感器六维力结果回调函数********************************************/ 
void GetArmState::ForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"\nforce-force_fx is %f\nforce-force_fy is %f\nforce-force_fz is %f\nforce-force_mx is %f\nforce-force_my is %f\nforce-force_mz is %f\n "
  , msg->force_fx
  , msg->force_fy
  , msg->force_fz
  , msg->force_mx
  , msg->force_my
  , msg->force_mz);
}
/*******************************************************end**********************************************************/
/***********************************************机械臂六维力系统受力结果回调函数******************************************/ 
void GetArmState::ZeroForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"\nzero-force_fx is %f\nzero-force_fy is %f\nzero-force_fz is %f\nzero-force_mx is %f\nzero-force_my is %f\nzero-force_mz is %f\n"
  , msg->force_fx
  , msg->force_fy
  , msg->force_fz
  , msg->force_mx
  , msg->force_my
  , msg->force_mz);
}
/*******************************************************end**********************************************************/
/***********************************************机械臂六维力系统受力结果回调函数******************************************/ 
void GetArmState::WorkForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"\nwork-zero_fx is %f\nwork-zero_fy is %f\nwork-zero_fz is %f\nwork-zero_mx is %f\nwork-zero_my is %f\nwork-zero_mz is %f\n "
  , msg->force_fx
  , msg->force_fy
  , msg->force_fz
  , msg->force_mx
  , msg->force_my
  , msg->force_mz);
}
/*******************************************************end**********************************************************/
/***********************************************机械臂六维力系统受力结果回调函数******************************************/ 
void GetArmState::ToolForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"\ntool-zero_fx is %f\ntool-zero_fy is %f\ntool-zero_fz is %f\ntool-zero_mx is %f\ntool-zero_my is %f\ntool-zero_mz is %f\n"
  , msg->force_fx
  , msg->force_fy
  , msg->force_fz
  , msg->force_mx
  , msg->force_my
  , msg->force_mz);
}
/*******************************************************end**********************************************************/

/*******************************************获取位姿函数****************************************/
void GetArmState::get_arm_state()
{
    std_msgs::msg::Empty get_state;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    this->arm_state_publisher_->publish(get_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    this->arm_software_version_publisher_->publish(get_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    this->get_force_data_publisher_->publish(get_state);
}

/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
GetArmState::GetArmState():rclcpp::Node("get_state")
{
  subscription_original_state = this->create_subscription<rm_ros_interfaces::msg::Armoriginalstate>("rm_driver/get_current_arm_original_state_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::GetArmOriginalState_Callback, this,_1));
  subscription_arm_state = this->create_subscription<rm_ros_interfaces::msg::Armstate>("rm_driver/get_current_arm_state_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::GetArmState_Callback, this,_1));
  subscription_software_version = this->create_subscription<rm_ros_interfaces::msg::Armsoftversion>("rm_driver/get_arm_software_version_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::GetArmSoftwareVersion_Callback, this,_1));
  subscription_force_data = this->create_subscription<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_force_data_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::ForceData_Callback, this,_1));
  subscription_zero_force_data = this->create_subscription<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_zero_force_data_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::ZeroForceData_Callback, this,_1));
  subscription_work_force_data = this->create_subscription<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_work_force_data_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::WorkForceData_Callback, this,_1));
  subscription_tool_force_data = this->create_subscription<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_tool_force_data_result", rclcpp::ParametersQoS(), std::bind(&GetArmState::ToolForceData_Callback, this,_1));
  arm_state_publisher_ = this->create_publisher<std_msgs::msg::Empty>("rm_driver/get_current_arm_state_cmd", rclcpp::ParametersQoS());
  arm_software_version_publisher_ = this->create_publisher<std_msgs::msg::Empty>("rm_driver/get_arm_software_version_cmd", rclcpp::ParametersQoS());
  get_force_data_publisher_ = this->create_publisher<std_msgs::msg::Empty>("rm_driver/get_force_data_cmd", rclcpp::ParametersQoS());
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
