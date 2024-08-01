//
// Created by ubuntu on 23-11-28.
//
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/movejp.hpp"
#include "rm_ros_interfaces/msg/movel.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class MoveLDemo: public rclcpp::Node
{
  public:
    MoveLDemo();                                                                          //构造函数
    void movejp_demo();                                                                   //movejp运动规划函数
    void movel_demo();                                                                    //movel运动规划函数
    void MoveJPDemo_Callback(const std_msgs::msg::Bool & msg);                            //结果回调函数
    void MoveLDemo_Callback(const std_msgs::msg::Bool & msg);                             //结果回调函数
    
  
  private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr publisher_;               //声明发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;                    //声明订阅器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;          //声明发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_subscription_;              //声明订阅器
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveLDemo::MoveJPDemo_Callback(const std_msgs::msg::Bool & msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg.data)
    {
        RCLCPP_INFO (this->get_logger(),"*******MoveJP succeeded\n");
        /*******************************************movel运动****************************************/
        sleep(1);
        rm_ros_interfaces::msg::Movel moveL_TargetPose;
        moveL_TargetPose.pose.position.x = -0.317239;
        moveL_TargetPose.pose.position.y = 0.120903;
        moveL_TargetPose.pose.position.z = 0.295765;
        moveL_TargetPose.pose.orientation.x = -0.983404;
        moveL_TargetPose.pose.orientation.y = -0.178432;
        moveL_TargetPose.pose.orientation.z = 0.032271;
        moveL_TargetPose.pose.orientation.w = 0.006129;
        moveL_TargetPose.speed = 20;
        moveL_TargetPose.trajectory_connect = 0;
        moveL_TargetPose.block = true;
        
        this->movel_publisher_->publish(moveL_TargetPose);
    } else {
        RCLCPP_INFO (this->get_logger(),"*******MoveJP Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveLDemo::MoveLDemo_Callback(const std_msgs::msg::Bool & msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg.data)
    {
        RCLCPP_INFO (this->get_logger(),"*******MoveL succeeded\n");
    } else {
        RCLCPP_INFO (this->get_logger(),"*******MoveL Failed\n");
    }
}   
/***********************************************end**************************************************/

/*******************************************movejp运动函数****************************************/
void MoveLDemo::movejp_demo()
{
    rm_ros_interfaces::msg::Movejp moveJ_P_TargetPose;
    moveJ_P_TargetPose.pose.position.x = -0.257239;
    moveJ_P_TargetPose.pose.position.y = 0.120903;
    moveJ_P_TargetPose.pose.position.z = 0.205765;
    moveJ_P_TargetPose.pose.orientation.x = -0.983404;
    moveJ_P_TargetPose.pose.orientation.y = -0.178432;
    moveJ_P_TargetPose.pose.orientation.z = 0.032271;
    moveJ_P_TargetPose.pose.orientation.w = 0.006129;
    moveJ_P_TargetPose.speed = 20;
    moveJ_P_TargetPose.trajectory_connect = 0;
    moveJ_P_TargetPose.block = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    this->publisher_->publish(moveJ_P_TargetPose);
}
/***********************************************end**************************************************/


/***********************************构造函数，初始化发布器订阅器****************************************/
MoveLDemo::MoveLDemo():rclcpp::Node("Movel_demo_node")
{

  subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_p_result", rclcpp::ParametersQoS(), std::bind(&MoveLDemo::MoveJPDemo_Callback, this,_1));
  publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movejp>("/rm_driver/movej_p_cmd", rclcpp::ParametersQoS());
  movel_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movel_result", rclcpp::ParametersQoS(), std::bind(&MoveLDemo::MoveLDemo_Callback, this,_1));
  movel_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movel>("/rm_driver/movel_cmd", rclcpp::ParametersQoS());
  sleep(1);
  movejp_demo();
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveLDemo>());
  rclcpp::shutdown();
  return 0;
}
