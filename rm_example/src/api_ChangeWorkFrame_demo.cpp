//
// Created by ubuntu on 23-11-28.
//
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class ChangeWorkFrame: public rclcpp::Node
{
  public:
    ChangeWorkFrame();                                                                    //构造函数
    void change_work_frame_programe();                                                    //改变工作坐标系函数
    void ChangeWorkState_Callback(const std_msgs::msg::Bool & msg);                       //结果回调函数
  
  private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;                       //声明发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;                   //声明订阅器
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ChangeWorkFrame::ChangeWorkState_Callback(const std_msgs::msg::Bool & msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg.data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Switching the tool coordinate system succeeded\n");
    } else {
        RCLCPP_INFO (this->get_logger(),"*******Switching the tool coordinate system Failed\n");
    }
}
/***********************************************end**************************************************/

/*******************************************更换工作坐标系函数****************************************/
void ChangeWorkFrame::change_work_frame_programe()
{
    std_msgs::msg::String change_work_frame;
    change_work_frame.data = "Base";                             //可更改工作坐标系位置
    this->publisher_->publish(change_work_frame);
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
ChangeWorkFrame::ChangeWorkFrame():rclcpp::Node("changeframe")
{
  subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/change_work_frame_result", 10, std::bind(&ChangeWorkFrame::ChangeWorkState_Callback, this,_1));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/rm_driver/change_work_frame_cmd", 10);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  change_work_frame_programe();
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChangeWorkFrame>());
  rclcpp::shutdown();
  return 0;
}
