// Copyright (c) 2024  RealMan Intelligent Ltd
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


#include "rm_driver.h"

using namespace std::chrono_literals;

static void my_handler(int sig)  // can be called asynchronously
{ 
    (void)sig;
    ctrl_flag = true; // set flag
}

//连接机械臂网络   
int Arm_Socket_Start_Connect(void)
{
    int Arm_Socket;                         //机械臂TCp网络通信套接字
    int Arm_connect;                        //机械臂TCP连接状态

    Arm_Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (Arm_Socket <= 0)
    {
        return 2;
    }

    struct sockaddr_in serAddr;
    // struct timeval tm;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(tcp_port);
    serAddr.sin_addr.s_addr = inet_addr(tcp_ip);
    int flag = 0;
    int old_flag = 0;
    flag |= O_NONBLOCK;
    // 设置为非阻塞模式
    old_flag = flag = fcntl(Arm_Socket, F_SETFL, O_NONBLOCK );
    // 查看连接状态
    Arm_connect = connect(Arm_Socket, (struct sockaddr *)&serAddr, sizeof(serAddr));
    // ROS_INFO("Arm_connect=%d\n",Arm_connect);
    if (Arm_connect != 0)
    {
        if(errno != EINPROGRESS) //connect返回错误。
		{
			std::cout<<"Arm_connect="<< Arm_connect <<"connect failed"<<std::endl;
            close(Arm_Socket);
            return 3;
		}
        else
        {
            struct timeval tm;  

			tm.tv_sec = 2;      
			tm.tv_usec = 0;

			fd_set wset;

			FD_ZERO(&wset);

			FD_SET(Arm_Socket,&wset); 
			int res = select(Arm_Socket+1, NULL, &wset, NULL, &tm);
            if(res <= 0)
			{
				std::cout<<"********************Connect faile check your connect!**************"<<std::endl;
				close(Arm_Socket);
				return 3;
			}

            if(FD_ISSET(Arm_Socket,&wset))
			{

				int err = -1;
				socklen_t len = sizeof(int);

				if(getsockopt(Arm_Socket, SOL_SOCKET, SO_ERROR, &err, &len ) < 0) //两种错误处理方式
				{
					std::cout<<"errno :" << errno << strerror(errno) <<std::endl;
					close(Arm_Socket);
					return 4;
				}
 
				if(err)
				{
					std::cout<<"********************Connect faile check your connect!**************"<<std::endl;
					errno = err;
					close(Arm_Socket);
					return 5;
				}
			}

        }

    }
    fcntl(Arm_Socket, F_SETFL, old_flag); //最后恢复sock的阻塞属性。
    close(Arm_Socket);
    return 0;
}

int Arm_Start(void)
{
    std::string version;
    Rm_Api.rm_init(RM_TRIPLE_MODE_E);
    //m_sockhand =  Rm_Api.Service_Arm_Socket_Start((char*)tcp_ip, tcp_port, 5000);
    version = Rm_Api.rm_api_version();
    // std::cout << version.c_str() << std::endl;
    // Rm_Api.rm_set_log_call_back(NULL ,0);
    robot_handle = Rm_Api.rm_create_robot_arm((char*)tcp_ip, tcp_port);

    
    // Rm_Api.rm_set_robot_dof(robot_handle,7);
    // robot_handle = Rm_Api.rm_create_robot_arm("192.168.1.18", 8080);
    if(robot_handle->id == -1)
    {
        //rm_delete_robot_arm(robot_handle);
        std::cout<<"arm connect err..."<<std::endl;
    }
    else if(robot_handle != NULL)
    {
        std::cout<<"connect success, arm id :"<<robot_handle->id<<std::endl;
    }
    // m_sockhand =  Rm_Api.Service_Arm_Socket_Start((char*)"192.168.1.234", 8080, 5000);
    //std::cout<<m_sockhand<<std::endl;
    return 0;
}

void Arm_Close(void)
{
    //Rm_Api.Service_Arm_Socket_Close(m_sockhand);
    Rm_Api.rm_delete_robot_arm(robot_handle);
}

void RmArm::Arm_MoveJ_Callback(rm_ros_interfaces::msg::Movej::SharedPtr msg)
{
    float joint[7];
    int v;
    int block;
    int32_t res;
    std_msgs::msg::UInt32 movej_data;
    std_msgs::msg::Bool movej_result;
    int trajectory_connect;

    for(int i = 0; i < 6; i++)
    {
        joint[i] = msg->joint[i] * RAD_DEGREE;
    }
    if(msg->dof == 7)
    {
        joint[6] = msg->joint[6] * RAD_DEGREE;
    }
    v = msg->speed;
    trajectory_connect = msg->trajectory_connect;
    block = msg->block;
    //res = Rm_Api.Service_Movej_Cmd(m_sockhand, joint, v ,0, trajectory_connect, block);
    res = Rm_Api.rm_movej(robot_handle, joint, v ,0, trajectory_connect, block);
    movej_data.data = res;
    if(movej_data.data == 0)
    {
        movej_result.data = true;
        this->MoveJ_Cmd_Result->publish(movej_result);
    }
    else
    {
        movej_result.data = false;
        this->MoveJ_Cmd_Result->publish(movej_result);
        RCLCPP_INFO (this->get_logger(),"MoveJ error code is %d\n",movej_data.data);
    }
}

void RmArm::Arm_MoveL_Callback(rm_ros_interfaces::msg::Movel::SharedPtr msg)
{
    rm_pose_t pose;
    int v;
    bool block;
    int32_t res;
    std_msgs::msg::UInt32 movel_data;
    std_msgs::msg::Bool movel_result;
    rm_quat_t rec_pose;
    rm_euler_t tarns_euler;
    int trajectory_connect;

    pose.position.x = msg->pose.position.x;
    pose.position.y = msg->pose.position.y;
    pose.position.z = msg->pose.position.z;
    rec_pose.w = msg->pose.orientation.w;
    rec_pose.x = msg->pose.orientation.x;
    rec_pose.y = msg->pose.orientation.y;
    rec_pose.z = msg->pose.orientation.z;
    // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
    tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
    pose.euler.rx = tarns_euler.rx;
    pose.euler.ry = tarns_euler.ry;
    pose.euler.rz = tarns_euler.rz;
    v = msg->speed;
    block = msg->block;
    trajectory_connect = msg->trajectory_connect;
    // res = Rm_Api.Service_Movel_Cmd(m_sockhand, pose, v ,0, trajectory_connect, block);
    res = Rm_Api.rm_movel(robot_handle, pose, v ,0, trajectory_connect, block);
    movel_data.data = res;
    if(movel_data.data == 0)
    {
        movel_result.data = true;
        this->MoveL_Cmd_Result->publish(movel_result);
    }
    else
    {
        movel_result.data = false;
        this->MoveL_Cmd_Result->publish(movel_result);
        RCLCPP_INFO (this->get_logger(),"MoveL error code is %d\n",movel_data.data);
    }
}

void RmArm::Arm_MoveC_Callback(rm_ros_interfaces::msg::Movec::SharedPtr msg)
{
    
    rm_pose_t pose_via, pose_to;
    int v,loop;
    int32_t res;
    std_msgs::msg::UInt32 movec_data;
    std_msgs::msg::Bool movec_result;
    rm_quat_t rec_pose_via, rec_pose_to;
    rm_euler_t tarns_euler_via, tarns_euler_to;
    int trajectory_connect;
    bool block;

    pose_via.position.x = msg->pose_mid.position.x;
    pose_via.position.y = msg->pose_mid.position.y;
    pose_via.position.z = msg->pose_mid.position.z;
    rec_pose_via.w = msg->pose_mid.orientation.w;
    rec_pose_via.x = msg->pose_mid.orientation.x;
    rec_pose_via.y = msg->pose_mid.orientation.y;
    rec_pose_via.z = msg->pose_mid.orientation.z;
    // tarns_euler_via = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose_via);
    tarns_euler_via = Rm_Api.rm_algo_quaternion2euler(rec_pose_via);
    pose_via.euler.rx = tarns_euler_via.rx;
    pose_via.euler.ry = tarns_euler_via.ry;
    pose_via.euler.rz = tarns_euler_via.rz;

    pose_to.position.x = msg->pose_end.position.x;
    pose_to.position.y = msg->pose_end.position.y;
    pose_to.position.z = msg->pose_end.position.z;
    rec_pose_to.w = msg->pose_end.orientation.w;
    rec_pose_to.x = msg->pose_end.orientation.x;
    rec_pose_to.y = msg->pose_end.orientation.y;
    rec_pose_to.z = msg->pose_end.orientation.z;
    // tarns_euler_to = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose_to);
    tarns_euler_to = Rm_Api.rm_algo_quaternion2euler(rec_pose_to);
    pose_to.euler.rx = tarns_euler_to.rx;
    pose_to.euler.ry = tarns_euler_to.ry;
    pose_to.euler.rz = tarns_euler_to.rz;

    v = msg->speed;
    loop = msg->loop;
    block = msg->block;
    trajectory_connect = msg->trajectory_connect;
    // res = Rm_Api.Service_Movec_Cmd(m_sockhand, pose_via, pose_to, v, 0, loop, trajectory_connect, block);
    res = Rm_Api.rm_movec(robot_handle, pose_via, pose_to, v, 0, loop, trajectory_connect, block);
    movec_data.data = res;
    if(movec_data.data == 0)
    {
        movec_result.data = true;
        this->MoveC_Cmd_Result->publish(movec_result);
    }
    else
    {
        movec_result.data = false;
        this->MoveC_Cmd_Result->publish(movec_result);
        RCLCPP_INFO (this->get_logger(),"MoveC error code is %d\n",movec_data.data);
    }
}

void RmArm::Arm_Movej_CANFD_Callback(rm_ros_interfaces::msg::Jointpos::SharedPtr msg)
{
    float joint[7];
    bool follow;
    float expand;
    int32_t res;
    int trajectory_mode;
    int radio;
    std_msgs::msg::UInt32 movej_CANFD_data;

    for(int i = 0; i < 6; i++)
    {
        joint[i] = msg->joint[i] * RAD_DEGREE;
    }
    if(msg->dof == 7)
    {
        joint[6] = msg->joint[6] * RAD_DEGREE;
    }

    follow = msg->follow;
    expand = msg->expand * RAD_DEGREE;
    trajectory_mode = trajectory_mode_;
    radio = radio_;
    //std::cout<<"Service_Movej_CANFD_With_Radio is run!!!!"<<std::endl;
    // res = Rm_Api.Service_Movej_CANFD_With_Radio(m_sockhand, joint, follow, expand, trajectory_mode, radio);
    res = Rm_Api.rm_movej_canfd(robot_handle, joint, follow, expand, trajectory_mode, radio);
    
    movej_CANFD_data.data = res;
    if(movej_CANFD_data.data != 0)
    {
        RCLCPP_INFO (this->get_logger(),"Movej CANFD error code is %d\n",movej_CANFD_data.data);
    }
}

void RmArm::Arm_Movej_CANFD_Custom_Callback(rm_ros_interfaces::msg::Jointposcustom::SharedPtr msg)
{
    float joint[7];
    bool follow;
    float expand;
    int32_t res;
    int trajectory_mode;
    int radio;
    std_msgs::msg::UInt32 movej_CANFD_data;

    for(int i = 0; i < 6; i++)
    {
        joint[i] = msg->joint[i] * RAD_DEGREE;
    }
    if(msg->dof == 7)
    {
        joint[6] = msg->joint[6] * RAD_DEGREE;
    }

    follow = msg->follow;
    expand = msg->expand * RAD_DEGREE;
    trajectory_mode = msg->trajectory_mode;
    radio = msg->radio;
    // std::cout<<"Service_Movej_CANFD_Trajectory is run!!!!"<<std::endl;
    // res = Rm_Api.Service_Movej_CANFD_With_Radio(m_sockhand, joint, follow, expand, trajectory_mode, radio);
    res = Rm_Api.rm_movej_canfd(robot_handle, joint, follow, expand, trajectory_mode, radio);

    movej_CANFD_data.data = res;
    if(movej_CANFD_data.data != 0)
    {
        RCLCPP_INFO (this->get_logger(),"Movej CANFD error code is %d\n",movej_CANFD_data.data);
    }
}

void RmArm::Arm_Movep_CANFD_Callback(rm_ros_interfaces::msg::Cartepos::SharedPtr msg)
{
    
    rm_pose_t pose;
    bool follow;
    int32_t res;
    std_msgs::msg::UInt32 movep_CANFD_data;
    rm_quat_t rec_pose;
    rm_euler_t tarns_euler;
    int trajectory_mode;
    int radio;

    pose.position.x = msg->pose.position.x;
    pose.position.y = msg->pose.position.y;
    pose.position.z = msg->pose.position.z;
    rec_pose.w = msg->pose.orientation.w;
    rec_pose.x = msg->pose.orientation.x;
    rec_pose.y = msg->pose.orientation.y;
    rec_pose.z = msg->pose.orientation.z;
    // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
    tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
    pose.euler.rx = tarns_euler.rx;
    pose.euler.ry = tarns_euler.ry;
    pose.euler.rz = tarns_euler.rz;
    follow = msg->follow;
    trajectory_mode = trajectory_mode_;
    radio = radio_;

    // res = Rm_Api.Service_Movep_CANFD_With_Radio(m_sockhand, pose, follow, trajectory_mode, radio);
    res = Rm_Api.rm_movep_canfd(robot_handle, pose, follow, trajectory_mode, radio);

    movep_CANFD_data.data = res;
    if(movep_CANFD_data.data != 0)
    {
        RCLCPP_INFO (this->get_logger(),"Movep CANFD error code is %d\n",movep_CANFD_data.data);
    }
}

void RmArm::Arm_Movep_CANFD_Custom_Callback(rm_ros_interfaces::msg::Carteposcustom::SharedPtr msg)
{
    rm_pose_t pose;
    bool follow;
    int32_t res;
    std_msgs::msg::UInt32 movep_CANFD_data;
    rm_quat_t rec_pose;
    rm_euler_t tarns_euler;
    int trajectory_mode;
    int radio;

    pose.position.x = msg->pose.position.x;
    pose.position.y = msg->pose.position.y;
    pose.position.z = msg->pose.position.z;
    rec_pose.w = msg->pose.orientation.w;
    rec_pose.x = msg->pose.orientation.x;
    rec_pose.y = msg->pose.orientation.y;
    rec_pose.z = msg->pose.orientation.z;
    // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
    tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
    pose.euler.rx = tarns_euler.rx;
    pose.euler.ry = tarns_euler.ry;
    pose.euler.rz = tarns_euler.rz;
    follow = msg->follow;
    trajectory_mode = msg->trajectory_mode;
    radio = msg->radio;
    std::cout<<"Service_Movep_CANFD_Trajectory is run!!!!"<<std::endl;
    // res = Rm_Api.Service_Movep_CANFD_With_Radio(m_sockhand, pose, follow, trajectory_mode, radio);
    res = Rm_Api.rm_movep_canfd(robot_handle, pose, follow, trajectory_mode, radio);
    movep_CANFD_data.data = res;
    if(movep_CANFD_data.data != 0)
    {
        RCLCPP_INFO (this->get_logger(),"Movep CANFD error code is %d\n",movep_CANFD_data.data);
    }
}

void RmArm::Arm_MoveJ_P_Callback(rm_ros_interfaces::msg::Movejp::SharedPtr msg)
{
    
    rm_pose_t pose;
    int v;
    bool block;
    int32_t res;
    std_msgs::msg::UInt32 movej_p_data;
    std_msgs::msg::Bool movej_p_result;
    rm_quat_t rec_pose;
    rm_euler_t tarns_euler;
    int trajectory_connect;

    pose.position.x = msg->pose.position.x;
    pose.position.y = msg->pose.position.y;
    pose.position.z = msg->pose.position.z;
    rec_pose.w = msg->pose.orientation.w;
    rec_pose.x = msg->pose.orientation.x;
    rec_pose.y = msg->pose.orientation.y;
    rec_pose.z = msg->pose.orientation.z;
    // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
    tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
    pose.euler.rx = tarns_euler.rx;
    pose.euler.ry = tarns_euler.ry;
    pose.euler.rz = tarns_euler.rz;
    v = msg->speed;
    trajectory_connect = msg->trajectory_connect;
    block = msg->block;
    // res = Rm_Api.Service_Movej_P_Cmd(m_sockhand, pose, v ,0, trajectory_connect, block);
    res = Rm_Api.rm_movej_p(robot_handle, pose, v ,0, trajectory_connect, block);
    movej_p_data.data = res;
    if(movej_p_data.data == 0)
    {
        movej_p_result.data = true;
        this->MoveJ_P_Cmd_Result->publish(movej_p_result);
    }
    else
    {
        movej_p_result.data = false;
        this->MoveJ_P_Cmd_Result->publish(movej_p_result);
        RCLCPP_INFO (this->get_logger(),"Movej_p error code is %d\n",movej_p_data.data);
    }
}

void RmArm::Arm_Move_Stop_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    copy = msg;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 move_stop_data;
    std_msgs::msg::Bool move_stop_result;

    // block = msg->data;
    // res = Rm_Api.Service_Move_Stop_Cmd(m_sockhand, block);
    res = Rm_Api.rm_set_arm_stop(robot_handle);
    move_stop_data.data = res;
    if(move_stop_data.data == 0)
    {
        move_stop_result.data = true;
        this->Move_Stop_Cmd_Result->publish(move_stop_result);
    }
    else
    {
        move_stop_result.data = false;
        this->Move_Stop_Cmd_Result->publish(move_stop_result);
        RCLCPP_INFO (this->get_logger(),"Move stop error code is %d\n",move_stop_data.data);
    }
}

void RmArm::Set_Joint_Teach_Callback(rm_ros_interfaces::msg::Jointteach::SharedPtr msg)
{
    int num;
    int direction;
    int v;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 joint_teach_data;
    std_msgs::msg::Bool joint_teach_result;

    num = msg->num;
    direction = msg->direction;
    v = msg->speed;
    // block = msg->block;

    // res = Rm_Api.Service_Joint_Teach_Cmd(m_sockhand, num, direction, v , block);
    res = Rm_Api.rm_set_joint_teach(robot_handle, num, direction, v);
    joint_teach_data.data = res;
    if(joint_teach_data.data == 0)
    {
        joint_teach_result.data = true;
        this->Set_Joint_Teach_Cmd_Result->publish(joint_teach_result);
    }
    else
    {
        joint_teach_result.data = false;
        this->Set_Joint_Teach_Cmd_Result->publish(joint_teach_result);
        RCLCPP_INFO (this->get_logger(),"Joint_Teach error code is %d\n",joint_teach_data.data);
    }
}

void RmArm::Set_Pos_Teach_Callback(rm_ros_interfaces::msg::Posteach::SharedPtr msg)
{
    int type;
    int direction;
    int v;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 pos_teach_data;
    std_msgs::msg::Bool pos_teach_result;

    type = msg->type;
    direction = msg->direction;
    v = msg->speed;
    // block = msg->block;

    // res = Rm_Api.Service_Pos_Teach_Cmd(m_sockhand, (POS_TEACH_MODES)type, direction, v , block);
    res = Rm_Api.rm_set_pos_teach(robot_handle, (rm_pos_teach_type_e)type, direction, v);
    pos_teach_data.data = res;
    if(pos_teach_data.data == 0)
    {
        pos_teach_result.data = true;
        this->Set_Pos_Teach_Cmd_Result->publish(pos_teach_result);
    }
    else
    {
        pos_teach_result.data = false;
        this->Set_Pos_Teach_Cmd_Result->publish(pos_teach_result);
        RCLCPP_INFO (this->get_logger(),"Pos_Teach error code is %d\n",pos_teach_data.data);
    }
}

void RmArm::Set_Ort_Teach_Callback(rm_ros_interfaces::msg::Ortteach::SharedPtr msg)
{
    int type;
    int direction;
    int v;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 ort_teach_data;
    std_msgs::msg::Bool ort_teach_result;

    type = msg->type;
    direction = msg->direction;
    v = msg->speed;
    // block = msg->block;

    // res = Rm_Api.Service_Ort_Teach_Cmd(m_sockhand, (ORT_TEACH_MODES)type, direction, v , block);
    res = Rm_Api.rm_set_ort_teach(robot_handle, (rm_ort_teach_type_e)type, direction, v );
    ort_teach_data.data = res;
    if(ort_teach_data.data == 0)
    {
        ort_teach_result.data = true;
        this->Set_Ort_Teach_Cmd_Result->publish(ort_teach_result);
    }
    else
    {
        ort_teach_result.data = false;
        this->Set_Ort_Teach_Cmd_Result->publish(ort_teach_result);
        RCLCPP_INFO (this->get_logger(),"Ort_Teach error code is %d\n",ort_teach_data.data);
    }
}

void RmArm::Set_Stop_Teach_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    copy = msg;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 stop_teach_data;
    std_msgs::msg::Bool stop_teach_result;

    // block = msg->data;

    // res = Rm_Api.Service_Teach_Stop_Cmd(m_sockhand, block);
    res = Rm_Api.rm_set_stop_teach(robot_handle);
    stop_teach_data.data = res;
    if(stop_teach_data.data == 0)
    {
        stop_teach_result.data = true;
        this->Set_Stop_Teach_Cmd_Result->publish(stop_teach_result);
    }
    else
    {
        stop_teach_result.data = false;
        this->Set_Stop_Teach_Cmd_Result->publish(stop_teach_result);
        RCLCPP_INFO (this->get_logger(),"Stop_Teach error code is %d\n",stop_teach_data.data);
    }
}

void RmArm::Arm_Get_Realtime_Push_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    int32_t res;
    rm_ros_interfaces::msg::Setrealtimepush Setrealtime_msg;
    copy = msg;
    rm_realtime_push_config_t config;
    // res = Rm_Api.Service_Get_Realtime_Push(m_sockhand, &config);
    res = Rm_Api.rm_get_realtime_push(robot_handle, &config);
    if(res == 0)
    {
        Setrealtime_msg.cycle = config.cycle;
        Setrealtime_msg.port = config.port;
        Setrealtime_msg.force_coordinate = config.force_coordinate;
        Setrealtime_msg.ip = config.ip;
        Setrealtime_msg.hand_enable = config.custom_config.hand_state;
        Setrealtime_msg.joint_speed_enable = config.custom_config.joint_speed;
        Setrealtime_msg.lift_state_enable = config.custom_config.lift_state;
        Setrealtime_msg.expand_state_enable = config.custom_config.expand_state;
        Setrealtime_msg.arm_current_status_enable = config.custom_config.arm_current_status;
        Setrealtime_msg.aloha_state_enable = config.custom_config.aloha_state;
        Setrealtime_msg.plus_base_enable = config.custom_config.plus_base;
        rm_plus_base_g = config.custom_config.plus_base;
        Setrealtime_msg.plus_state_enable = config.custom_config.plus_state;
        rm_plus_state_g = config.custom_config.plus_state;
        udp_hand_g = config.custom_config.hand_state;
        this->Get_Realtime_Push_Result->publish(Setrealtime_msg);
    }
    else
    RCLCPP_INFO (this->get_logger(),"The error code is %d\n",res);
}

void RmArm::Arm_Set_Realtime_Push_Callback(const rm_ros_interfaces::msg::Setrealtimepush::SharedPtr msg)
{
    rm_realtime_push_config_t config;
    int32_t res;
    std_msgs::msg::Bool set_realtime_result;
    config.port = msg->port ;
    config.cycle = msg->cycle;
    config.force_coordinate = msg->force_coordinate;
    config.enable = true;
    strcpy(config.ip,msg->ip.data());
    rm_udp_custom_config_t config_enable;
    config_enable.expand_state = msg->expand_state_enable;
    config_enable.hand_state = msg->hand_enable;
    udp_hand_g = msg->hand_enable;
    config_enable.joint_speed = msg->joint_speed_enable;
    config_enable.lift_state = msg->lift_state_enable;
    config_enable.arm_current_status = msg->arm_current_status_enable;
    config_enable.aloha_state = msg->aloha_state_enable;
    config_enable.plus_base = msg->plus_base_enable;
    rm_plus_base_g = msg->plus_base_enable;
    config_enable.plus_state = msg->plus_state_enable;
    rm_plus_state_g = msg->plus_state_enable;
    config.custom_config = config_enable;
    // res = Rm_Api.Service_Set_Realtime_Push(m_sockhand, config);
    res = Rm_Api.rm_set_realtime_push(robot_handle, config);
    if(res == 0)
    {
        set_realtime_result.data = true;
        this->Set_Realtime_Push_Result->publish(set_realtime_result);
    }
    else
    {
        set_realtime_result.data = false;
        this->Set_Realtime_Push_Result->publish(set_realtime_result);
        RCLCPP_INFO (this->get_logger(),"The error code is %d\n",res);
    }
}

void RmArm::Set_UDP_Configuration(int udp_cycle, int udp_port, int udp_force_coordinate, std::string udp_ip, bool hand, bool rm_plus_base, bool rm_plus_state)
{
    int32_t res;
    rm_realtime_push_config_t config;
    config.port = udp_port ;
    config.cycle = udp_cycle/5;
    config.force_coordinate = udp_force_coordinate;
    config.enable = true;
    strcpy(config.ip,udp_ip.data());
    rm_udp_custom_config_t config_enable;
    config_enable.expand_state = 0;
    config_enable.hand_state = hand;
    udp_hand_g = hand;
    config_enable.joint_speed = 0;
    config_enable.lift_state = 0;
    config_enable.aloha_state = 0;
    config_enable.plus_base = rm_plus_base;
    rm_plus_base_g = rm_plus_base;
    config_enable.plus_state = rm_plus_state;
    rm_plus_state_g = rm_plus_state;
    config_enable.arm_current_status = 0;
    config.custom_config = config_enable;
    // res = Rm_Api.Service_Set_Realtime_Push(m_sockhand, config);
    res = Rm_Api.rm_set_realtime_push(robot_handle, config);
    if(res == 0)
    {
        RCLCPP_INFO (this->get_logger(),"UDP_Configuration is cycle:%dms,port:%d,force_coordinate:%d,ip:%s,hand:%d,rm_plus_base:%d,rm_plus_state:%d\n", udp_cycle, udp_port, udp_force_coordinate, udp_ip.c_str(),udp_hand_g,rm_plus_base_g,rm_plus_state_g);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"The error code is %d\n",res);
    }
}

void RmArm::Get_Arm_Version()
{
    // ArmSoftwareInfo arm_software_info;
    rm_arm_software_version_t arm_software_info;
    char product_version[100];
    int32_t res;
    //res = Rm_Api.Service_Get_Arm_Software_Info(m_sockhand, &arm_software_info);
    res = Rm_Api.rm_get_arm_software_info(robot_handle, &arm_software_info);
    if(res == 0)
    {
        RCLCPP_INFO (this->get_logger(),"product_version = %s",arm_software_info.product_version);
        strcpy(product_version, arm_software_info.product_version);
        Udp_RM_Joint.control_version = 1;
        for(int i=0;i<10;i++)
        {
            if(product_version[i]=='F')
            {
                Udp_RM_Joint.control_version = 2;
            }
        }
        // RCLCPP_INFO (this->get_logger(),"control_version = %d",Udp_RM_Joint.control_version);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Service_Get_Arm_Software_Version error = %d",res);
    }
}

// 机械臂固件版本信息查询，在Api中去掉了
// void RmArm::Arm_Get_Arm_Software_Version_Callback(const std_msgs::msg::Empty::SharedPtr msg)
// {
//     char plan_version[50];
//     char ctrl_version[50];
//     char kernal1[50];
//     char kernal2[50];
//     char product_version[50];
//     int32_t res;
//     rm_ros_interfaces::msg::Armsoftversion Armsoftversion_msg;
//     copy = msg;
//     res = Rm_Api.Service_Get_Arm_Software_Version(m_sockhand, plan_version, ctrl_version, kernal1, kernal2, product_version);
//     if(res == 0)
//     {
//         Armsoftversion_msg.planversion = plan_version;
//         Armsoftversion_msg.ctrlversion = ctrl_version;
//         Armsoftversion_msg.kernal1 = kernal1;
//         Armsoftversion_msg.kernal2 = kernal2;
//         Armsoftversion_msg.productversion = product_version;
//         this->Get_Arm_Software_Version_Result->publish(Armsoftversion_msg);
//     }
//     else
//     {
//         RCLCPP_INFO (this->get_logger(),"The error code is %d\n",res);
//     }

// }

void RmArm::Arm_Start_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{

    int32_t res;
    std_msgs::msg::Bool arm_start_force_result;
    copy = msg;
    // res = Rm_Api.Service_Start_Force_Position_Move(m_sockhand, true);
    res = Rm_Api.rm_start_force_position_move(robot_handle);
    if(res == 0)
    {
        arm_start_force_result.data = true;
        this->Start_Force_Position_Move_Result->publish(arm_start_force_result);
    }
    else
    {
        arm_start_force_result.data = false;
        this->Start_Force_Position_Move_Result->publish(arm_start_force_result);
    }
}

void RmArm::Arm_Stop_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    int32_t res;
    std_msgs::msg::Bool arm_stop_force_result;
    copy = msg;
    // res = Rm_Api.Service_Stop_Force_Position_Move(m_sockhand, true);
    res = Rm_Api.rm_stop_force_position_move(robot_handle);
    if(res == 0)
    {
        arm_stop_force_result.data = true;
        this->Stop_Force_Position_Move_Result->publish(arm_stop_force_result);
    }
    else
    {
        arm_stop_force_result.data = false;
        this->Stop_Force_Position_Move_Result->publish(arm_stop_force_result);
    }
}

void RmArm::Arm_Force_Position_Move_Joint_Callback(const rm_ros_interfaces::msg::Forcepositionmovejoint::SharedPtr msg)
{
    int32_t res;
    float joint[7];
    int sensor;
    int mode;
    std_msgs::msg::Bool force_position_move_joint_result;
    int dir;
    float force;
    bool follow;
    for(int i = 0;i<6;i++)
    {
        joint[i] = msg->joint[i]  * RAD_DEGREE;
    }
    if(msg->dof == 7)
    {
        joint[6] = msg->joint[6]  * RAD_DEGREE;
    }

    sensor = msg->sensor;
    mode = msg->mode;
    dir = msg->dir;
    force = msg->force;
    follow = msg->follow;
    // res = Rm_Api.Service_Force_Position_Move_Joint(m_sockhand, joint, sensor, mode, dir, force, follow);
    res = Rm_Api.rm_force_position_move_joint(robot_handle, joint, sensor, mode, dir, force, follow);
    if(res != 0)
    {
        RCLCPP_INFO (this->get_logger(),"Arm force position move joint error code is %d\n",res);
    }
}

// void RmArm::Arm_Force_Position_Move_Joint_75_Callback(const rm_ros_interfaces::msg::Forcepositionmovejoint75::SharedPtr msg)
// {
//     int32_t res;
//     float joint[7];
//     byte sensor;
//     byte mode;
//     std_msgs::msg::Bool force_position_move_joint_result;
//     int dir;
//     float force;
//     bool follow;
//     for(int i = 0;i<7;i++)
//     {
//         joint[i] = msg->joint[i]  * RAD_DEGREE;
//     }
//     sensor = msg->sensor;
//     mode = msg->mode;
//     dir = msg->dir;
//     force = msg->force;
//     follow = msg->follow;
//     res = Rm_Api.Service_Force_Position_Move_Joint(m_sockhand, joint, sensor, mode, dir, force, follow);
//     if(res != 0)
//     {
//         RCLCPP_INFO (this->get_logger(),"Arm force position move joint error code is %d\n",res);
//     }
// }

void RmArm::Arm_Force_Position_Move_Pose_Callback(const rm_ros_interfaces::msg::Forcepositionmovepose::SharedPtr msg)
{
    int32_t res;
    rm_pose_t joint_pose;
    int sensor;
    int mode;
    rm_quat_t qua;
    rm_euler_t euler;
    int dir;
    float force;
    bool follow;
    qua.w = msg->pose.orientation.w;
    qua.x = msg->pose.orientation.x;
    qua.y = msg->pose.orientation.y;
    qua.z = msg->pose.orientation.z;
    // euler = Rm_Api.Service_Algo_Quaternion2Euler(qua);
    euler = Rm_Api.rm_algo_quaternion2euler(qua);
    joint_pose.position.x = msg->pose.position.x;
    joint_pose.position.y = msg->pose.position.y;
    joint_pose.position.z = msg->pose.position.z;
    joint_pose.euler.rx = euler.rx;
    joint_pose.euler.ry = euler.ry;
    joint_pose.euler.rz = euler.rz;
    sensor = msg->sensor;
    mode = msg->mode;
    dir = msg->dir;
    force = msg->force;
    follow = msg->follow;
    // res = Rm_Api.Service_Force_Position_Move_Pose(m_sockhand, joint_pose, sensor, mode, dir, force, follow);
    res = Rm_Api.rm_force_position_move_pose(robot_handle, joint_pose, sensor, mode, dir, force, follow);
    if(res != 0)
    {
        RCLCPP_INFO (this->get_logger(),"Arm force position move pose error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Force_Postion_Callback(const rm_ros_interfaces::msg::Setforceposition::SharedPtr msg)
{
    int32_t res;
    std_msgs::msg::Bool arm_set_force_postion_result;
    int sensor;
    int mode;
    int direction;
    int N;
    // bool block;
    sensor = msg->sensor;
    mode = msg->mode;
    direction = msg->direction;
    N = msg->n;
    // block = msg->block;
    // res = Rm_Api.Service_Set_Force_Postion(m_sockhand, sensor, mode, direction, N, block);
    res = Rm_Api.rm_set_force_position(robot_handle, sensor, mode, direction, N);
    if(res == 0)
    {
        arm_set_force_postion_result.data = true;
        this->Set_Force_Postion_Result->publish(arm_set_force_postion_result);
    }
    else
    {
        arm_set_force_postion_result.data = false;
        this->Set_Force_Postion_Result->publish(arm_set_force_postion_result);
        RCLCPP_INFO (this->get_logger(),"Arm_set_force_postion_callback error code is %d\n",res);
    }
}
void RmArm::Arm_Stop_Force_Postion_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    copy = msg;
    int32_t res;
    std_msgs::msg::Bool arm_stop_force_postion_result;
    // bool block;
    // block = msg->data;
    // res = Rm_Api.Service_Stop_Force_Postion(m_sockhand, block);
    res = Rm_Api.rm_stop_force_position(robot_handle);
    if(res == 0)
    {
        arm_stop_force_postion_result.data = true;
        this->Stop_Force_Postion_Result->publish(arm_stop_force_postion_result);
    }
    else
    {
        arm_stop_force_postion_result.data = false;
        this->Stop_Force_Postion_Result->publish(arm_stop_force_postion_result);
        RCLCPP_INFO (this->get_logger(),"Arm stop force postion error code is %d\n",res);
    }
}

void RmArm::Arm_Change_Work_Frame_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    //FRAME_NAME work_frame;
    const char *work_name=msg->data.c_str();
    int32_t res;
    std_msgs::msg::Bool arm_change_work_frame_result;
    // strcpy(*work_name, msg->data.c_str());
    // res = Rm_Api.Service_Change_Work_Frame(m_sockhand, work_frame.name, RM_BLOCK);
    res = Rm_Api.rm_change_work_frame(robot_handle, work_name);
    if(res == 0)
    {
        arm_change_work_frame_result.data = true;
        this->Change_Work_Frame_Result->publish(arm_change_work_frame_result);
    }
    else
    {
        arm_change_work_frame_result.data = false;
        this->Change_Work_Frame_Result->publish(arm_change_work_frame_result);
        RCLCPP_INFO (this->get_logger(),"Arm_change_work_frame_callback error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Curr_WorkFrame_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    //FRAME frame;
    rm_frame_t work_frame;
    int32_t res;
    std_msgs::msg::String curr_frame;
    std_msgs::msg::Bool arm_change_work_frame_result;
    copy = msg;
    // memset(frame.frame_name.name,'\0',sizeof(frame.frame_name.name));
    // res = Rm_Api.Service_Get_Current_Work_Frame(m_sockhand, &frame);
    res = Rm_Api.rm_get_current_work_frame(robot_handle, &work_frame);
    if(res == 0)
    {
        curr_frame.data = work_frame.frame_name;
        this->Get_Curr_WorkFrame_Result->publish(curr_frame);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm_get_curr_workFrame_callback error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Current_Tool_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    //FRAME frame;
    rm_frame_t tool_frame;
    int32_t res;
    std_msgs::msg::String curr_frame;
    std_msgs::msg::Bool arm_change_work_frame_result;
    copy = msg;
    //memset(frame.frame_name.name,'\0',sizeof(frame.frame_name.name));
    // res = Rm_Api.Service_Get_Current_Tool_Frame(m_sockhand, &frame);
    res = Rm_Api.rm_get_current_tool_frame(robot_handle, &tool_frame);
    if(res == 0)
    {
        curr_frame.data = tool_frame.frame_name;
        this->Get_Current_Tool_Frame_Result->publish(curr_frame);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm_get_curr_workFrame_callback error code is %d\n",res);
    }
}

void RmArm::Arm_Get_All_Tool_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    // FRAME_NAME name[10];
    rm_frame_name_t frame_names[10];
    int32_t res;
    rm_ros_interfaces::msg::Getallframe all_tool_frame;
    int len=-1;
    copy = msg;
    int i;
    res = Rm_Api.rm_get_total_tool_frame(robot_handle, frame_names, &len);
    if(res == 0 && len <= 10)
    {
        for(i = 0;i<len;i++)
        {
            // RCLCPP_INFO (this->get_logger(),"Arm all tool frame is %s\n",frame_names[i].name);
            all_tool_frame.frame_name[i] =std::string(frame_names[i].name);
        }
        for(i = len;i<10;i++)
        {
            all_tool_frame.frame_name[i] = "";
        }
        this->Get_All_Tool_Frame_Result->publish(all_tool_frame);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm_get_all_tool_frame_callback error code is %d\n",res);
    }
}

void RmArm::Arm_Get_All_Work_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    // char name[10];
    rm_frame_name_t frame_names[10]; 
    int32_t res;
    rm_ros_interfaces::msg::Getallframe all_work_frame;
    int len=-1;
    int i;
    // for(i = 0;i<=9;i++)
    // {
    //     memset(name[i].name,'\0',sizeof(name[i].name));
    // }
    copy = msg;
    // res = Rm_Api.Service_Get_All_Work_Frame(m_sockhand, name, &len);
    res = Rm_Api.rm_get_total_work_frame(robot_handle, frame_names, &len);
    if(res == 0 && len <= 10)
    {
        for(i = 0;i<=len;i++)
        {
            // RCLCPP_INFO (this->get_logger(),"Arm all work frame is %s\n",frame_names[i].name);
            all_work_frame.frame_name[i] = std::string(frame_names[i].name);
        }
        for(i = len;i<10;i++)
        {
            // RCLCPP_INFO (this->get_logger(),"Arm all work frame is %s\n",frame_names[i].name);
            all_work_frame.frame_name[i] = "";
        }
        this->Get_All_Work_Frame_Result->publish(all_work_frame);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm_get_all_work_frame_callback error code is %d\n",res);
    }
}


void RmArm::Arm_Set_Tool_Voltage_Callback(const std_msgs::msg::UInt16::SharedPtr msg)
{
    int type;
    int32_t res;
    std_msgs::msg::Bool arm_set_tool_voltage_result;
    type = msg->data;
    // res = Rm_Api.Service_Set_Tool_Voltage(m_sockhand, type, RM_BLOCK);
    res = Rm_Api.rm_set_tool_voltage(robot_handle, type);
    if(res == 0)
    {
        arm_set_tool_voltage_result.data = true;
        this->Set_Tool_Voltage_Result->publish(arm_set_tool_voltage_result);
    }
    else
    {
        arm_set_tool_voltage_result.data = false;
        this->Set_Tool_Voltage_Result->publish(arm_set_tool_voltage_result);
        RCLCPP_INFO (this->get_logger(),"Arm set tool voltage error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Joint_Err_Clear_Callback(const rm_ros_interfaces::msg::Jointerrclear::SharedPtr msg)
{
    int joint_num;
    // bool block;
    int32_t res;
    std_msgs::msg::Bool set_joint_err_clear_result;
    joint_num = msg->joint_num;
    // block = msg->block;
    // res = Rm_Api.Service_Set_Joint_Err_Clear(m_sockhand, joint_num, block);
    res = Rm_Api.rm_set_joint_clear_err(robot_handle, joint_num);

    if(res == 0)
    {
        set_joint_err_clear_result.data = true;
        this->Set_Joint_Err_Clear_Result->publish(set_joint_err_clear_result);
    }
    else
    {
        set_joint_err_clear_result.data = false;
        this->Set_Joint_Err_Clear_Result->publish(set_joint_err_clear_result);
        RCLCPP_INFO (this->get_logger(),"Arm set joint err clear callback error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Gripper_Pick_On_Callback(const rm_ros_interfaces::msg::Gripperpick::SharedPtr msg)
{
    int speed;
    int force;
    bool block;
    int timeout;
    int32_t res;
    std_msgs::msg::Bool set_gripper_pick_on_result;
    speed = msg->speed;
    force = msg->force;
    block = msg->block;
    timeout = msg->timeout;
    // res = Rm_Api.Service_Set_Gripper_Pick_On(m_sockhand, speed, force, block, timeout);
    res = Rm_Api.rm_set_gripper_pick_on(robot_handle, speed, force, block, timeout);
    if(res == 0)
    {
        set_gripper_pick_on_result.data = true;
        this->Set_Gripper_Pick_On_Result->publish(set_gripper_pick_on_result);
    }
    else
    {
        set_gripper_pick_on_result.data = false;
        this->Set_Gripper_Pick_On_Result->publish(set_gripper_pick_on_result);
        RCLCPP_INFO (this->get_logger(),"Arm set gripper pick on error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Gripper_Pick_Callback(const rm_ros_interfaces::msg::Gripperpick::SharedPtr msg)
{
    int speed;
    int force;
    bool block;
    int timeout;
    int32_t res;
    std_msgs::msg::Bool set_gripper_pick_result;
    speed = msg->speed;
    force = msg->force;
    block = msg->block;
    timeout = msg->timeout;
    // res = Rm_Api.Service_Set_Gripper_Pick(m_sockhand, speed, force, block, timeout);
    res = Rm_Api.rm_set_gripper_pick(robot_handle, speed, force, block, timeout);
    if(res == 0)
    {
        set_gripper_pick_result.data = true;
        this->Set_Gripper_Pick_Result->publish(set_gripper_pick_result);
    }
    else
    {
        set_gripper_pick_result.data = false;
        this->Set_Gripper_Pick_Result->publish(set_gripper_pick_result);
        RCLCPP_INFO (this->get_logger(),"Arm set gripper pick error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Gripper_Position_Callback(const rm_ros_interfaces::msg::Gripperset::SharedPtr msg)
{
    int position;
    bool block;
    int timeout;
    int32_t res;
    std_msgs::msg::Bool set_gripper_position_result;
    position = msg->position;
    block = msg->block;
    timeout = msg->timeout;
    // res = Rm_Api.Service_Set_Gripper_Position(m_sockhand, position, block, timeout);
    res = Rm_Api.rm_set_gripper_position(robot_handle, position, block, timeout);
    if(res == 0)
    {
        set_gripper_position_result.data = true;
        this->Set_Gripper_Position_Result->publish(set_gripper_position_result);
    }
    else
    {
        set_gripper_position_result.data = false;
        this->Set_Gripper_Position_Result->publish(set_gripper_position_result);
        RCLCPP_INFO (this->get_logger(),"Arm set gripper position error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Posture_Callback(const rm_ros_interfaces::msg::Handposture::SharedPtr msg)
{
    int posture_num;
    bool block;
    int timeout;
    int32_t res;
    std_msgs::msg::Bool set_hand_posture_result;
    posture_num = msg->posture_num;
    block = msg->block;
    timeout = msg->timeout;
    // res = Rm_Api.Service_Set_Hand_Posture(m_sockhand, posture_num, block);
    res = Rm_Api.rm_set_hand_posture(robot_handle, posture_num, block, timeout);
    if(res == 0)
    {
        set_hand_posture_result.data = true;
        this->Set_Hand_Posture_Result->publish(set_hand_posture_result);
    }
    else
    {
        set_hand_posture_result.data = false;
        this->Set_Hand_Posture_Result->publish(set_hand_posture_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand posture error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Seq_Callback(const rm_ros_interfaces::msg::Handseq::SharedPtr msg)
{
    int seq_num;
    bool block;
    int timeout;
    int32_t res;
    std_msgs::msg::Bool set_hand_seq_result;
    seq_num = msg->seq_num;
    block = msg->block;
    timeout = msg->timeout;
    // res = Rm_Api.Service_Set_Hand_Seq(m_sockhand, seq_num, block);
    res = Rm_Api.rm_set_hand_seq(robot_handle, seq_num, block, timeout);
    if(res == 0)
    {
        set_hand_seq_result.data = true;
        this->Set_Hand_Seq_Result->publish(set_hand_seq_result);
    }
    else
    {
        set_hand_seq_result.data = false;
        this->Set_Hand_Seq_Result->publish(set_hand_seq_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand seq error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Angle_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg)
{
    int angle[6];
    // bool block;
    int32_t res;
    std_msgs::msg::Bool set_hand_angle_result;
    for(int i = 0;i<6;i++)
    {
        angle[i] = msg->hand_angle[i];
    }
    // block = msg->block;
    //res = Rm_Api.Service_Set_Hand_Angle(m_sockhand, angle, block);
    res = Rm_Api.rm_set_hand_angle(robot_handle, angle);
    
    if(res == 0)
    {
        set_hand_angle_result.data = true;
        this->Set_Hand_Angle_Result->publish(set_hand_angle_result);
    }
    else
    {
        set_hand_angle_result.data = false;
        this->Set_Hand_Angle_Result->publish(set_hand_angle_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand angle error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Speed_Callback(const rm_ros_interfaces::msg::Handspeed::SharedPtr msg)
{
    int speed;
    //bool block;
    int32_t res;
    std_msgs::msg::Bool set_hand_speed_result;
    speed = msg->hand_speed;
    // block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Speed(m_sockhand, speed, block);
    res = Rm_Api.rm_set_hand_speed(robot_handle, speed);
    if(res == 0)
    {
        set_hand_speed_result.data = true;
        this->Set_Hand_Speed_Result->publish(set_hand_speed_result);
    }
    else
    {
        set_hand_speed_result.data = false;
        this->Set_Hand_Speed_Result->publish(set_hand_speed_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand speed error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Force_Callback(const rm_ros_interfaces::msg::Handforce::SharedPtr msg)
{
    int force;
    // bool block;
    int32_t res;
    std_msgs::msg::Bool set_hand_force_result;
    force = msg->hand_force;
    // block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Force(m_sockhand, force, block);
    res = Rm_Api.rm_set_hand_force(robot_handle, force);
    if(res == 0)
    {
        set_hand_force_result.data = true;
        this->Set_Hand_Force_Result->publish(set_hand_force_result);
    }
    else
    {
        set_hand_force_result.data = false;
        this->Set_Hand_Force_Result->publish(set_hand_force_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand force error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Follow_Angle_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg)
{
    int angle[6];
    int block;
    int32_t res;
    std_msgs::msg::Bool set_hand_angle_result;
    for(int i = 0;i<6;i++)
    {
        angle[i] = msg->hand_angle[i];
    }
    block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Follow_Angle(m_sockhand, angle, block);
    res = Rm_Api.rm_set_hand_follow_angle(robot_handle, angle, block);
    if(res == 0)
    {
        set_hand_angle_result.data = true;
        this->Set_Hand_Follow_Angle_Result->publish(set_hand_angle_result);
    }
    else
    {
        set_hand_angle_result.data = false;
        this->Set_Hand_Follow_Angle_Result->publish(set_hand_angle_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand follow angle error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Follow_Pos_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg)
{
    int pos[6];
    int block;
    int32_t res;
    std_msgs::msg::Bool set_hand_pos_result;
    for(int i = 0;i<6;i++)
    {
        pos[i] = msg->hand_angle[i];
    }
    block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Follow_Pos(m_sockhand, pos, block);
    res = Rm_Api.rm_set_hand_follow_pos(robot_handle, pos, block);
    if(res == 0)
    {
        set_hand_pos_result.data = true;
        this->Set_Hand_Follow_Pos_Result->publish(set_hand_pos_result);
    }
    else
    {
        set_hand_pos_result.data = false;
        this->Set_Hand_Follow_Pos_Result->publish(set_hand_pos_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand follow angle error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Lift_Speed_Callback(const rm_ros_interfaces::msg::Liftspeed::SharedPtr msg)
{
    int speed;
    int32_t res;
    std_msgs::msg::Bool set_lift_speed_result;
    speed = msg->speed;
    // res = Rm_Api.Service_Set_Lift_Speed(m_sockhand, speed);
    res = Rm_Api.rm_set_lift_speed(robot_handle, speed);
    if(res == 0)
    {
        set_lift_speed_result.data = true;
        this->Set_Lift_Speed_Result->publish(set_lift_speed_result);
    }
    else
    {
        set_lift_speed_result.data = false;
        this->Set_Lift_Speed_Result->publish(set_lift_speed_result);
        RCLCPP_INFO (this->get_logger(),"Arm set lift speed result error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Lift_Height_Callback(const rm_ros_interfaces::msg::Liftheight::SharedPtr msg)
{
    int speed;
    int height;
    bool block;
    int32_t res;
    std_msgs::msg::Bool set_lift_height_result;
    speed = msg->speed;
    height = msg->height;
    block = msg->block;
    // res = Rm_Api.Service_Set_Lift_Height(m_sockhand, height, speed, block);
    res = Rm_Api.rm_set_lift_height(robot_handle, speed, height, block);
    if(res == 0)
    {
        set_lift_height_result.data = true;
        this->Set_Lift_Height_Result->publish(set_lift_height_result);
    }
    else
    {
        set_lift_height_result.data = false;
        this->Set_Lift_Height_Result->publish(set_lift_height_result);
        RCLCPP_INFO (this->get_logger(),"Arm set lift height result error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Lift_State_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    // int current;
    // int height;
    // int err_flag;
    // int mode;
    int32_t res;
    rm_ros_interfaces::msg::Liftstate lift_state;
    rm_expand_state_t state;
    copy = msg;
    // res = Rm_Api.Service_Get_Lift_State(m_sockhand, &height, &current, &err_flag, &mode);
    res = Rm_Api.rm_get_lift_state(robot_handle, &state);
    if(res == 0)
    {
        lift_state.current = state.current;
        lift_state.height = state.pos;
        lift_state.err_flag = state.err_flag;
        lift_state.mode = state.mode;
        // lift_state.current = current;
        // lift_state.height = height;
        // lift_state.err_flag = err_flag;
        // lift_state.mode = mode;
        this->Get_Lift_State_Result->publish(lift_state);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm set lift state result error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Rm_Plus_Mode_Callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int mode;
    int32_t res;
    std_msgs::msg::Bool set_rm_plus_mode;
    mode = msg->data;
    // res = Rm_Api.Service_Set_Lift_Height(m_sockhand, height, speed, block);
    res = Rm_Api.rm_set_rm_plus_mode(robot_handle, mode);
    if(res == 0)
    {
        set_rm_plus_mode.data = true;
        this->Set_Rm_Plus_Mode_Result->publish(set_rm_plus_mode);
    }
    else
    {
        set_rm_plus_mode.data = false;
        this->Set_Rm_Plus_Mode_Result->publish(set_rm_plus_mode);
        RCLCPP_INFO (this->get_logger(),"Arm set rm plus mode result error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Rm_Plus_Mode_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    int32_t mode;
    int32_t res;
    std_msgs::msg::Int32 get_rm_plus_mode;
    copy = msg;
    // res = Rm_Api.Service_Get_Lift_State(m_sockhand, &height, &current, &err_flag, &mode);
    res = Rm_Api.rm_get_rm_plus_mode(robot_handle, &mode);
    if(res == 0)
    {
        get_rm_plus_mode.data = mode;
        this->Get_Rm_Plus_Mode_Result->publish(get_rm_plus_mode);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm get rm plus mode result error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Rm_Plus_Touch_Callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int mode;
    int32_t res;
    std_msgs::msg::Bool set_rm_plus_touch;
    mode = msg->data;
    // res = Rm_Api.Service_Set_Lift_Height(m_sockhand, height, speed, block);
    res = Rm_Api.rm_set_rm_plus_touch(robot_handle, mode);
    if(res == 0)
    {
        set_rm_plus_touch.data = true;
        this->Set_Rm_Plus_Touch_Result->publish(set_rm_plus_touch);
    }
    else
    {
        set_rm_plus_touch.data = false;
        this->Set_Rm_Plus_Touch_Result->publish(set_rm_plus_touch);
        RCLCPP_INFO (this->get_logger(),"Arm set rm plus touch result error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Rm_Plus_Touch_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    int32_t mode;
    int32_t res;
    std_msgs::msg::Int32 get_rm_plus_touch;
    copy = msg;
    // res = Rm_Api.Service_Get_Lift_State(m_sockhand, &height, &current, &err_flag, &mode);
    res = Rm_Api.rm_get_rm_plus_touch(robot_handle, &mode);
    if(res == 0)
    {
        get_rm_plus_touch.data = mode;
        this->Get_Rm_Plus_Touch_Result->publish(get_rm_plus_touch);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm get rm plus touch result error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Current_Arm_State_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    rm_pose_t pose;
    // float joint[7];
    // u_int16_t Err;
    // u_int8_t Err_len;
    rm_current_arm_state_t  current_state;
    int32_t res;
    copy = msg;
    std_msgs::msg::Bool get_current_arm_State_result;
    rm_euler_t euler;
    rm_quat_t quat;
    int i;
    // res = Rm_Api.Service_Get_Current_Arm_State(m_sockhand, joint, &pose, &Err, &Err_len);
    res = Rm_Api.rm_get_current_arm_state(robot_handle, &current_state);
    if(res == 0)
    {
        pose = current_state.pose;
        
        Arm_original_state.dof = 6;
        Arm_state.dof = 6;
        for(i=0;i<6;i++)
        {
            Arm_original_state.joint[i] = current_state.joint[i];
            Arm_state.joint[i] = current_state.joint[i] * DEGREE_RAD;
        }
        if(arm_dof_g == 7)
        {
            Arm_original_state.joint[6] = current_state.joint[6];
            Arm_state.joint[6] = current_state.joint[6] * DEGREE_RAD;
            Arm_original_state.dof = 7;
            Arm_state.dof = 7;
        }
        
        Arm_original_state.pose[0] = pose.position.x;
        Arm_original_state.pose[1] = pose.position.y;
        Arm_original_state.pose[2] = pose.position.z;
        Arm_original_state.pose[3] = pose.euler.rx;
        Arm_original_state.pose[4] = pose.euler.ry;
        Arm_original_state.pose[5] = pose.euler.rz;
        Arm_original_state.err = *current_state.err.err;
        Arm_original_state.err_len = current_state.err.err_len;
        this->Get_Current_Arm_Original_State_Result->publish(Arm_original_state);

        euler.rx = pose.euler.rx;
        euler.ry = pose.euler.ry;
        euler.rz = pose.euler.rz;
        // quat = Rm_Api.Service_Algo_Euler2Quaternion(euler);
        quat = Rm_Api.rm_algo_euler2quaternion(euler);
        Arm_state.pose.orientation.w = quat.w;
        Arm_state.pose.orientation.x = quat.x;
        Arm_state.pose.orientation.y = quat.y;
        Arm_state.pose.orientation.z = quat.z;
        Arm_state.pose.position.x = pose.position.x;
        Arm_state.pose.position.y = pose.position.y;
        Arm_state.pose.position.z = pose.position.z;
        Arm_state.err = *current_state.err.err;
        Arm_state.err_len = current_state.err.err_len;
        this->Get_Current_Arm_State_Result->publish(Arm_state);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm get current arm state error code is %d\n",res);
    }
}

void RmArm::Arm_Clear_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    copy = msg;
    // bool block;
    int32_t res;
    std_msgs::msg::Bool clear_force_data_result;
    // block = msg->data;
    // res = Rm_Api.Service_Clear_Force_Data(m_sockhand, block);
    res = Rm_Api.rm_clear_force_data(robot_handle);
    
    if(res == 0)
    {
        clear_force_data_result.data = true;
        this->Clear_Force_Data_Result->publish(clear_force_data_result);
    }
    else
    {
        clear_force_data_result.data = false;
        this->Clear_Force_Data_Result->publish(clear_force_data_result);
        RCLCPP_INFO (this->get_logger(),"Arm clear force data error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    rm_ros_interfaces::msg::Sixforce force;
    rm_ros_interfaces::msg::Sixforce zero_force;
    rm_ros_interfaces::msg::Sixforce work_zero;
    rm_ros_interfaces::msg::Sixforce tool_zero;
    // float force_data[6];
    // float zero_force_data[6];
    // float work_zero_data[6];
    // float tool_zero_data[6];
    rm_force_data_t data;
    copy = msg;
    int32_t res;
    // res = Rm_Api.Service_Get_Force_Data(m_sockhand, force_data, zero_force_data, work_zero_data, tool_zero_data);
    res = Rm_Api.rm_get_force_data(robot_handle, &data);
    if(res == 0)
    {
        force.force_fx = data.force_data[0];
        force.force_fy = data.force_data[1];
        force.force_fz = data.force_data[2];
        force.force_mx = data.force_data[3];
        force.force_my = data.force_data[4];
        force.force_mz = data.force_data[5];
        Get_Force_Data_Result->publish(force);
        zero_force.force_fx = data.zero_force_data[0];
        zero_force.force_fy = data.zero_force_data[1];
        zero_force.force_fz = data.zero_force_data[2];
        zero_force.force_mx = data.zero_force_data[3];
        zero_force.force_my = data.zero_force_data[4];
        zero_force.force_mz = data.zero_force_data[5];
        Get_Zero_Force_Result->publish(zero_force);
        work_zero.force_fx = data.work_zero_force_data[0];
        work_zero.force_fy = data.work_zero_force_data[1];
        work_zero.force_fz = data.work_zero_force_data[2];
        work_zero.force_mx = data.work_zero_force_data[3];
        work_zero.force_my = data.work_zero_force_data[4];
        work_zero.force_mz = data.work_zero_force_data[5];
        Get_Work_Zero_Result->publish(work_zero);
        tool_zero.force_fx = data.tool_zero_force_data[0];
        tool_zero.force_fy = data.tool_zero_force_data[1];
        tool_zero.force_fz = data.tool_zero_force_data[2];
        tool_zero.force_mx = data.tool_zero_force_data[3];
        tool_zero.force_my = data.tool_zero_force_data[4];
        tool_zero.force_mz = data.tool_zero_force_data[5];
        Get_Tool_Zero_Result->publish(tool_zero);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm get force data error code is %d\n",res);
    }
}

void Udp_Robot_Status_Callback(rm_realtime_arm_joint_state_t data)
{
    for(int i = 0; i < 6; i++)
    {
        Udp_RM_Joint.joint[i] = data.joint_status.joint_position[i];
        Udp_RM_Joint.err_flag[i] = data.joint_status.joint_err_code[i];
        Udp_RM_Joint.joint_current[i] = data.joint_status.joint_current[i];
        Udp_RM_Joint.en_flag[i] = data.joint_status.joint_en_flag[i];
        Udp_RM_Joint.joint_speed[i] = data.joint_status.joint_speed[i];
        Udp_RM_Joint.joint_temperature[i] = data.joint_status.joint_temperature[i];
        Udp_RM_Joint.joint_voltage[i] = data.joint_status.joint_voltage[i];
    }
    if(arm_dof_g == 7)
    {
        Udp_RM_Joint.joint[6] = data.joint_status.joint_position[6];
        Udp_RM_Joint.err_flag[6] = data.joint_status.joint_err_code[6];
        Udp_RM_Joint.joint_current[6] = data.joint_status.joint_current[6];
        Udp_RM_Joint.en_flag[6] = data.joint_status.joint_en_flag[6];
        Udp_RM_Joint.joint_speed[6] = data.joint_status.joint_speed[6];
        Udp_RM_Joint.joint_temperature[6] = data.joint_status.joint_temperature[6];
        Udp_RM_Joint.joint_voltage[6] = data.joint_status.joint_voltage[6];
    }
    if(Udp_RM_Joint.udp_rm_err.err_len != data.err.err_len)
    {
        Udp_RM_Joint.udp_rm_err.err_len = data.err.err_len;
        Udp_RM_Joint.udp_rm_err.err.resize(Udp_RM_Joint.udp_rm_err.err_len);
        for(int i = 0; i<Udp_RM_Joint.udp_rm_err.err_len; i++)
        {
            Udp_RM_Joint.udp_rm_err.err[i] = data.err.err[i];
        }
    }

    if(Udp_RM_Joint.control_version == 2)
    {
        for(int i = 0; i < 6; i++)
        {
            Udp_RM_Joint.six_force[i] = data.force_sensor.force[i];
            Udp_RM_Joint.zero_force[i] = data.force_sensor.zero_force[i];
        }
    }
    if(udp_hand_g == true)
    {
        for(int i = 0; i < 6; i++)
        {
            Udp_RM_Joint.hand_angle[i] = data.handState.hand_angle[i];
            Udp_RM_Joint.hand_force[i] = data.handState.hand_force[i];
            Udp_RM_Joint.hand_pos[i] = data.handState.hand_pos[i];
            Udp_RM_Joint.hand_state[i] = data.handState.hand_state[i];
        }
        Udp_RM_Joint.hand_err = data.handState.hand_err;
    }
    
    if(rm_plus_base_g == true)
    {
        for(int i = 0; i < 10; i++)
        {
            Udp_RM_Joint.udp_rm_plus_base_info.manu[i] = data.plus_base_info.manu[i];
            Udp_RM_Joint.udp_rm_plus_base_info.hv[i] = data.plus_base_info.hv[i];
            Udp_RM_Joint.udp_rm_plus_base_info.sv[i] = data.plus_base_info.sv[i];
            Udp_RM_Joint.udp_rm_plus_base_info.bv[i] = data.plus_base_info.bv[i];
            Udp_RM_Joint.udp_rm_plus_base_info.angle_low[i] = data.plus_base_info.angle_low[i];
            Udp_RM_Joint.udp_rm_plus_base_info.angle_up[i] = data.plus_base_info.angle_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_up[i] = data.plus_base_info.pos_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_low[i] = data.plus_base_info.pos_low[i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_up[i] = data.plus_base_info.speed_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_low[i] = data.plus_base_info.speed_low[i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_up[i] = data.plus_base_info.force_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_low[i] = data.plus_base_info.force_low[i];
        }
        for(int i = 0; i < 2; i++)
        {
            Udp_RM_Joint.udp_rm_plus_base_info.angle_low[10+i] = data.plus_base_info.angle_low[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.angle_up[10+i] = data.plus_base_info.angle_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_up[10+i] = data.plus_base_info.pos_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_low[10+i] = data.plus_base_info.pos_low[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_up[10+i] = data.plus_base_info.speed_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_low[10+i] = data.plus_base_info.speed_low[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_up[10+i] = data.plus_base_info.force_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_low[10+i] = data.plus_base_info.force_low[10+i];
        }
        Udp_RM_Joint.udp_rm_plus_base_info.id = data.plus_base_info.id;
        Udp_RM_Joint.udp_rm_plus_base_info.dof = data.plus_base_info.dof;
        Udp_RM_Joint.udp_rm_plus_base_info.check = data.plus_base_info.check;
        Udp_RM_Joint.udp_rm_plus_base_info.bee = data.plus_base_info.bee;
        Udp_RM_Joint.udp_rm_plus_base_info.force = data.plus_base_info.force;
        Udp_RM_Joint.udp_rm_plus_base_info.touch = data.plus_base_info.touch;
        Udp_RM_Joint.udp_rm_plus_base_info.touch_num = data.plus_base_info.touch_num;
        Udp_RM_Joint.udp_rm_plus_base_info.touch_sw = data.plus_base_info.touch_sw;
        Udp_RM_Joint.udp_rm_plus_base_info.hand = data.plus_base_info.hand;
    }

    if(rm_plus_state_g == true)
    {
        Udp_RM_Joint.udp_rm_plus_state_info.sys_state = data.plus_state_info.sys_state;
        for(int i = 0; i < 12; i++)
        {
            Udp_RM_Joint.udp_rm_plus_state_info.dof_state[i] = data.plus_state_info.dof_state[i];
            Udp_RM_Joint.udp_rm_plus_state_info.dof_err[i] = data.plus_state_info.dof_err[i];
            Udp_RM_Joint.udp_rm_plus_state_info.pos[i] = data.plus_state_info.pos[i];
            Udp_RM_Joint.udp_rm_plus_state_info.speed[i] = data.plus_state_info.speed[i];
            Udp_RM_Joint.udp_rm_plus_state_info.angle[i] = data.plus_state_info.angle[i];
            Udp_RM_Joint.udp_rm_plus_state_info.current[i] = data.plus_state_info.current[i];
            Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i] = data.plus_state_info.normal_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i] = data.plus_state_info.tangential_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i] = data.plus_state_info.tangential_force_dir[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tsa[i] = data.plus_state_info.tsa[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tma[i] = data.plus_state_info.tma[i];
            Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i] = data.plus_state_info.touch_data[i];
            Udp_RM_Joint.udp_rm_plus_state_info.force[i] = data.plus_state_info.force[i];
        }
        for(int i = 12; i < 18; i++)
        {
            Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i] = data.plus_state_info.normal_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i] = data.plus_state_info.tangential_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i] = data.plus_state_info.tangential_force_dir[i];
            Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i] = data.plus_state_info.touch_data[i];
        }
    }

    Udp_RM_Joint.joint_position[0] = data.waypoint.position.x;
    Udp_RM_Joint.joint_position[1] = data.waypoint.position.y;
    Udp_RM_Joint.joint_position[2] = data.waypoint.position.z;
    
    Udp_RM_Joint.joint_quat[0] = data.waypoint.quaternion.w;
    Udp_RM_Joint.joint_quat[1] = data.waypoint.quaternion.x;
    Udp_RM_Joint.joint_quat[2] = data.waypoint.quaternion.y;
    Udp_RM_Joint.joint_quat[3] = data.waypoint.quaternion.z;

    Udp_RM_Joint.joint_euler[0] = data.waypoint.euler.rx;
    Udp_RM_Joint.joint_euler[1] = data.waypoint.euler.ry;
    Udp_RM_Joint.joint_euler[2] = data.waypoint.euler.rz;

    Udp_RM_Joint.arm_current_status = data.arm_current_status;
    //std::cout<<"callback arm_current_status is "<<Udp_RM_Joint.arm_current_status<<std::endl;

    if(Udp_RM_Joint.control_version == 3)
    {
        Udp_RM_Joint.one_force = data.force_sensor.force[0];
        Udp_RM_Joint.one_zero_force = data.force_sensor.zero_force[0];
    }
    // Udp_RM_Joint.sys_err = data.sys_err;
    // Udp_RM_Joint.arm_err = data.arm_err;
    Udp_RM_Joint.coordinate = data.force_sensor.coordinate;
    // if(udp_hand_g == true)
    // {
    
    // }
}


void UdpPublisherNode::udp_timer_callback() 
{
    if(connect_state == 0)
    {
        // Rm_Api.Service_Realtime_Arm_Joint_State(Udp_RobotStatuscallback);
        Rm_Api.rm_realtime_arm_state_call_back(Udp_Robot_Status_Callback);
        udp_joint_error_code_.dof = 6;
        for(int i = 0;i<6;i++)
        {
            udp_real_joint_.position[i] = Udp_RM_Joint.joint[i] * DEGREE_RAD;
            udp_joint_error_code_.joint_error[i] = Udp_RM_Joint.err_flag[i];
            udp_joint_current_.joint_current[i] = Udp_RM_Joint.joint_current[i];
            udp_joint_en_flag_.joint_en_flag[i] = Udp_RM_Joint.en_flag[i];
            udp_joint_speed_.joint_speed[i] = Udp_RM_Joint.joint_speed[i];
            udp_joint_temperature_.joint_temperature[i] = Udp_RM_Joint.joint_temperature[i];
            udp_joint_voltage_.joint_voltage[i] = Udp_RM_Joint.joint_voltage[i];
        }
        if(arm_dof_g == 7)
        {
            udp_real_joint_.position[6] = Udp_RM_Joint.joint[6] * DEGREE_RAD;
            udp_joint_error_code_.joint_error[6] = Udp_RM_Joint.err_flag[6];
            udp_joint_current_.joint_current[6] = Udp_RM_Joint.joint_current[6];
            udp_joint_en_flag_.joint_en_flag[6] = Udp_RM_Joint.en_flag[6];
            udp_joint_speed_.joint_speed[6] = Udp_RM_Joint.joint_speed[6];
            udp_joint_temperature_.joint_temperature[6] = Udp_RM_Joint.joint_temperature[6];
            udp_joint_voltage_.joint_voltage[6] = Udp_RM_Joint.joint_voltage[6];
            udp_joint_error_code_.dof = 7;
        }
        if(udp_rm_err_.err_len != Udp_RM_Joint.udp_rm_err.err_len)
        {
            udp_rm_err_.err_len = Udp_RM_Joint.udp_rm_err.err_len;
            udp_rm_err_.err.resize(udp_rm_err_.err_len);
            for(int i = 0; i<udp_rm_err_.err_len; i++)
            {
                udp_rm_err_.err[i] = Udp_RM_Joint.udp_rm_err.err[i];
            }
        }
        this->Rm_Err_Result->publish(udp_rm_err_);
        udp_real_joint_.header.stamp = this->now();
        this->Joint_Position_Result->publish(udp_real_joint_);
        this->Joint_Error_Code_Result->publish(udp_joint_error_code_);
        udp_arm_current_status_.arm_current_status = Udp_RM_Joint.arm_current_status;
        //RCLCPP_INFO (this->get_logger(),"udp publisher arm_current_status is %d\n",udp_arm_current_status_.arm_current_status);
        this->Arm_Current_Status_Result->publish(udp_arm_current_status_);
        this->Joint_Current_Result->publish(udp_joint_current_);
        this->Joint_En_Flag_Result->publish(udp_joint_en_flag_);
        this->Joint_Speed_Result->publish(udp_joint_speed_);
        this->Joint_Temperature_Result->publish(udp_joint_temperature_);
        this->Joint_Voltage_Result->publish(udp_joint_voltage_);
        udp_arm_pose_.position.x = Udp_RM_Joint.joint_position[0];
        udp_arm_pose_.position.y = Udp_RM_Joint.joint_position[1];
        udp_arm_pose_.position.z = Udp_RM_Joint.joint_position[2];
        udp_arm_pose_.orientation.w = Udp_RM_Joint.joint_quat[0];
        udp_arm_pose_.orientation.x = Udp_RM_Joint.joint_quat[1];
        udp_arm_pose_.orientation.y = Udp_RM_Joint.joint_quat[2];
        udp_arm_pose_.orientation.z = Udp_RM_Joint.joint_quat[3];
        this->Arm_Position_Result->publish(udp_arm_pose_);
        udp_joint_pose_euler_.euler[0] = Udp_RM_Joint.joint_euler[0];
        udp_joint_pose_euler_.euler[1] = Udp_RM_Joint.joint_euler[1];
        udp_joint_pose_euler_.euler[2] = Udp_RM_Joint.joint_euler[2];
        udp_joint_pose_euler_.position[0]=Udp_RM_Joint.joint_position[0];
        udp_joint_pose_euler_.position[1]=Udp_RM_Joint.joint_position[1];
        udp_joint_pose_euler_.position[2]=Udp_RM_Joint.joint_position[2];
        this->Joint_Pose_Euler_Result->publish(udp_joint_pose_euler_);
        // sys_err_.data = Udp_RM_Joint.sys_err;
        // this->Sys_Err_Result->publish(sys_err_);
        // arm_err_.data = Udp_RM_Joint.arm_err;
        // this->Arm_Err_Result->publish(arm_err_);
        arm_coordinate_.data = Udp_RM_Joint.coordinate;
        this->Arm_Coordinate_Result->publish(arm_coordinate_);
        if(udp_hand_g == true)
        {
            for(int i = 0;i<6;i++)
            {
                udp_hand_status_.hand_angle[i] = Udp_RM_Joint.hand_angle[i];
                udp_hand_status_.hand_force[i] = Udp_RM_Joint.hand_force[i];
                udp_hand_status_.hand_pos[i] = Udp_RM_Joint.hand_pos[i];
                udp_hand_status_.hand_state[i] = Udp_RM_Joint.hand_state[i];
            }
            udp_hand_status_.hand_err = Udp_RM_Joint.hand_err;  
            this->Hand_Status_Result->publish(udp_hand_status_);
        }
        if(rm_plus_base_g == true)
        {
            for(int i = 0;i<10;i++)
            {
                udp_rm_plus_base_.pos_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_up[i];
                udp_rm_plus_base_.pos_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_low[i];
                udp_rm_plus_base_.speed_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_up[i];
                udp_rm_plus_base_.speed_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_low[i];
                udp_rm_plus_base_.force_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.force_up[i];
                udp_rm_plus_base_.force_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.force_low[i];
                udp_rm_plus_base_.angle_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_low[i];
                udp_rm_plus_base_.angle_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_up[i];
            }
            for(int i = 0; i < 2; i++)
            {
                udp_rm_plus_base_.pos_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_up[10+i];
                udp_rm_plus_base_.pos_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_low[10+i];
                udp_rm_plus_base_.speed_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_up[10+i];
                udp_rm_plus_base_.speed_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_low[10+i];
                udp_rm_plus_base_.force_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.force_up[10+i];
                udp_rm_plus_base_.force_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.force_low[10+i];
                udp_rm_plus_base_.angle_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_low[10+i];
                udp_rm_plus_base_.angle_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_up[10+i];
            }
            udp_rm_plus_base_.manu = Udp_RM_Joint.udp_rm_plus_base_info.manu;
            udp_rm_plus_base_.hv = Udp_RM_Joint.udp_rm_plus_base_info.hv;
            udp_rm_plus_base_.sv = Udp_RM_Joint.udp_rm_plus_base_info.sv;
            udp_rm_plus_base_.bv = Udp_RM_Joint.udp_rm_plus_base_info.bv;

            udp_rm_plus_base_.id = Udp_RM_Joint.udp_rm_plus_base_info.id;
            udp_rm_plus_base_.dof = Udp_RM_Joint.udp_rm_plus_base_info.dof;
            udp_rm_plus_base_.check = Udp_RM_Joint.udp_rm_plus_base_info.check;
            udp_rm_plus_base_.bee = Udp_RM_Joint.udp_rm_plus_base_info.bee;
            udp_rm_plus_base_.force = Udp_RM_Joint.udp_rm_plus_base_info.force;
            udp_rm_plus_base_.touch = Udp_RM_Joint.udp_rm_plus_base_info.touch;
            udp_rm_plus_base_.touch_num = Udp_RM_Joint.udp_rm_plus_base_info.touch_num;
            udp_rm_plus_base_.touch_sw = Udp_RM_Joint.udp_rm_plus_base_info.touch_sw;
            udp_rm_plus_base_.hand = Udp_RM_Joint.udp_rm_plus_base_info.hand;
            this->Rm_Plus_Base_Result->publish(udp_rm_plus_base_);
        }
        if(rm_plus_state_g == true)
        {
            udp_rm_plus_state_.sys_state = Udp_RM_Joint.udp_rm_plus_state_info.sys_state;
            for(int i = 0; i < 12; i++)
            {
                udp_rm_plus_state_.dof_state[i] = Udp_RM_Joint.udp_rm_plus_state_info.dof_state[i];
                udp_rm_plus_state_.dof_err[i] = Udp_RM_Joint.udp_rm_plus_state_info.dof_err[i];
                udp_rm_plus_state_.pos[i] = Udp_RM_Joint.udp_rm_plus_state_info.pos[i];
                udp_rm_plus_state_.speed[i] = Udp_RM_Joint.udp_rm_plus_state_info.speed[i];
                udp_rm_plus_state_.angle[i] = Udp_RM_Joint.udp_rm_plus_state_info.angle[i];
                udp_rm_plus_state_.current[i] = Udp_RM_Joint.udp_rm_plus_state_info.current[i];
                udp_rm_plus_state_.normal_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i];
                udp_rm_plus_state_.tangential_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i];
                udp_rm_plus_state_.tangential_force_dir[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i];
                udp_rm_plus_state_.tsa[i] = Udp_RM_Joint.udp_rm_plus_state_info.tsa[i];
                udp_rm_plus_state_.tma[i] = Udp_RM_Joint.udp_rm_plus_state_info.tma[i];
                udp_rm_plus_state_.touch_data[i] = Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i];
                udp_rm_plus_state_.force[i] = Udp_RM_Joint.udp_rm_plus_state_info.force[i];
            }
            for(int i = 12; i < 18; i++)
            {
                udp_rm_plus_state_.normal_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i];
                udp_rm_plus_state_.tangential_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i];
                udp_rm_plus_state_.tangential_force_dir[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i];
                udp_rm_plus_state_.touch_data[i] = Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i];
            }
            this->Rm_Plus_State_Result->publish(udp_rm_plus_state_);
        }
        if(Udp_RM_Joint.control_version == 2)
        {
            udp_sixforce_.force_fx = Udp_RM_Joint.six_force[0];
            udp_sixforce_.force_fy = Udp_RM_Joint.six_force[1];
            udp_sixforce_.force_fz = Udp_RM_Joint.six_force[2];
            udp_sixforce_.force_mx = Udp_RM_Joint.six_force[3];
            udp_sixforce_.force_my = Udp_RM_Joint.six_force[4];
            udp_sixforce_.force_mz = Udp_RM_Joint.six_force[5];
            this->Six_Force_Result->publish(udp_sixforce_);
            udp_zeroforce_.force_fx = Udp_RM_Joint.zero_force[0];
            udp_zeroforce_.force_fy = Udp_RM_Joint.zero_force[1];
            udp_zeroforce_.force_fz = Udp_RM_Joint.zero_force[2];
            udp_zeroforce_.force_mx = Udp_RM_Joint.zero_force[3];
            udp_zeroforce_.force_my = Udp_RM_Joint.zero_force[4];
            udp_zeroforce_.force_mz = Udp_RM_Joint.zero_force[5];
            this->Six_Zero_Force_Result->publish(udp_zeroforce_);
        }
        if(Udp_RM_Joint.control_version == 3)
        {
            udp_oneforce_.force_fz = Udp_RM_Joint.one_force;
            this->One_Force_Result->publish(udp_oneforce_);
            udp_onezeroforce_.force_fz = Udp_RM_Joint.one_zero_force;
            this->One_Zero_Force_Result->publish(udp_onezeroforce_);
        }
        if(ctrl_flag == true )
        {
            rclcpp::shutdown();
        }
    }
    else
    {
        if(come_time == 0)
        {Arm_Close();}
        come_time++;
        while(Arm_Socket_Start_Connect())
        {
            if(ctrl_flag == true )
            {
                rclcpp::shutdown();
                exit(0);
            }
            RCLCPP_INFO (this->get_logger(),"Wait for connect ");
            sleep(1);
                    
        }
        Arm_Start();
        come_time = 0;
        connect_state = 0;
        RCLCPP_INFO (this->get_logger(),"Connect success\n");
    }
}

void UdpPublisherNode::heart_timer_callback()
{
    int run_mode;
    if(connect_state == 0)
    {
        connect_state = Rm_Api.rm_get_arm_run_mode(robot_handle, &run_mode);
        // RCLCPP_INFO (this->get_logger(),"connect_state = %d\n",connect_state);
    }
    else
    {
        ;
    }
}

bool UdpPublisherNode::read_data()
{
    memset(udp_socket_buffer, 0, sizeof(udp_socket_buffer));

    ssize_t numBytes = recvfrom(16, udp_socket_buffer, sizeof(udp_socket_buffer), 0,
        (struct sockaddr*) & clientAddr, &clientAddrLen);
    if (numBytes < 0) {
        // std::cerr << "Error in recvfrom" << std::endl;
        close(16);
        return false;
    }
    // 将接收到的数据输出到控制台
    udp_socket_buffer[numBytes] = '\0'; // 添加字符串结束符
    std::cout << "Received from "                                         //<< inet_ntoa(clientAddr.sin_addr)
    << ":" << ntohs(clientAddr.sin_port) << ": "
    << udp_socket_buffer << std::endl;
    if((udp_socket_buffer[numBytes-2]==0X0D)&&(udp_socket_buffer[numBytes-1]==0X0A))
    {
        return true;
    }
    else
    {
        // ROS_ERROR("udp_socket_buffer IS error");
        return false;
    }
}

UdpPublisherNode::UdpPublisherNode():
    rclcpp::Node("udp_publish_node"){
        /*************************************************多线程********************************************/
        callback_group_time1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_time2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_time3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        /*****************************************************UDP定时器*****************************************************************/
        Udp_Timer = this->create_wall_timer(std::chrono::milliseconds(udp_cycle_g), 
        std::bind(&UdpPublisherNode::udp_timer_callback,this), callback_group_time1_);
        /*****************************************************定时器*****************************************************************/
        Heart_Timer = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&UdpPublisherNode::heart_timer_callback,this), callback_group_time2_);
        /********************************************************************UDP传输数据**********************************************************/
        Joint_Position_Result = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);                                    //发布当前的关节角度
        Arm_Position_Result = this->create_publisher<geometry_msgs::msg::Pose>("rm_driver/udp_arm_position", 10);                            //发布当前的关节姿态
        Six_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_six_force", 10);                          //发布当前的原始六维力数据
        Six_Zero_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_six_zero_force", 10);                //发布当前标坐标系下六维力数据
        One_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_one_force", 10);                          //发布当前的原始一维力数据
        One_Zero_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_one_zero_force", 10);                //发布当前目标坐标系下一维力数据
        Joint_Error_Code_Result = this->create_publisher<rm_ros_interfaces::msg::Jointerrorcode>("rm_driver/udp_joint_error_code", 10);      //发布当前的关节错误码
        // Sys_Err_Result = this->create_publisher<std_msgs::msg::UInt16>("rm_driver/udp_sys_err", 10);                                      //发布当前的系统错误码
        Rm_Err_Result = this->create_publisher<rm_ros_interfaces::msg::Rmerr>("rm_driver/udp_rm_err", 10);                                   //发布当前的机械臂错误码
        Arm_Coordinate_Result = this->create_publisher<std_msgs::msg::UInt16>("rm_driver/udp_arm_coordinate", 10);                           //发布当前六维力数据的基准坐标系
        Hand_Status_Result = this->create_publisher<rm_ros_interfaces::msg::Handstatus>("rm_driver/udp_hand_status", 10);                    //发布灵巧手状态数据

        Arm_Current_Status_Result = this->create_publisher<rm_ros_interfaces::msg::Armcurrentstatus>("rm_driver/udp_arm_current_status", 10);
        Joint_Current_Result = this->create_publisher<rm_ros_interfaces::msg::Jointcurrent>("rm_driver/udp_joint_current",10);
        Joint_En_Flag_Result = this->create_publisher<rm_ros_interfaces::msg::Jointenflag>("rm_driver/udp_joint_en_flag",10);
        Joint_Pose_Euler_Result = this->create_publisher<rm_ros_interfaces::msg::Jointposeeuler>("rm_driver/udp_joint_pose_euler",10);
        Joint_Speed_Result = this->create_publisher<rm_ros_interfaces::msg::Jointspeed>("rm_driver/udp_joint_speed",10);
        Joint_Temperature_Result = this->create_publisher<rm_ros_interfaces::msg::Jointtemperature>("rm_driver/udp_joint_temperature",10);
        Joint_Voltage_Result = this->create_publisher<rm_ros_interfaces::msg::Jointvoltage>("rm_driver/udp_joint_voltage",10);
        Rm_Plus_Base_Result = this->create_publisher<rm_ros_interfaces::msg::Rmplusbase>("rm_driver/udp_rm_plus_base",10);
        Rm_Plus_State_Result = this->create_publisher<rm_ros_interfaces::msg::Rmplusstate>("rm_driver/udp_rm_plus_state",10);
    }


RmArm::~RmArm()
{ 
    Arm_Close();
}

RmArm::RmArm():
    rclcpp::Node("rm_driver"){
    //参数初始化
    this->declare_parameter("arm_ip", "192.168.1.188");
    arm_ip_ = this->get_parameter("arm_ip").as_string();
    
    this->declare_parameter("udp_ip", "192.168.1.10");
    udp_ip_ = this->get_parameter("udp_ip").as_string();

    this->declare_parameter<std::string>("arm_type", arm_type_);
    this->get_parameter<std::string>("arm_type", arm_type_);

    this->declare_parameter<int>("tcp_port", tcp_port_);
    this->get_parameter<int>("tcp_port", tcp_port_);

    this->declare_parameter<int>("udp_port", udp_port_);
    this->get_parameter<int>("udp_port", udp_port_);

    this->declare_parameter<int>("arm_dof", arm_dof_);
    this->get_parameter<int>("arm_dof", arm_dof_);

    this->declare_parameter<int>("udp_cycle", udp_cycle_);
    this->get_parameter<int>("udp_cycle", udp_cycle_);

    this->declare_parameter<int>("udp_force_coordinate", udp_force_coordinate_);
    this->get_parameter<int>("udp_force_coordinate", udp_force_coordinate_);

    this->declare_parameter<bool>("udp_hand", udp_hand_);
    this->get_parameter<bool>("udp_hand", udp_hand_);

    this->declare_parameter<bool>("udp_plus_base", udp_rm_plus_base_);
    this->get_parameter<bool>("udp_plus_base", udp_rm_plus_base_);

    this->declare_parameter<bool>("udp_plus_state", udp_rm_plus_state_);
    this->get_parameter<bool>("udp_plus_state", udp_rm_plus_state_);

    this->declare_parameter<int>("trajectory_mode", trajectory_mode_);
    this->get_parameter<int>("trajectory_mode", trajectory_mode_);

    this->declare_parameter<int>("radio", radio_);
    this->get_parameter<int>("radio", radio_);

    this->declare_parameter("arm_joints", arm_joints);
    
    udp_cycle_g = udp_cycle_;
    if(arm_type_ == "RM_65")
    {
        // Rm_Api.Service_RM_API_Init(65, NULL);
        // RCLCPP_INFO (this->get_logger(),"0000000000000000000000000000000000000000000000000");
        realman_arm = 65;
    }
    else if(arm_type_ == "RM_75")
    {
        // Rm_Api.Service_RM_API_Init(75, NULL);
        realman_arm = 75;
    }
    else if(arm_type_ == "RM_63")
    {
        // Rm_Api.Service_RM_API_Init(632, NULL);
        realman_arm = 63;
    }
    else if(arm_type_ == "RM_eco65")
    {
        // Rm_Api.Service_RM_API_Init(651, NULL);
        realman_arm = 651;
    }
    else if(arm_type_ == "RM_eco63")
    {
        // Rm_Api.Service_RM_API_Init(634, NULL);
        realman_arm = 634;
    }
    else if(arm_type_ == "RM_eco62")
    {
        // Rm_Api.Service_RM_API_Init(62, NULL);
        realman_arm = 62;
    }
    else if(arm_type_ == "GEN_72")
    {
        // Rm_Api.Service_RM_API_Init(72, NULL);
        realman_arm = 72;
    }
    tcp_ip = (char*)arm_ip_.c_str();
    
    tcp_port = tcp_port_;
    udp_hand_g = udp_hand_;
    rm_plus_base_g = udp_rm_plus_base_;
    rm_plus_state_g = udp_rm_plus_state_;
    RCLCPP_INFO (this->get_logger(),"arm_ip is %s", arm_ip_.c_str());
    while(Arm_Socket_Start_Connect())
    {
        if(ctrl_flag == true )
        {
            rclcpp::shutdown();
            exit(0);
        }
        RCLCPP_INFO (this->get_logger(),"Waiting for connect");
        sleep(1);
    }
    usleep(2000000);
    RCLCPP_INFO (this->get_logger(),"%s_driver is running ",arm_type_.c_str());
    /************************************************初始化变量********************************************/
    udp_real_joint_.name.resize(arm_dof_);
    udp_real_joint_.position.resize(arm_dof_);
    udp_joint_error_code_.joint_error.resize(arm_dof_);
    Arm_original_state.joint.resize(arm_dof_);
    Arm_state.joint.resize(arm_dof_);
    udp_joint_current_.joint_current.resize(arm_dof_);
    udp_joint_en_flag_.joint_en_flag.resize(arm_dof_);
    udp_joint_speed_.joint_speed.resize(arm_dof_);
    udp_joint_temperature_.joint_temperature.resize(arm_dof_);
    udp_joint_voltage_.joint_voltage.resize(arm_dof_);
    arm_dof_g = arm_dof_;
    if (this->get_parameter("arm_joints", arm_joints))
    {
        for (int i = 0; i < arm_dof_; i++)
        {
            udp_real_joint_.name[i] = arm_joints[i];
            // RCLCPP_INFO (this->get_logger(),"arm_joints[%d]: %s",  i,  arm_joints[i].c_str());
        }
    }
    /**************************************************end**********************************************/
    
    /**********************************************初始化、连接函数****************************************/

    Arm_Start();

    /***************************************************end**********************************************/

    /*************************************************多线程********************************************/
    callback_group_sub1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt1 = rclcpp::SubscriptionOptions();
    sub_opt1.callback_group = callback_group_sub1_;
    callback_group_sub2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt2 = rclcpp::SubscriptionOptions();
    sub_opt2.callback_group = callback_group_sub2_;
    callback_group_sub3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt3 = rclcpp::SubscriptionOptions();
    sub_opt3.callback_group = callback_group_sub3_;
    callback_group_sub4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt4 = rclcpp::SubscriptionOptions();
    sub_opt4.callback_group = callback_group_sub4_;
    
    Get_Arm_Version();//获取机械臂版本

    Set_UDP_Configuration(udp_cycle_, udp_port_, udp_force_coordinate_, udp_ip_, udp_hand_, udp_rm_plus_base_, udp_rm_plus_state_);

    /******************************************************获取udp配置********************************************************************/
    Get_Realtime_Push_Result = this->create_publisher<rm_ros_interfaces::msg::Setrealtimepush>("rm_driver/get_realtime_push_result", rclcpp::ParametersQoS());
    Get_Realtime_Push_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_realtime_push_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Realtime_Push_Callback,this,std::placeholders::_1),
        sub_opt2);
    /******************************************************设置udp配置********************************************************************/
    Set_Realtime_Push_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_realtime_push_result", rclcpp::ParametersQoS());
    Set_Realtime_Push_Cmd = this->create_subscription<rm_ros_interfaces::msg::Setrealtimepush>("rm_driver/set_realtime_push_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Realtime_Push_Callback,this,std::placeholders::_1),
        sub_opt2);
/******************************************************************************end*******************************************************************/

/***********************************************************************运动配置**********************************************************************/
    /****************************************MoveJ运动控制*************************************/
    MoveJ_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movej_result", rclcpp::ParametersQoS());
    MoveJ_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movej>("rm_driver/movej_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveJ_Callback,this,std::placeholders::_1),
        sub_opt4);
    /****************************************MoveL运动控制*************************************/
    MoveL_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movel_result", rclcpp::ParametersQoS());
    MoveL_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movel>("rm_driver/movel_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveL_Callback,this,std::placeholders::_1),
        sub_opt4);
    /****************************************MoveC运动控制*************************************/
    MoveC_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movec_result", rclcpp::ParametersQoS());
    MoveC_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movec>("rm_driver/movec_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveC_Callback,this,std::placeholders::_1),
        sub_opt4);
    /******************************************角度透传*****************************************/
    Movej_CANFD_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointpos>("rm_driver/movej_canfd_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movej_CANFD_Callback,this,std::placeholders::_1),
        sub_opt4);
    Movej_CANFD_Custom_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointposcustom>("rm_driver/movej_canfd_custom_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movej_CANFD_Custom_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*******************************************位姿透传****************************************/
    Movep_CANFD_Cmd = this->create_subscription<rm_ros_interfaces::msg::Cartepos>("rm_driver/movep_canfd_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movep_CANFD_Callback,this,std::placeholders::_1),
        sub_opt4);
    Movep_CANFD_Custom_Cmd = this->create_subscription<rm_ros_interfaces::msg::Carteposcustom>("rm_driver/movep_canfd_custom_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movep_CANFD_Custom_Callback,this,std::placeholders::_1),
        sub_opt4);
    /****************************************MoveJ_P运动控制*************************************/
    MoveJ_P_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movej_p_result", rclcpp::ParametersQoS());
    MoveJ_P_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movejp>("rm_driver/movej_p_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveJ_P_Callback,this,std::placeholders::_1),
        sub_opt4);
    /***********************************************轨迹急停****************************************/
    Move_Stop_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/move_stop_result", rclcpp::ParametersQoS());
    Move_Stop_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/move_stop_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Move_Stop_Callback,this,std::placeholders::_1),
        sub_opt2);
/******************************************************************************end*******************************************************************/

/******************************************************************************示教指令*****************************************************************/
    /*********************************************************关节示教*****************************************************************/
    Set_Joint_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_joint_teach_result", rclcpp::ParametersQoS());
    Set_Joint_Teach_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointteach>("rm_driver/set_joint_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Joint_Teach_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*********************************************************位置示教*****************************************************************/
    Set_Pos_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_pos_teach_result", rclcpp::ParametersQoS());
    Set_Pos_Teach_Cmd = this->create_subscription<rm_ros_interfaces::msg::Posteach>("rm_driver/set_pos_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Pos_Teach_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*********************************************************姿态示教*****************************************************************/
    Set_Ort_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_ort_teach_result", rclcpp::ParametersQoS());
    Set_Ort_Teach_Cmd = this->create_subscription<rm_ros_interfaces::msg::Ortteach>("rm_driver/set_ort_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Ort_Teach_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*********************************************************示教停止*****************************************************************/
    Set_Stop_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_stop_teach_result", rclcpp::ParametersQoS());
    Set_Stop_Teach_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/set_stop_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Stop_Teach_Callback,this,std::placeholders::_1),
        sub_opt2);
/******************************************************************************end*****************************************************************/

    /************************************************************************查询机械臂固件版本***************************************************************/
    // Get_Arm_Software_Version_Result = this->create_publisher<rm_ros_interfaces::msg::Armsoftversion>("rm_driver/get_arm_software_version_result", rclcpp::ParametersQoS());
    // Get_Arm_Software_Version_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_arm_software_version_cmd",rclcpp::ParametersQoS(),
    //     std::bind(&RmArm::Arm_Get_Arm_Software_Version_Callback,this,std::placeholders::_1),
    //     sub_opt2);
    /*********************************************************************************end*******************************************************************/

/**********************************************************************透传力位混合控制***********************************************************/
    /******************************************************开启力位混合********************************************************************/
    Start_Force_Position_Move_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/start_force_position_move_result", rclcpp::ParametersQoS());
    Start_Force_Position_Move_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/start_force_position_move_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Start_Force_Position_Move_Callback,this,std::placeholders::_1),
        sub_opt2);
    /********************************************************关闭力位混合*******************************************************************/
    Stop_Force_Position_Move_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/stop_force_position_move_result", rclcpp::ParametersQoS());
    Stop_Force_Position_Move_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/stop_force_position_move_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Stop_Force_Position_Move_Callback,this,std::placeholders::_1),
        sub_opt2);
    /********************************************************角度透传力位混合*****************************************************************/
    Force_Position_Move_Joint_Cmd = this->create_subscription<rm_ros_interfaces::msg::Forcepositionmovejoint>("rm_driver/force_position_move_joint_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Force_Position_Move_Joint_Callback,this,std::placeholders::_1),
        sub_opt4);
    /********************************************************位姿透传力位混合*****************************************************************/
    Force_Position_Move_Pose_Cmd = this->create_subscription<rm_ros_interfaces::msg::Forcepositionmovepose>("rm_driver/force_position_move_pose_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Force_Position_Move_Pose_Callback,this,std::placeholders::_1),
        sub_opt4);
    /********************************************************设置力位混合控制*******************************************************************/
    Set_Force_Postion_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_force_postion_result", rclcpp::ParametersQoS());
    Set_Force_Postion_Cmd = this->create_subscription<rm_ros_interfaces::msg::Setforceposition>("rm_driver/set_force_postion_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Force_Postion_Callback,this,std::placeholders::_1),
        sub_opt4);
    /********************************************************结束力位混合控制*******************************************************************/
    Stop_Force_Postion_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/stop_force_postion_result", rclcpp::ParametersQoS());
    Stop_Force_Postion_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/stop_force_postion_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Stop_Force_Postion_Callback,this,std::placeholders::_1),
        sub_opt4);
/****************************************************************************end******************************************************************/

/************************************************************************坐标系指令*************************************************************/
    /**********************************************************切换工作坐标系********************************************************************/
    Change_Work_Frame_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/change_work_frame_result", rclcpp::ParametersQoS());
    Change_Work_Frame_Cmd = this->create_subscription<std_msgs::msg::String>("rm_driver/change_work_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Change_Work_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得工作坐标系********************************************************************/
    Get_Curr_WorkFrame_Result = this->create_publisher<std_msgs::msg::String>("rm_driver/get_curr_workFrame_result", rclcpp::ParametersQoS());
    Get_Curr_WorkFrame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_curr_workFrame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Curr_WorkFrame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得工具坐标系********************************************************************/
    Get_Current_Tool_Frame_Result = this->create_publisher<std_msgs::msg::String>("rm_driver/get_current_tool_frame_result", rclcpp::ParametersQoS());
    Get_Current_Tool_Frame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_current_tool_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Current_Tool_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得所有工具坐标系********************************************************************/
    Get_All_Tool_Frame_Result = this->create_publisher<rm_ros_interfaces::msg::Getallframe>("rm_driver/get_all_tool_frame_result", rclcpp::ParametersQoS());
    Get_All_Tool_Frame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_all_tool_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_All_Tool_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得所有工作坐标系********************************************************************/
    Get_All_Work_Frame_Result = this->create_publisher<rm_ros_interfaces::msg::Getallframe>("rm_driver/get_all_work_frame_result", rclcpp::ParametersQoS());
    Get_All_Work_Frame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_all_work_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_All_Work_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
/*****************************************************************************end***************************************************************/
    
    /**********************************************************设置工具端电源输出********************************************************************/
    Set_Tool_Voltage_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_tool_voltage_result", rclcpp::ParametersQoS());
    Set_Tool_Voltage_Cmd = this->create_subscription<std_msgs::msg::UInt16>("rm_driver/set_tool_voltage_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Tool_Voltage_Callback,this,std::placeholders::_1),
        sub_opt2);
    /*****************************************************************************end***************************************************************/

    /**********************************************************清除机械臂错误码********************************************************************/
    Set_Joint_Err_Clear_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_joint_err_clear_result", rclcpp::ParametersQoS());
    Set_Joint_Err_Clear_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointerrclear>("rm_driver/set_joint_err_clear_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Joint_Err_Clear_Callback,this,std::placeholders::_1),
        sub_opt2);
    /*****************************************************************************end***************************************************************/

/********************************************************************末端工具-手爪控制****************************************************************/
    /****************************************手爪持续力控夹取**********************************/
    Set_Gripper_Pick_On_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_gripper_pick_on_result", rclcpp::ParametersQoS());
    Set_Gripper_Pick_On_Cmd = this->create_subscription<rm_ros_interfaces::msg::Gripperpick>("rm_driver/set_gripper_pick_on_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Gripper_Pick_On_Callback,this,std::placeholders::_1),
        sub_opt3);
    /********************************************手爪力控夹取**********************************/
    Set_Gripper_Pick_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_gripper_pick_result", rclcpp::ParametersQoS());
    Set_Gripper_Pick_Cmd = this->create_subscription<rm_ros_interfaces::msg::Gripperpick>("rm_driver/set_gripper_pick_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Gripper_Pick_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*****************************************手爪到达指定位置**********************************/
    Set_Gripper_Position_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_gripper_position_result", rclcpp::ParametersQoS());
    Set_Gripper_Position_Cmd = this->create_subscription<rm_ros_interfaces::msg::Gripperset>("rm_driver/set_gripper_position_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Gripper_Position_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/

/********************************************************************末端工具-五指灵巧手控制************************************************************/
    /****************************************设置灵巧手手势序号**********************************/
    Set_Hand_Posture_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_posture_result", rclcpp::ParametersQoS());
    Set_Hand_Posture_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handposture>("rm_driver/set_hand_posture_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Posture_Callback,this,std::placeholders::_1),
        sub_opt3);
    /***************************************设置灵巧手动作序列序号*********************************/
    Set_Hand_Seq_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_seq_result", rclcpp::ParametersQoS());
    Set_Hand_Seq_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handseq>("rm_driver/set_hand_seq_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Seq_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手角度************************************/
    Set_Hand_Angle_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_angle_result", rclcpp::ParametersQoS());
    Set_Hand_Angle_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handangle>("rm_driver/set_hand_angle_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Angle_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手速度************************************/
    Set_Hand_Speed_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_speed_result", rclcpp::ParametersQoS());
    Set_Hand_Speed_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handspeed>("rm_driver/set_hand_speed_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Speed_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手力度************************************/
    Set_Hand_Force_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_force_result", rclcpp::ParametersQoS());
    Set_Hand_Force_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handforce>("rm_driver/set_hand_force_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Force_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手角度跟随************************************/
    Set_Hand_Follow_Angle_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_follow_angle_result", rclcpp::ParametersQoS());
    Set_Hand_Follow_Angle_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handangle>("rm_driver/set_hand_follow_angle_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Follow_Angle_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手姿势跟随************************************/
    Set_Hand_Follow_Pos_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_follow_pos_result", rclcpp::ParametersQoS());
    Set_Hand_Follow_Pos_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handangle>("rm_driver/set_hand_follow_pos_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Follow_Pos_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/

/********************************************************************升降机构************************************************************/
    /****************************************设置升降机构速度**********************************/
    Set_Lift_Speed_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_lift_speed_result", rclcpp::ParametersQoS());
    Set_Lift_Speed_Cmd = this->create_subscription<rm_ros_interfaces::msg::Liftspeed>("rm_driver/set_lift_speed_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Lift_Speed_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************设置升降机构高度**********************************/
    Set_Lift_Height_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_lift_height_result", rclcpp::ParametersQoS());
    Set_Lift_Height_Cmd = this->create_subscription<rm_ros_interfaces::msg::Liftheight>("rm_driver/set_lift_height_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Lift_Height_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************获取升降机构状态**********************************/
    Get_Lift_State_Result = this->create_publisher<rm_ros_interfaces::msg::Liftstate>("rm_driver/get_lift_state_result", rclcpp::ParametersQoS());
    Get_Lift_State_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_lift_state_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Lift_State_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/

    /***************************************************获取机械臂当前状态********************************************/
    Get_Current_Arm_Original_State_Result = this->create_publisher<rm_ros_interfaces::msg::Armoriginalstate>("rm_driver/get_current_arm_original_state_result", rclcpp::ParametersQoS());
    Get_Current_Arm_State_Result = this->create_publisher<rm_ros_interfaces::msg::Armstate>("rm_driver/get_current_arm_state_result", rclcpp::ParametersQoS());
    Get_Current_Arm_State_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_current_arm_state_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Current_Arm_State_Callback,this,std::placeholders::_1),
        sub_opt2);
/*********************************************************************六维力***************************************************************/
    /*****************************************************六维力数据清零**********************************************/
    Clear_Force_Data_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/clear_force_data_result", rclcpp::ParametersQoS());
    Clear_Force_Data_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/clear_force_data_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Clear_Force_Data_Callback,this,std::placeholders::_1),
        sub_opt2);
    /******************************************************获取六维力数据************************************************/
    /***************************************************传感器受到的外力数据***********************************************/
    Get_Force_Data_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_force_data_result", rclcpp::ParametersQoS());
    /***************************************************系统受到的外力数据***********************************************/
    Get_Zero_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_zero_force_data_result", rclcpp::ParametersQoS());
    /************************************************工作坐标系下系统受到的外力数据******************************************/
    Get_Work_Zero_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_work_force_data_result", rclcpp::ParametersQoS());
    /***********************************************工具坐标系下系统受到的外力数据********************************************/
    Get_Tool_Zero_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_tool_force_data_result", rclcpp::ParametersQoS());
    Get_Force_Data_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_force_data_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Force_Data_Callback,this,std::placeholders::_1),
        sub_opt2);
/*******************************************************************************end*****************************************************************/

/********************************************************************末端生态协议************************************************************/
    /****************************************设置末端生态协议模式**********************************/
    Set_Rm_Plus_Mode_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_rm_plus_mode_result", rclcpp::ParametersQoS());
    Set_Rm_Plus_Mode_Cmd = this->create_subscription<std_msgs::msg::Int32>("rm_driver/set_rm_plus_mode_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Rm_Plus_Mode_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************查询末端生态协议模式**********************************/
    Get_Rm_Plus_Mode_Result = this->create_publisher<std_msgs::msg::Int32>("rm_driver/get_rm_plus_mode_result", rclcpp::ParametersQoS());
    Get_Rm_Plus_Mode_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_rm_plus_mode_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Rm_Plus_Mode_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************设置触觉传感器模式**********************************/
    Set_Rm_Plus_Touch_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_rm_plus_touch_result", rclcpp::ParametersQoS());
    Set_Rm_Plus_Touch_Cmd = this->create_subscription<std_msgs::msg::Int32>("rm_driver/set_rm_plus_touch_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Rm_Plus_Touch_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************获取触觉传感器模式**********************************/
    Get_Rm_Plus_Touch_Result = this->create_publisher<std_msgs::msg::Int32>("rm_driver/get_rm_plus_touch_result", rclcpp::ParametersQoS());
    Get_Rm_Plus_Touch_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_rm_plus_touch_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Rm_Plus_Touch_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/

}   



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, my_handler); 
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),8,true);
    auto node = std::make_shared<RmArm>();
    auto udpnode = std::make_shared<UdpPublisherNode>();
    executor.add_node(node);
    executor.add_node(udpnode);
    executor.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}