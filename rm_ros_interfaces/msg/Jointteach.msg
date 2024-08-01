uint8 num          # 示教关节的序号，1~7
uint8 direction    # 示教方向，0-负方向，1-正方向
uint8 speed        # 速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比
bool block         # RM_NONBLOCK0-非阻塞，发送后立即返回；RM_BLOCK1-阻塞，等待控制器返回设置成功指令