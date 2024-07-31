# POS_TEACH_MODES type  示教类型
# typedef enum
# {
#     X_Dir = 0,       //X轴方向
#      Y_Dir = 1,       //Y轴方向
#     Z_Dir = 2,       //Z轴方向
# }POS_TEACH_MODES;
uint8 type           # 示教类型 输入0 | 1 | 2
uint8 direction      # 示教方向，0-负方向，1-正方向
uint8 speed          # 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
bool block           # RM_NONBLOCK0-非阻塞，发送后立即返回；RM_BLOCK1-阻塞，等待控制器返回设置成功指令