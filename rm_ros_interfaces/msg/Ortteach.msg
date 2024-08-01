# ORT_TEACH_MODES type   # 示教类型
# typedef enum
# {
#     RX_Rotate = 0,       //RX轴方向
#     RY_Rotate = 1,       //RY轴方向
#     RZ_Rotate = 2,       //RZ轴方向
# }ORT_TEACH_MODES;
uint8 type             # 示教类型 输入0 | 1 | 2
uint8 direction        # 示教方向，0-负方向，1-正方向
uint8 speed            # 速度比例1~100，即规划速度和加速度占机械臂末端最大角速度和角加速度的百分比
bool block             # RM_NONBLOCK0-非阻塞，发送后立即返回；RM_BLOCK1-阻塞，等待控制器返回设置成功指令