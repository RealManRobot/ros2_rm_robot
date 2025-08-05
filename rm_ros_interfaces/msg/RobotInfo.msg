# 机械臂自由度（关节数量）
uint8 arm_dof
# 机械臂型号
# 示例：0=RM_65, 1=RM_75, 2=RML_63I(已弃用), 3=RML_63II,4=RML_63III,5=ECO_65,6=ECO_62,7=GEN_72,8=ECO63,9=通用机器人
uint8 arm_model
# 末端力传感器版本
# 示例：0=标准版, 1=一维力版, 2=六维力版, 3=一体化六维力版
uint8 force_type
# 机械臂控制器版本（3：三代，4：四代）
uint8 robot_controller_version
# 是否读取成功
bool state
