int32 pos	    #当前角度 精度 0.001°，单位：°
int32 current	#当前电流 单位：mA，精度：1mA
uint16 err_flag	#驱动错误代码
bool en_flag    #1 为上使能，0 为掉使能
uint8 joint_id  #关节id号
int16 mode      #当前升降状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动

