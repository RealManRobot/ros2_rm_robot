string manu              # 设备厂家
int8 type                # 设备类型 1：两指夹爪 2：五指灵巧手 3：三指夹爪
string hv                # 硬件版本
string sv                # 软件版本
string bv                # boot版本
int32 id                 # 设备ID
int8 dof                 # 自由度
int8 check               # 自检开关
int8 bee                 # 蜂鸣器开关
bool force               # 力控支持
bool touch               # 触觉支持
int8 touch_num           # 触觉个数
int8 touch_sw            # 触觉开关
int8 hand                # 手方向 1 ：左手 2： 右手
int32[12] pos_up         # 位置上限,单位：无量纲
int32[12] pos_low        # 位置下限,单位：无量纲
int32[12] angle_up       # 角度上限,单位：0.01度
int32[12] angle_low      # 角度下限,单位：0.01度
int32[12] speed_up       # 速度上限,单位：无量纲
int32[12] speed_low      # 速度下限,单位：无量纲
int32[12] force_up       # 力上限,单位：0.001N 
int32[12] force_low      # 力下限,单位：0.001N 
