string  project_path     # 下发文件路径文件名
int32 project_path_len   # 名称长度
int32 plan_speed         # 规划速度比例系数
int32 only_save          # 0-保存并运行文件，1-仅保存文件，不运行
int32 save_id            # 保存到控制器中的编号
int32 step_flag          # 设置单步运行方式模式，1-设置单步模式 0-设置正常运动模式
int32 auto_start         # 设置默认在线编程文件，1-设置默认  0-设置非默认
int32 project_type       # 下发文件类型。0-在线编程文件，1-拖动示教轨迹文件