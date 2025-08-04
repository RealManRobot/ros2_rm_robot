int32 run_state   # 运行状态 0 未开始 1运行中 2暂停中
int32 id          # 运行轨迹编号
int32 edit_id     # 上次编辑的在线编程编号 id
int32 plan_num    # 运行行数
int32 total_loop      # 循环指令数量
int32 step_mode       # 单步模式，1 为单步模式，0 为非单步模式
int32 plan_speed      # 全局规划速度比例 1-100
int32[] loop_num        # 循环行数
int32[] loop_cont       # 对应循环次数
bool state        # 文件下发状态，成功true,失败false