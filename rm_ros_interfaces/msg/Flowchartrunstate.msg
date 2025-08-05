uint8 run_state  # 运行状态 0 未开始 1运行中 2暂停中
uint8 id         # 当前使能的文件id。
string name      # 当前使能的文件名称。
uint8 plan_speed # 当前使能的文件全局规划速度比例 1-100。
uint8 step_mode  # 单步模式，0为空，1为正常, 2为单步。
string modal_id  # 运行到的流程图块的id。未运行则不返回
bool state       # 查询状态信息