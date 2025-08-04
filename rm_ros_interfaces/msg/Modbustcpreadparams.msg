int32 address      # 数据起始地址
string master_name   # Modbus 主站名称，最大长度15个字符，不超过15个字符（master_name与IP二选一，若有IP和port优先使用IP和port）
string ip           # 主机连接的 IP 地址（master_name与IP二选一，若有IP和port优先使用IP和port）
int32 port         # 主机连接的端口号
int32 num           # 读取数据数量，最大不超过100