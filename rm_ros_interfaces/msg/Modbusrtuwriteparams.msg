int32 address    # 数据起始地址
int32 device     # 外设设备地址
int32 type        # 0-控制器端modbus主机；1-工具端modbus主机。
int32 num         # 要写的数据的数量，最大不超过100
int32[] data     # 要写的数据，数据长度不超过100