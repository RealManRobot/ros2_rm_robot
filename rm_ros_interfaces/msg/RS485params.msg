int32 mode     # 0代表默认RS485串行通讯，1代表modbus-RTU主站模式，2-代表modbus-RTU从站模式。
int32 baudrate # 波特率(当前支持9600 19200 38400 57600 115200 230400 460800)
bool state     # 查询状态 true查询成功 false查询失败