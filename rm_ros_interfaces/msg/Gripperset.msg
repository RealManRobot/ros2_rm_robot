#设置手爪到固定位置，手爪到位置后或者所受力超过阈值后停止
uint16 position   #手爪目标位置，范围：1～1000,代表手爪开口度：0～70mm
bool block
uint16 timeout    #timeout 设置返回超时时间，阻塞模式生效
