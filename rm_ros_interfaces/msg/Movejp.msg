geometry_msgs/Pose pose
uint8 speed
uint8 trajectory_connect   #0 代表立即规划，1 代表和下一条轨迹一起规划，当为 1 时，轨迹不会立即执行
bool block