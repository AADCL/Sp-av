# Decoded RC levels, not action requests. Consumers must require valid and
# independently verify freshness. A first frame or reconnection is not an edge.
std_msgs/Header header
bool valid
string reason
# Roll/pitch/yaw: calibrated PX4 channel sense [-1,1]. Throttle: [0,1].
float64 roll
float64 pitch
float64 throttle
float64 yaw
string mode
bool kill_switch
bool arm_switch
bool land_switch
bool trigger_switch
