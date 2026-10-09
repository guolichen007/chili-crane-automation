uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

string frame_id
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max
string geometry_version
bool flow_projection_verified
