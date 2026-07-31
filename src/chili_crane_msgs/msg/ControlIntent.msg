uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

uint8 ACTION_NONE=0
uint8 ACTION_STOP_ALL=1
uint8 ACTION_MOVE_X=2
uint8 ACTION_MOVE_Y=3
uint8 ACTION_RAISE=4
uint8 ACTION_LOWER=5
uint8 ACTION_GRAB_OPEN=6
uint8 ACTION_GRAB_CLOSE=7
uint8 ACTION_HOLD=8

std_msgs/Header header
uint8 validity
string reason
string request_id
uint8 action
float64 target_x_m
float64 target_y_m
float64 target_grab_bottom_z_m
float64 speed_scale
float64 timeout_sec
