uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason
string sensor_id
string calibration_id
string extrinsic_version
string intrinsic_version
