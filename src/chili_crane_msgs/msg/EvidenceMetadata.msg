uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

builtin_interfaces/Time measurement_stamp
builtin_interfaces/Time receive_stamp
uint64 source_counter
float64 evidence_age_sec
string calibration_id
string config_version
