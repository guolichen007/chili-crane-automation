uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason
float64 raw_position
float64 position_m
float64 velocity_mps
bool homed
bool running
bool fault
string calibration_id
float64 evidence_age_sec
