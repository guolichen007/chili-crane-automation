uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason
string station_id
bool present
bool aligned
bool full
bool overflow
geometry_msgs/Vector3 offset_m
float64 confidence
float64 evidence_age_sec
