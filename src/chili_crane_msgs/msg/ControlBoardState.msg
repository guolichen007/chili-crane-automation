uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason
bool e_stop_active
bool power_ok
bool manual_mode
bool auto_mode
bool io_heartbeat_ok
bool fault
float64 evidence_age_sec
