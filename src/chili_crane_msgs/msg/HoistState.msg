uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

int8 MOTION_DOWN=-1
int8 MOTION_STOPPED=0
int8 MOTION_UP=1
int8 MOTION_UNKNOWN=2

std_msgs/Header header
uint8 validity
string reason
int8 motion
bool upper_limit
bool lower_limit
bool brake_engaged
bool running
bool fault
float64 evidence_age_sec
