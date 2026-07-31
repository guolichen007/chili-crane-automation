uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

uint8 LOAD_UNKNOWN=0
uint8 LOAD_SUSPENDED_EMPTY=1
uint8 LOAD_CONTACT_CANDIDATE=2
uint8 LOAD_LOADED=3
uint8 LOAD_SENSOR_FAULT=4

std_msgs/Header header
uint8 validity
string reason
uint8 load_state
float64 value
float64 delta
string unit
string calibration_id
float64 evidence_age_sec
