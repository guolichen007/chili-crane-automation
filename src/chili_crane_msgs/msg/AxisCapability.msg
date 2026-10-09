uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

string axis
string drive_profile
bool verified
bool speed_command_supported
bool stop_distance_verified
bool reaction_time_verified
bool fixed_speed_verified_safe
bool pulse_jog_allowed
bool pulse_jog_timing_valid
float64 minimum_on_time
float64 minimum_off_time
string config_version
