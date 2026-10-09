uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

string axis
int8 direction
bool enable
bool speed_command_valid
float64 speed_command
string source_command_id
string run_id
string task_id
string cycle_id
string session_id
uint64 command_epoch
uint64 command_sequence
uint64 actuation_sequence
builtin_interfaces/Time issued_stamp
builtin_interfaces/Time expire_stamp
chili_crane_msgs/EvidenceMetadata evidence
string intent_id
