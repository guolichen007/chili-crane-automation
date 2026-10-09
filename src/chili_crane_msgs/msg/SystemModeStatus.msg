uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

uint8 BOOT=0
uint8 SELF_CHECK=1
uint8 SAFE_IDLE=2
uint8 AUTO_PENDING=3
uint8 AUTO_READY=4
uint8 AUTO_ACTIVE=5
uint8 REMOTE=6
uint8 PROTECTIVE_STOP=7
uint8 FAULT_LATCHED=8
uint8 EMERGENCY_STOP=9
uint8 MAINTENANCE=10
uint8 mode
string session_id
uint64 command_epoch
bool release_automatic_outputs
bool discard_pending_commands
bool require_new_task
