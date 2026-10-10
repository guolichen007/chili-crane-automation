uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason
string[] signal_names
bool[] signal_values
chili_crane_msgs/EvidenceMetadata evidence
