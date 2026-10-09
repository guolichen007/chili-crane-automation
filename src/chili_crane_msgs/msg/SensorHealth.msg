uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason
string sensor_id
float64 evidence_age_sec
float64 observed_rate_hz
uint64 received_count
uint64 dropped_count
uint64 fallback_count
chili_crane_msgs/EvidenceMetadata evidence
