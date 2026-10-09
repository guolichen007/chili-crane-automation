uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason
string pit_id
float64 resolution_m
uint32 valid_cell_count
uint32 total_cell_count
float64 coverage
float64 robust_min_z_m
float64 robust_max_z_m
float64 evidence_age_sec
chili_crane_msgs/EvidenceMetadata evidence
