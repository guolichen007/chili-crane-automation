uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

builtin_interfaces/Time a_mid_stamp
builtin_interfaces/Time b_mid_stamp
float64 header_delta_sec
float64 mid_delta_sec
float64 interval_overlap_sec
float64 interval_overlap_ratio
uint64 paired_count
uint64 dropped_a
uint64 dropped_b
bool pairing_valid
bool timing_ready
bool extrinsic_valid
bool spatial_merge_ready
bool dual_lidar_full_ready
bool ptp_verified
chili_crane_msgs/EvidenceMetadata evidence
