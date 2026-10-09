uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

bool a_online
bool a_fresh
bool a_frame_valid
bool a_calibration_valid
bool b_online
bool b_fresh
bool b_frame_valid
bool b_calibration_valid
bool pairing_valid
bool extrinsic_valid
bool coverage_valid
bool dual_lidar_ready
chili_crane_msgs/EvidenceMetadata evidence
