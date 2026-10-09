uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

bool configuration_ready
bool hardware_ready
bool dual_lidar_ready
bool localization_ready
bool grab_tracking_ready
bool pit_perception_ready
bool control_ready
bool safety_ready
bool x_auto_ready
bool y_auto_ready
bool z_raise_ready
bool z_lower_ready
bool grab_ready
bool system_ready
string[] blocking_reasons
chili_crane_msgs/EvidenceMetadata evidence
