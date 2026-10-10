uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

std_msgs/Header header
uint8 validity
string reason

string run_id
string task_id
string cycle_id
string git_sha
string config_hash
string map_id
string[] required_sensor_ids
chili_crane_msgs/CalibrationRef[] calibrations
string grab_geometry_version
builtin_interfaces/Time started_stamp
chili_crane_msgs/EvidenceMetadata evidence
