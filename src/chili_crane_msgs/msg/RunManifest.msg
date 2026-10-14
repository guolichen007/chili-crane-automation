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
string timebase_json
string map_id
string[] required_sensor_ids
chili_crane_msgs/CalibrationRef[] calibrations
string grab_geometry_version
builtin_interfaces/Time started_stamp
chili_crane_msgs/EvidenceMetadata evidence
string schema_version
string phase
string recording_profile
bool recording_enabled
# recording_enabled is a compatibility alias for rosbag_enabled, never JSONL.
bool journal_enabled
bool rosbag_configured
bool rosbag_enabled
bool rosbag_process_alive
string rosbag_storage_id
bool recording_capacity_configured
