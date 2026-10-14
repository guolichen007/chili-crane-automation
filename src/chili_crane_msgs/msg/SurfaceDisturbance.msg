# ONLY synthetic/replay on Phase2B shadow input. Not a TaskExecution command.
std_msgs/Header header
uint8 validity
string reason
string event_id
# Bind delayed synthetic/replay events to the producer they invalidate.
string surface_session_id
uint64 source_counter
string source_type
string map_id
string semantic_version
string pit_id
string transform_id
string kernel_id
geometry_msgs/Polygon footprint
