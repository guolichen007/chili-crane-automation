# Dedicated bounded algorithm transport, NOT a status/debug interpolation map.
chili_crane_msgs/PitSurfaceStatus status
string surface_session_id
uint8 validity
string reason
chili_crane_msgs/PlanningSpatialContext spatial_context
float64 origin_x_m
float64 origin_y_m
uint32 width
uint32 height
# V2 breaking wire semantics: observed-only indexed cells; absent indices UNKNOWN.
string cell_encoding
chili_crane_msgs/PitSurfaceCell[<=100000] cells
