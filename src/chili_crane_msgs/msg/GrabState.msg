uint8 UNKNOWN=0
uint8 NOT_CONFIGURED=1
uint8 STALE=2
uint8 DEGRADED=3
uint8 VALID=4

uint8 TRACK_UNKNOWN=0
uint8 TRACK_CANDIDATE=1
uint8 TRACK_TRACKED=2
uint8 TRACK_LOST_HOLD=3
uint8 TRACK_LOST=4

uint8 OPENING_UNKNOWN=0
uint8 OPENING_OPEN=1
uint8 OPENING_CLOSED=2
uint8 OPENING_TRANSITION=3
uint8 OPENING_CONFLICT=4

uint8 SOURCE_NONE=0
uint8 SOURCE_MEASURED=1
uint8 SOURCE_FILTERED=2
uint8 SOURCE_PREDICTED=3

std_msgs/Header header
uint8 validity
string reason
uint8 tracking_state
uint8 opening_state
uint8 source
geometry_msgs/PoseWithCovariance pose
float64 bottom_z_m
float64 top_z_m
float64 length_m
float64 width_m
float64 height_m
geometry_msgs/Polygon footprint_map
float64 confidence
float64 evidence_age_sec
bool fault
bool motion_stable
chili_crane_msgs/EvidenceMetadata evidence
