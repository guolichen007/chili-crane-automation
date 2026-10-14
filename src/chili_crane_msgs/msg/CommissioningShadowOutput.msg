# Diagnostic expected mapping ONLY, never an ActuationRequest/authorization.
std_msgs/Header header
uint8 validity
string reason
string semantic_signal
string arbitration_mode
bool[8] expected_do
bool software_only
bool physical_output_enabled
bool automatic_control_enabled
chili_crane_msgs/EvidenceMetadata evidence
