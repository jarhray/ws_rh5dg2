std_msgs/Header header
uint64 command_id
# Accepted means the register write was acknowledged, not that motion finished.
bool accepted
string message
# Valid only when accepted=true; -1 marks a skipped angle channel.
# Acknowledged targets after angle limiting, not measured positions.
int16[13] applied_values
bool[13] clamped
