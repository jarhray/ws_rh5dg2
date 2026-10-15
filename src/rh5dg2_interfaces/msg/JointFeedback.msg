# One quantity per topic; values use the same 13-channel order as HandState.
# header.stamp is acquisition completion time, NOT publication time. On expiry
# or disconnect, the last values/stamp remain but valid becomes false.
# A zero stamp means this quantity has never been acquired.
std_msgs/Header header
# Fresh transport data; does not mean error-free motors or calibrated sensors.
bool valid
# angle_actual: "0.1 deg", force_actual: "g", current_actual: "mA",
# temperature: "degC", motor_status/error_code: "code" (unmodified firmware codes).
string unit
string[13] joint_names
int16[13] values
# Supported channels, independent of freshness. force_actual indices 6..9
# have no corresponding force sensor; their raw values must not be used.
bool[13] channel_present
