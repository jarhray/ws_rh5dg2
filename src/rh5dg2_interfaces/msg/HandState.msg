# Header time is snapshot publication time. Each field has its own acquisition
# stamp and validity. Cached values survive failures but must not be used when invalid.
std_msgs/Header header
uint64 sequence
bool connected
bool enabled
# Stale means disconnected or angle/error feedback expired; inspect other flags too.
bool stale
string status
string[13] joint_names
int16[13] angle_raw
int16[13] force_raw
int16[13] current_ma
int16[13] temperature_c
int16[13] motor_status
int16[13] error_code
# Channels 7-10 (indices 6-9) have no meaningful forceAct measurement in the manual.
bool[13] force_sensor_present
bool angle_valid
bool force_valid
bool current_valid
bool temperature_valid
bool motor_status_valid
bool error_valid
builtin_interfaces/Time angle_stamp
builtin_interfaces/Time force_stamp
builtin_interfaces/Time current_stamp
builtin_interfaces/Time temperature_stamp
builtin_interfaces/Time motor_status_stamp
builtin_interfaces/Time error_stamp
