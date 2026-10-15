# The topic selects the operation: angle_set, speed_set, force_set,
# current_limit_set, or mode_set. Units/order/limits match HandCommand.
# Requires a fresh nonzero stamp no earlier than the current command enable.
# Results use command_result; use distinct command_id values across all inputs.
std_msgs/Header header
uint64 command_id
int16[13] values
