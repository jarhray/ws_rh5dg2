# Header stamp is last successful acquisition time, unchanged on invalidation.
# Finger order: pinky, ring, middle, index, thumb. No forceAct-to-joint-torque conversion.
std_msgs/Header header
bool valid
float64[5] normal_force_n
float64[5] tangential_force_n
uint16[5] direction_deg
bool[5] direction_valid
uint32[5] proximity
# Three register-order triples: normal (0.01 N), tangential (0.01 N), direction (degree).
# Physical palm side labels differ between supplied materials; retain raw order.
uint16[9] palm_raw
