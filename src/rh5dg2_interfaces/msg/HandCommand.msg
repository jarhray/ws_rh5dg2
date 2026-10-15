# Fixed wire order: pinky root, ring root, middle root, middle splay,
# index root, index splay, pinky PIP, ring PIP, middle PIP, index PIP,
# thumb splay, thumb first bend, thumb second bend.
# Angles are signed register values in 0.1 degree, NOT ROS joint radians.
# ANGLE -1 means leave unchanged, including the two splay channels.
# Other ANGLE values are clamped to the fixed manual limits in registers.hpp.
# A nonzero timestamp is required; expired/future commands are rejected.
std_msgs/Header header
uint64 command_id
uint8 ANGLE=0
uint8 SPEED=1
uint8 FORCE=2
uint8 CURRENT_LIMIT=3
uint8 MODE=4
uint8 kind
int16[13] values
