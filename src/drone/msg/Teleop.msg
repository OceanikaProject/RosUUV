# 16 bit value for switches 0bfedcba9876543210
# byte 0: 0 - tangage off, 1 - tangage on
# byte 1: 0 - yaw off, 1 - yaw on
# byte 2-3: 0 - hand clenches, 1 - hand neutral, 2 hand unclenches
# byte 4-5: 0 - 30% speed mode, 1 - 60% speed mode, 2 - 100% speed mode
# byte 6-7: 0 - lights off, 1-3 change brightness
# byte 8: 0 - stabilization off, 1 - stabilization on
# byte 9: 0 - motors off, 1 - motors on
std_msgs/Header header
uint16 status
int8 lx
int8 ly
int8 rx
int8 ry