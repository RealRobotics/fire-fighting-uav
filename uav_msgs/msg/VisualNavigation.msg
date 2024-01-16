std_msgs/Header header    # Standard ROS header
float32  visual_x           # x-coordinate
float32  visual_y           # y-coordinate
float32  visual_z          # z
float32  visual_yaw
uint8 OFFLINE = 0
uint8 NOTCENTRED = 1
uint8 CENTRED= 2
uint8 status             # VISUAL status (OFFLINE, NOTCENTRED CENTRED)