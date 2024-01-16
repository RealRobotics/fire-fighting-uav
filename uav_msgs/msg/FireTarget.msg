std_msgs/Header header    # Standard ROS header
float64 Fire_x           # x-coordinate
float64 Fire_y           # y-coordinate
float64 Fire_depth       # depth
uint8 NO_FIRE = 0
uint8 DETECTED = 1
uint8 TRACKED = 2
uint8 status             # fire status (NO_FIRE, DETECTED, TRACKED)
