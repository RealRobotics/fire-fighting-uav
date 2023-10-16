uint32 input_msg_id
uint8 status
uint8 UNSET = 0   # The status of battery is unknown
uint8 OK = 1 # When battery level is above mission critical threshold
uint8 MISSION_CRITICAL = 2 # When battery level is below mission critical threshold but above safety threshold
uint8 SAFETY_CRITICAL = 3 # When battery level is below safety critical level
