uint32 input_msg_id
uint8 status
uint8 UNSET = 0   # The status of battery is unknown
uint8 OK = 1 # When battery level is above mission critical threshold
uint8 MISSION_CRITICAL = 2 # When battery level is below mission critical threshold but above safety threshold
uint8 SAFETY_CRITICAL = 3 # When battery level is below safety critical level
uint8 BATTERY_READING_OUT_OF_SCOPE = 4 # When battery reading would be below 0 or above 100
