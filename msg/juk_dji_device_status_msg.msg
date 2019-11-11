uint8 CONTROL_BY_RC = 1
uint8 CONTROL_BY_MOBILE = 2
uint8 CONTROL_BY_SDK = 3

uint8 ARMED = 1
uint8 DISARMED = 0

uint8 GEAR_UP
uint8 GEAR_DOWN

uint8 ON_GROUND_STANDBY = 1
uint8 TAKEOFF = 2
uint8 IN_AIR_STANDBY = 3
uint8 LANDING = 4
uint8 FINISHING_LANDING = 5

uint8 authority
uint8 armed
uint8 gear
uint16 voltage	#volt*100
time changeTime	 #time of last authority change
