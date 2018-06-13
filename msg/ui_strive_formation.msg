# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#

float64 lat		# Latitude, (degrees)
float64 lon		# Longitude, (degrees)
float32 alt		# Altitude AMSL, (meters)

float32 vel_n		# North velocity in NED earth-fixed frame, (metres/sec)
float32 vel_e		# East velocity in NED earth-fixed frame, (metres/sec)
float32 vel_d		# Down velocity in NED earth-fixed frame, (metres/sec)

int8 status    	        # current state, 0:manual, 1:altctl, 2:posctl, 3:mission, 4:loiter, 5:rtl, 6:acro, 7:offboard, 8:stab, 9:ratitude, 10:takeoff, 11:land, 12:followtarg, 13:max
int8 kill 		# kill 0:normal, 1:kill(stop motors)
uint8 sysid		# sysID
