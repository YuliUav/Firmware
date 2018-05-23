#lib/goe
# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#
float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)
float32 yaw 			# Euler yaw angle relative to NED earth-fixed frame, -PI..+PI, (radians)
float32 vel_n			# East velocity in NED earth-fixed frame, (metres/sec)
float32 vel_e			# North velocity in NED earth-fixed frame, (metres/sec)
float32 vel_d			# Down velocity in NED earth-fixed frame, (metres/sec)
float32 x			#position from vision
float32 y			#position from vision
float32 z			#position from vision
float32 vel_x			#velocity from vision
float32 vel_y			#velocity from vision
float32 vel_z			#velocity from vision
bool track_enabled 		#true if the target position precesion is gool enough
bool catched			#true if the craw is closed
bool climb_finished


