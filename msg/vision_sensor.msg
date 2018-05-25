#lib/goe
# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#

float32 vision_x			# vision_x, (metres)
float32 vision_y			# vision_y, (metres)
float32 vision_z		        # vision_z, (metres)
float32 vision_vx                       #(m/s)
float32 vision_vy                       #(m/s)
float32 vision_vz                       #(m/s)
int32 vision_distortion_x		# vision_distortion_x,(metres) 
int32 vision_distortion_y             	# vision_distortion_y,(metres) 
int8 status				#is the vision data valid. 0:invalid, 1:valid
