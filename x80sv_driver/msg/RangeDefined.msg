# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured. 
# This message is not appropriate for fixed-range obstacle detectors, 
# such as the Sharp GP2D15. This message is also not appropriate for laser 
# scanners. See the LaserScan message if you are working with a laser scanner.

sensor_msgs/Range range			
			
float64 angleFromCenter		# the angle (rad) the sensor has in relation to the centroid of the robot

float64 distanceFromCenter	# the distance (m) the sensor has in relation to the centroid of the robot

float64 yOffsetFromCenter	# the offset (m) the sensor has on the y axis in relation to the centroid of the robot