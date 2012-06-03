#!/usr/bin/env python
import roslib; roslib.load_manifest('kybernetes_motion')
import rospy
import math

from   std_msgs.msg    import Bool
from   std_msgs.msg    import Float32

# Publisher for the enabled status
velocityPublisher = rospy.Publisher('/motion_interface/velocity', Float32)
directionPublisher = rospy.Publisher('/motion_interface/direction', Float32)

# Global data
desiredHeading = 0.0
headingFound = False
direction = 0.0

# The velocity callback
def enabled_callback(on):
	# Alert the log to the change in state
	fuck = "this"

# Normalize the degrees
def normaliseD(degrees):
	# Normalise
	deg = degrees
	if deg >= 360:
		while deg >= 360: deg -= 360
	elif deg < 0:
		while deg < 0: deg += 360
	return deg

# Get shortest distance
def getShortAngle(a, b):
	angle = abs(a - b) % 360
	if angle > 180:
		angle = 360 - angle

	if a < b: angle = angle * -1
	return angle

# The direction callback
def heading_callback(heading):
	# Alert the log to the change in velocity
	global desiredHeading
	global headingFound
	
	_heading = normaliseD(heading.data)

	if headingFound == False:
		headingFound = True
		desiredHeading = _heading

	difference = getShortAngle(_heading, desiredHeading)
	if difference > 33.3: difference = 33.3
	if difference < -33.3: difference = -33.3
	directionPublisher.publish(Float32(difference * 3))
	rospy.loginfo("Diff = %f", difference * 3)

# Main processing (routing) node - takes the ros messages and publishes them to the device
def keep_heading():
	rospy.init_node('keep_heading', anonymous = False)
	global desiredHeading
	global headingFound
	desiredHeading = 0.0
	headingFound = False

	# Subscribe to velocity and direction messages
	rospy.Subscriber("/motion_interface/enabled", Bool, enabled_callback)
	rospy.Subscriber("/imu0/state", Float32, heading_callback)
	
	# Perform a loop checking on the enabled status of the drivers
	while not rospy.is_shutdown():
		velocityPublisher.publish(Float32(80.0))
		rospy.sleep(0.25)

# Start the main class
if __name__ == '__main__':
	try:
		keep_heading()
	except rospy.ROSInterruptException: pass
