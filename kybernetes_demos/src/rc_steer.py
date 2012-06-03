#!/usr/bin/env python
import roslib; roslib.load_manifest('kybernetes_demos')
import rospy
from   std_msgs.msg    import Float32
from   sensor_msgs.msg import Joy

# Publisher for the motion statuses
velocityPublisher = rospy.Publisher('/motion_interface/velocity', Float32)
directionPublisher = rospy.Publisher('/motion_interface/direction', Float32)

# The joystick callback
def joystick_callback(joy_state):
	# Remap the RC steering axis to direction
	_direction = Float32(joy_state.axes[1])
	directionPublisher.publish(_direction)
	
	# Alert the log to the change in velocity
	rospy.loginfo("Setting direction to %f us", _direction.data)

# Main processing (routing) node - takes the ros messages and publishes them to the device
def rc_steer():
	rospy.init_node('rc_steer', anonymous = False)
	
	# Subscribe to velocity and direction messages
	rospy.Subscriber("/motion_interface/rc", Joy, joystick_callback)
	
	# Perform a loop checking on the enabled status of the drivers
	rospy.spin()
	
	# Perform closing operations (like stopping motors)
	velocityPublisher.publish(Float32(0.0))
	directionPublisher.publish(Float32(1500.0))

# Start the main class
if __name__ == '__main__':
	try:
		rc_steer()
	except rospy.ROSInterruptException: pass

