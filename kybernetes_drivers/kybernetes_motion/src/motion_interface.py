#!/usr/bin/env python
import roslib; roslib.load_manifest('kybernetes_motion')
import rospy
import serial
import simplejson      as json
from   std_msgs.msg    import Bool
from   std_msgs.msg    import Float32
from   sensor_msgs.msg import Joy

# Publisher for the enabled status
enabledPublisher = rospy.Publisher('/motion_interface/enabled', Bool)
joystickPublisher = rospy.Publisher('/motion_interface/rc', Joy)
interface = serial.Serial(rospy.get_param("/motion_interface/port"), rospy.get_param("/motion_interface/baudrate"), timeout=1)

# The velocity callback
def velocity_callback(velocity):
	# Generate and send the JSON message to set the velocity
	message = json.dumps({'velocity':velocity.data})
	interface.write(message)
	interface.write('\r')
	
	# Alert the log to the change in velocity
	rospy.loginfo("Setting velocity to %f m/s", velocity.data)

# The direction callback
def direction_callback(direction):
	# Generate the JSON message to set the velocity
	message = json.dumps({'direction':direction.data})
	interface.write(message)
	interface.write('\r')
	
	# Alert the log to the change in velocity
	rospy.loginfo("Setting direction to %f degrees", direction.data)

# Main processing (routing) node - takes the ros messages and publishes them to the device
def motion_interface():
	rospy.init_node('motion_interface', anonymous = False)
	interface.flushInput()

	# Subscribe to velocity and direction messages
	rospy.Subscriber("/motion_interface/velocity", Float32, velocity_callback)
	rospy.Subscriber("/motion_interface/direction", Float32, direction_callback)
	
	# Perform a loop checking on the enabled status of the drivers
	while not rospy.is_shutdown():
		# Build a message to send to the controller (request packet)
		message = json.dumps({'request' : ['enabled', 'throttle', 'steering']});
		interface.write(message)
		interface.write('\r')
		#rospy.loginfo(message)
		
		# Fetch a JSON message from the controller and process (or atleast attempt to)
		try:
			_object = json.loads(interface.readline());
			enabledPublisher.publish(Bool(_object['enabled']))
			_joy = Joy()
			_joy.header = rospy.Header()
			_joy.axes.append(_object['throttle'])
			_joy.axes.append(_object['steering'])
			joystickPublisher.publish(_joy)
		except json.decoder.JSONDecodeError:
			enabledPublisher.publish(Bool(False))
			rospy.logwarn("Invalid message received")
		except: pass

		rospy.sleep(0.1);
		
	# Perform closing operations (like stopping motors)
	message = json.dumps({'velocity' : 0.0, 'direction' : 0.0})
	interface.write(message)
	interface.write('\r')

# Start the main class
if __name__ == '__main__':
	try:
		motion_interface()
	except rospy.ROSInterruptException: pass
