#!/usr/bin/env python
import roslib; roslib.load_manifest('kybernetes_sensors')
import rospy
import serial
import simplejson             as json
from   kybernetes_sensors.msg import Sonars
from   kybernetes_sensors.msg import Bumpers
from   std_msgs.msg           import String
from   std_msgs.msg           import Bool
from   std_msgs.msg           import Float32

# Publisher for kybernetes' sensor data
sonarPublisher = rospy.Publisher('/sensor_interface/sonars', Sonars)
bumperPublisher = rospy.Publisher('/sensor_interface/bumpers', Bumpers)
interface = serial.Serial(rospy.get_param("/sensor_interface/port"), rospy.get_param("/sensor_interface/baudrate"), timeout=1)

# Main processing (routing) node - takes in ros messages from device
def sensor_interface():
	rospy.init_node('sensor_interface', anonymous = False)
	interface.flushInput()
	
	# Perform a loop checking on the enabled status of the drivers
	while not rospy.is_shutdown():
		# Fetch a JSON message from the controller and process (or atleast attempt to)
		try:
			# Get the object and create some messages
			_object = json.loads(interface.readline());
			_sonars = Sonars()
			_bumpers = Bumpers()
			
			# Get the sonar data
			_sonars.header = rospy.Header()
			_sonars.sonars.append(_object['sonars'][0])
			_sonars.sonars.append(_object['sonars'][1])
			_sonars.sonars.append(_object['sonars'][2])
			_sonars.sonars.append(_object['sonars'][3])
			_sonars.sonars.append(_object['sonars'][4])
			
			# Get the bumper data
			_bumpers.header = rospy.Header()
			_bumpers.bumpers.append(_object['bumpers'][0])
			_bumpers.bumpers.append(_object['bumpers'][1])
			_bumpers.bumpers.append(_object['bumpers'][2])
			_bumpers.bumpers.append(_object['bumpers'][3])
			
			# Publish the data
			sonarPublisher.publish(_sonars);
			bumperPublisher.publish(_bumpers);
		except json.decoder.JSONDecodeError:
			rospy.logwarn("Invalid message received")
		except: pass

# Start the main class
if __name__ == '__main__':
	try:
		sensor_interface()
	except rospy.ROSInterruptException: pass
