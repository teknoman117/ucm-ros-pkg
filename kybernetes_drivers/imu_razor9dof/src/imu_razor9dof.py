#!/usr/bin/env python
import roslib; roslib.load_manifest('imu_razor9dof')
import rospy
import serial
import simplejson             as json
from   sensor_msgs.msg        import Imu
from   std_msgs.msg           import Float32

# Publisher for kybernetes' sensor data
imuPublisher = rospy.Publisher('imu', Imu)
cmpPublisher = rospy.Publisher('heading', Float32)
interface = serial.Serial(rospy.get_param("port"), rospy.get_param("baudrate"), timeout=1)

# Main processing (routing) node - takes in ros messages from device
def sensor_interface():
	rospy.init_node('imu_razor9dof', anonymous = False)
	interface.flushInput()
	
	# Perform a loop checking on the enabled status of the drivers
	while not rospy.is_shutdown():
		# Fetch a JSON message from the controller and process (or atleast attempt to)
		try:
			# Get the object and create some messages
			_object = json.loads(interface.readline())
			_imu = Imu();
			_heading = Float32()
			
			# Get the imu data (fuck quaternions!!)
			_imu.header = rospy.Header()
			roll = _object['angles'][0]
			pitch = _object['angles'][1]
			yaw = _object['angles'][2]
			#_imu.orientation = tf::createQuaternionFromRPY(roll, pitch, yaw)
			_imu.linear_acceleration.x = _object['accel'][0]
			_imu.linear_acceleration.y = _object['accel'][1]
			_imu.linear_acceleration.z = _object['accel'][2]

			# Get the heading data
			#_heading.data = _object['heading']
			_heading.data = _object['angles'][2]
			# Publish the data
			imuPublisher.publish(_imu);
			cmpPublisher.publish(_heading);
		except json.decoder.JSONDecodeError:
			rospy.logwarn("Invalid message received")
		except: pass

# Start the main class
if __name__ == '__main__':
	try:
		sensor_interface()
	except rospy.ROSInterruptException: pass
