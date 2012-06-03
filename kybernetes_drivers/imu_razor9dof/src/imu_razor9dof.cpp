#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <seriallib/serialstream.h>
#include <jsoncpp/json.h>

// Global Data
SerialDevice     *imu = NULL;
ros::Publisher    broadcast;
sensor_msgs::Imu _imu_message;
ros::Timer        timer;
bool              available = false;

// Timer shit
void publish_notification(const ros::TimerEvent& event)
{
    if(!available) return;
    broadcast.publish(_imu_message);
    available = false;
}

// Main Node Function
int main (int argc, char** argv)
{
    // Init the ROS node system
    ros::init(argc, argv, "imu_razor9dof");
    ros::NodeHandle n;

    // Get the parameters
    std::string imudev;
    int baudrate;
    double period;
    if(!n.getParam("device", imudev)) imudev = "/dev/ttyUSB0";
    if(!n.getParam("baudrate", baudrate)) baudrate = 57600;
    if(!n.getParam("period", period)) period = 0.1;

    // Create the imu publisher
    imu       = new SerialDevice(imudev, B57600);
    broadcast = n.advertise<sensor_msgs::Imu>("state", 1000);
    timer     = n.createTimer(ros::Duration(period), publish_notification);

    // Configure the serial port further
    struct termios settings;
    imu->get_termios(&settings);
    settings.c_lflag |= (ICANON | ECHO | ECHOE);
    imu->set_termios(&settings);
    imu->flush();

    // Get input data bitches
    int count = 0;
    while(ros::ok())
    {
        // Operate only if lines are in the buffer
        unsigned int b = imu->available();
        while(b > 0) {
            // Allocate a buffer large enough to hold the sentence
            std::string buffer;
            buffer.resize(b);

            // Fetch the data from the serial buffer
            imu->read((char *)buffer.data(), b);
            if(count == 1) {
                // dump the first sentence block, its probably corrupted
                count++;
                break;
            }

            // Parse the values
            Json::Value root;
            Json::Reader reader;
            if (!reader.parse( buffer, root ))
            {
                //ROS_WARN("[IMU] Invalid message received");
                break;
            }

            // Compile into the IMU message
            _imu_message.orientation = tf::createQuaternionMsgFromRollPitchYaw(root["angles"][0].asFloat() * (M_PI / 180.0), 
                                                                               root["angles"][1].asFloat() * (M_PI / 180.0), 
                                                                               root["angles"][2].asFloat() * (M_PI / 180.0));
            available = true;
            //ROS_INFO("Yaw = %lf", root["angles"][2].asFloat() * (M_PI / 180.0));

            // Re-evaluate the buffer data count
            b = imu->available();
        }

        // Give some time to the node daemon
        ros::spinOnce();
    }
    return 0;
}
