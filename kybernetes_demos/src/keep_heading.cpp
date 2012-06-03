#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <kdl/frames.hpp>

#include <kybernetes_demos/pid.h>

ros::Publisher velocityPublisher, directionPublisher;
sPID           pid;

// Angle Helpers
double normaliseDegrees(double degrees)
{
    double _d = degrees;
    if(_d >= 360.0)
        while(_d >= 360.0) _d -= 360.0;
    else if(_d <= 0.0)
        while(_d <= 0.0) _d += 360.0;
    return _d;
}

double getShortAngle(double a, double b)
{
    double angle = fmod(fabs(a - b), 360.0);
    if(angle > 180.0) angle = 360.0 - angle;
    if(a < b) angle *= -1;
    return angle;
}

// IMU callback
void imu_callback(const sensor_msgs::Imu& _imu)
{
    // We need to convert back to roll, pitch, and yaw
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    KDL::Rotation::Quaternion(_imu.orientation.x, 
                              _imu.orientation.y, 
                              _imu.orientation.z, 
                              _imu.orientation.w).GetRPY(roll, 
                                                         pitch, 
                                                         yaw);
    // Do something with RPY (Feed into steering PID)
    std_msgs::Float32 _position;
    std_msgs::Float32 _velocity;
    pid.pv = getShortAngle(0.0, yaw * (180 / M_PI));
    pid_update(&pid, 0.1);
    _position.data = pid.cv;
    _velocity.data = 50.0;
    //pid_dump(&pid);
    ROS_INFO("Input: %f, Output: %f", pid.pv, pid.cv);
    directionPublisher.publish(_position);
    velocityPublisher.publish(_velocity);
}

// Main Function
int main(int argc, char** argv)
{
    // Init the ROS node system
    ros::init(argc, argv, "keep_heading");
    ros::NodeHandle n;
    pid_init(&pid);
    pid.cv_min = -100;
    pid.cv_max =  100;
    
    // Subscribers and Publishers
    ros::Subscriber imu_subscriber = n.subscribe("/imu0/state", 1, &imu_callback);
    velocityPublisher = n.advertise<std_msgs::Float32>("/motion_interface/velocity", 1000);
    directionPublisher = n.advertise<std_msgs::Float32>("/motion_interface/direction", 1000);
    
    // Loop forever
    ros::spin();
    
    // Return success
    return 0;
}
