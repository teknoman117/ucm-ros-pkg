/**
 * This node pulls images from a UVC camera and publishes either a YUYV, RGB24, or JPEG image
 * It also provides an interface to the Logitech Quickcam Orbit AF's pan/tilt system
 */

#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "uvc_camera/framegrabber_uvc.h"
#include "uvc_camera/MovePTZ.h"

// Storage copies for the PTZ state
signed short _pan = 0;
signed short _tilt = 0;
signed short _zoom = 0;

// Camera unit
uvc_camera::FrameGrabber_UVC *camera;

bool moveptz_relative(uvc_camera::MovePTZ::Request &request, uvc_camera::MovePTZ::Response &response)
{
    // Catch the bounds
    signed short pan = request.pan - ((request.pan > 0) ? (request.pan % 64) : -(request.pan % 64));
    signed short tilt = request.tilt - ((request.tilt > 0) ? (request.tilt % 64) : -(request.tilt % 64));
    signed short zoom = request.zoom;
    //ROS_INFO("r> p:%d t:%d z:%d", pan, tilt, zoom);
    
    // Perform the move
    if(!camera->movePTZ(pan, tilt, zoom))
    {
        // Assemble the new state
        response.rpan = (_pan += pan);
        response.rtilt = (_tilt += tilt);
        response.rzoom = (_zoom += zoom);
    
        // Return whether or not shit got done
        return true;
    }
    
    ROS_WARN("Move did not succeed - if motion occurred, shit went down");
    return false;
}

bool moveptz_absolute(uvc_camera::MovePTZ::Request &request, uvc_camera::MovePTZ::Response &response)
{
    // Catch the bounds
    signed short pan = request.pan - ((request.pan > 0) ? (request.pan % 64) : -(request.pan % 64));
    signed short tilt = request.tilt - ((request.tilt > 0) ? (request.tilt % 64) : -(request.tilt % 64));
    signed short zoom = request.zoom;
    //ROS_INFO("a> p:%d t:%d z:%d", pan, tilt, zoom);
    
    // Calculate difference
    signed short dPan = pan - _pan;
    signed short dTilt = tilt - _tilt;
    signed short dZoom = zoom - _zoom;
    //ROS_INFO("d> p:%d t:%d z:%d", dPan, dTilt, dZoom);
    
    // Perform the move
    if(!camera->movePTZ(dPan, dTilt, dZoom))
    {
        // Assemble the new state
        response.rpan = _pan = pan;
        response.rtilt = _tilt = tilt;
        response.rzoom = _zoom = zoom;
    
        // Return whether or not shit got done
        return true;
    }
    
    ROS_WARN("Move did not succeed - if motion occurred, shit went down");
    return false;
}

int main(int argc, char **argv)
{
    // Init the ROS node system
    ros::init(argc, argv, "uvc_camera");
    ros::NodeHandle n;
    sensor_msgs::Image image;
    sensor_msgs::CompressedImage compressed_image;
  
    // Get the parameters
    std::string videodev, mode;
    int width, height;
    if(!n.getParam("device", videodev)) videodev = "/dev/video0";
    if(!n.getParam("width", width)) width = 320;
    if(!n.getParam("height", height)) height = 240;
    if(!n.getParam("mode", mode)) mode = "rgb8";
  
    // Create the image publisher
    ros::Publisher broadcast;
    if(mode == "jpeg")
        broadcast = n.advertise<sensor_msgs::CompressedImage>("compressed_image", 1000);
    else 
        broadcast = n.advertise<sensor_msgs::Image>("image", 1000);
  
    // Set up the camera
    camera = new uvc_camera::FrameGrabber_UVC(videodev, width, height, mode);
    if(mode == "jpeg")
        camera->grab(compressed_image);
    else   
        camera->grab(image);
    camera->resetPTZ();
    ROS_INFO("Configured");
    ros::Duration(5.0).sleep();
  
    // Advertise ptz services
    ros::ServiceServer moveservice_relative = n.advertiseService("moveptz_relative", moveptz_relative);
    ros::ServiceServer moveservice_absolute = n.advertiseService("moveptz_absolute", moveptz_absolute);
  
    // main loop
    while (ros::ok())
    {
        // Pubish CompressedImages or Images
        if(mode == "jpeg") {
            camera->grab(compressed_image);
            broadcast.publish(compressed_image);
        } else {   
            camera->grab(image);
            broadcast.publish(image);
        }
        ros::spinOnce();
    }

    delete camera;
    return 0;
} 
