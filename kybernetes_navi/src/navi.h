/*
 *  John Rice
 *
 *  -Navi-
 *
 * Hey, listen! This is the navigation node for 
 * kybernetes.  It needs input from the gps, the
 * acceleromiter, and the velocity measurer...
 *
 * Hey, listen! Idealy it wont need input from
 * the gps... just a list of coords to go to.
 * Those can be converted to angle and distance, 
 * right?
 *
 * Hey, listen! The sonars give an array from 0 to
 * 4 starting on the left.
 *
 * hey_listen_you_should_go_here is the main loop
 * 
 * the hey_listen_*senseor* are callbacks
 * to the publishers
 *
 * Hey, listen! The stering needs -100 to 100 and 
 * the motor takes cm per second.  Hey, listen! -100
 * is right.
 *
 * Hey, listen!  this makes use of havarsine distance
 * and some wierd bearing formula! look it up, its cool!
 */

//includes---------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <kybernetes_sensors/Sonars.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <list>
#include <cstdlib>
#include <math>
//includes=======================================

//defines----------------------------------------
#define MOTOR_STREAM  "motion_interface/velocity"
#define MOTOR_MSG_TYPE std_msgs::Float32
#define STEERING_STREAM "/motion_interface/direction"
#define STEERING_MSG_TYPE std_msgs::Float32
#define SONAR_STREAM "/sensor_interface/sonars"
#define SONAR_MSG_TYPE kybernetes_sensors::Sonars
#define GPS_STREAM "/fix"
#define GPS_MSG_TYPE sensor_msgs::NavSatFix
#define BUMP_STREAM
#define BUMP_MSG_TYPE
#define ACCEL_STREAM "/imu0/heading"
#define ACCEL_MSG_TYPE sensor_msgs::Float32
#define SONAR_STEER_THRESH 40
#define SONAR_SPEED_THRESH 60
#define GOAL_THRESH 0.01
const float sonarSteeringWeight[] = {0.5, 0.1, 0.0, -0.1, -0.5};
//defines========================================

//prototypes-------------------------------------
namespace Navi{
	void hey_listen_you_should_go_here();	//controll
	void hey_listen_sonarSays(const SONAR_MSG_TYPE& msg);		//sonar callback
	void hey_listen_gpsSays(const GPS_MSG_TYPE& msg);		//gps callback
	//void hey_listen_bumperSays(const BUMP_MSG_TYPE& msg);		//bumper callback
	bool hey_listen_is_this_the_goal();
	float dir_from_sonar();
	float dir_from_gps();
	float dist_from_gps();
	float speed_from_sonar();

	struct point{
		float lat, lon;
	}
};
//prototypes=====================================
