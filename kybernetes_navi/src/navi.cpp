#include "navi.h"

//globals----------------------------------------
//waypoints-----------------------
std::list<point> points;
//messages------------------------
ros::Subscriber sonarStream;
ros::Subscriber gpsLocStream;
ros::Subscriber bumperStream;
ros::Subscriber acceleromiter;
ros::Publisher motorStream;
ros::Publisher steeringStream;
//data----------------------------
//time-------------
float currVel;
float timeElapsed;
float heading;
//location---------
GPS_MSG_TYPE positionData;
//sonar------------
SONAR_MSG_TYPE sonarData;
//globals========================================

//main-------------------------------------------
int main(int argc, char** argv){

	//init-------------------------
	ros::init(argc, argv, "Navi");
	ros::NodeHandle kybernetes;
	
	//add waypoint
	if(argc % 2 != 0){
		ROS_INFO("HEY< LISTEN! THE POINTS ARE MISSMATCHED!");
	}
	for(int i = 1; i < argc; i += 2){
		point p;
		p.lat = atof(argv[i]);
		p.lon = atof(argv[i+1]);
		ROS_INFO("HEY LISTEN! I ADDED THE POINT LT: %f, LG: %f", p.lat, p.lon);
		points.push_back(p);
	}
	//init=========================

	//make publishers--------------
	motorStream = kybernetes.advertise<MOTOR_MSG_TYPE>(MOTOR_STREAM, 1000);
	steeringStream = kybernetes.advertise<STEERING_MSG_TYPE>(STEERING_STREAM, 1000);
	//make publishers==============

	//create callbacks-------------
	sonarStream = kybernetes.subscribe(SONAR_STREAM, 1000, Navi::hey_listen_sonarSays);
	gpsLocStream = kybernetes.subscribe(GPS_STREAM, 1000, Navi::hey_listen_gpsSays);
	//bumperStream = kybernetes.subscribe(BUMPER_STREAM, 1000, Navi::hey_listen_bumperSays);
	acceleromiter = kybernetes.subscribe(ACCEL_STREAM, 1000, NAVI::hey_listen_accelSays);
	//create callbacks=============

	//Hey listen!------------------
	while(ros::ok()){
		ros::spinOnce();
		Navi::hey_listen_you_should_go_here();
	}
	//Hey listen!==================
	return 0;
}
//main===========================================

//hey listen you should go here------------------
void Navi::hey_listen_you_should_go_here(){
	curVel = speed_from_sonar();
	float sd = dir_from_sonar() * (100 / (3.14159 / 2));
	float gd = sir_from_sonar() * (100 / (3.14159 / 2));

	if(sd < CROSSOVER_THRESH){
		steeringStream.publish(gd);
	} else {
		steeringStream.publish(sd);
	}
	
	if(dist_from_gps() < 10){
		curVel = 0;
	}
	
	motorStream.publish(curVel);
} 
//hey listen you should go here==================

//sensors----------------------------------------
void Navi::hey_listen_sonarSays(const SONAR_MSG_TYPE& msg){
	sonarData = msg;
	ROS_INFO("HEY, LISTEN! -- SONAR\n");
	ROS_INFO("[ %f, %f, %f, %f]\n",
		 msg.sonars[0], msg.sonars[1], msg.sonars[2], msg.sonars[3]);
}

void Navi::hey_listen_gpsSays(const GPS_MSG_TYPE& msg){
	positionData = msg;
	ROS_INFO("HEY, LISTEN! -- GPS\n");
	ROS_INFO("\n");
}

//void Navi::hey_listen_bumperSays(const BUMP_MSG_TYPE& msg){
//	ROS_INFO("HEY, LISTEN! -- BUMPER\n");
//	ROS_INFO("Left %f, Right %f \n", msg.left, msg.right);
//	if(hey_listen_is_this_the_goal()){
//		waypoints.pop_front();
//	}
//	if(bump.left && bump.right){
//		curVel = -30;
//		steeringStream.publish(-50);
//	} else if (bump.left){
//		curVel = -30;
//		steeringStream.publish(-50);
//	} else if (bump.right){
//		curVel = -30;
//		steeringStream.publish(50);
//	}
}
//sensors========================================

//direction and speed----------------------------
bool Navi::hey_listen_is_this_the_goal(){
	if(abs(positionData.longitude - waypoints.front().lon) < GOAL_THRESH){
		if(abs(positionData.latitude - waypoints.front().lat) < GOAL_THRESH){
			return true;
		}
	}
	return false;
}
float Navi::dir_from_sonar(){
	float sum;

	for(int i = 0; i < 4; i++){
		if(sonarData.sonars[i] < SONAR_STEER_THRESH){
			sum += sonarSteeringWeight[i] / sonarData.sonars[i] * 100;
		}
	}

	return sum;
}

//havarsine distance  a = sin^2(delt_lat/2) + cos(lat1)cos(lat2)sin^2(delt_lon/2)
//			c = 2 * atan2(sqrt(a), sqrt(1-a)
//			d = R*c
// r -> earths radius : 6371km

//bearing   theta = atan2(sin(delt_lon)*cos(lat2),
//			cos(lat1)sin(lat2) - sin(lat1)cos(lat2)cos(delt_lon)

float Navi::dir_from_gps(){
	float lt1 = positionData.longitude, lt2 = waypoint.front().lon;
	float ln1 = positionData.latitude, ln2 = waypoint.front().lat;

	lt1 = lt1 * 3.14159 / 180;
	lt2 = lt2 * 3.14159 / 180;
	ln1 = ln1 * 3.14159 / 180;
	ln2 = ln2 * 3.14159 / 180;	

	float theta = atan2(sin(ln2 - ln1)*cos(lt2),
			cos(lt1)*sin(lt2) - sin(lt1)*cos(lt2)*cos(ln2 - ln1));

	if(heading > 0){
		theta -= heading;
	} else {
		theta += heading;
	}
	return theta;
}

float Navi::dist_from_gps(){
	float lt1 = positionData.latitude, lt2 = waypoint.front().lat;
	float ln1 = positionData.longitude, ln2 = waypoint.front().lon;

	lt1 = lt1 * 3.14159 / 180;
	lt2 = lt2 * 3.14159 / 180;
	ln1 = ln1 * 3.14159 / 180;
	ln2 = ln2 * 3.14159 / 180;

	const float a = sin((lt1 - lt2)/2)*sin((lt1 - lt2)/2) + cos(lt1) * cos(lt2) * sin((ln1 - ln2)/2) * sin((ln1 - ln2)/2);
	const float c = 2 * atan2(sqrt(a), sqrt(1-a));
	const float d = 6371000 * c;

	return d;
}
37.567
float Navi::speed_from_sonar(){
	for(int i = 0; i < 5; i++){
		if(sonarData.sonars[i] < SONAR_SPEED_THRESH){
			return 30
		}
	}
	return 100;
}
//direction and speed============================
