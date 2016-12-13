#include "swm_ros_interface.h"

using namespace std;


char* str2char( string str ) {
	char *c = new char[ str.length()+1 ];
	strcpy( c, str.c_str());
	return c;
}

SwmRosInterfaceNodeClass::SwmRosInterfaceNodeClass() {

	counter_print = 0;
	
	//---params:
	// pub: subscription to rostopic, advertising on the SWM
	// sub: subscription to the SWM, publishing on rostopic
	//--
	ns = ros::this_node::getNamespace();
	string nodename = ros::this_node::getName();
	//---publishers (TO SWM)
	if (_nh.hasParam(nodename + "/pub/publish_operator_geopose")) { //send the position of the bg to the SWM 
		subSelfGeopose_ = _nh.subscribe("/CREATE/human_pose", 0, &SwmRosInterfaceNodeClass::readGeopose_publishSwm,this);
		cout << "Subscribing: \t [operator geopose]: /CREATE/human_pose" << endl; 
	}

	if (_nh.hasParam(nodename + "/pub/wasp_geopose")) { //send the position of the wasp to the SWM 
		subWaspGeopose_ = _nh.subscribe("geopose", 0, &SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp,this);
		cout << "Subscribing: \t [wasp geopose]" << endl; 
	}

	if (_nh.hasParam(nodename + "/pub/wasp_images")) { //send the images data to the SWM 
		//subWaspCamera_ = _nh.subscribe("camera_published", 0, &SwmRosInterfaceNodeClass::readCameraObservations_publishSwm,this);
		cout << "Subscribing: \t [wasp camera]" << endl; 
	}
	//---
	if (_nh.hasParam(nodename + "/sub/publish_operator_geopose")){
		pubBgGeopose_ = _nh.advertise<geographic_msgs::GeoPose>("/bg/geopose",10);
		publishers.push_back(BG_GEOPOSE);
		rate_publishers.push_back(5);	//rate in Hz at which we want to read from SWM
		counter_publishers.push_back(0);
	} 

	rate = 100;	//TODO maybe pick rate of node as twice the highest rate of publishers

	//gettimeofday(&tp, NULL);

	//---SWM
	char* pPath;
	pPath = getenv ("UBX_ROBOTSCENEGRAPH_DIR");
	if (pPath==NULL) {
  	ROS_ERROR("UBX_ROBOTSCENEGRAPH_DIR env not set!");
  	exit(0);
  }
  
  string ubx_conf_path(pPath);
	ubx_conf_path += "/examples/zyre/swm_zyre_config.json"; //the json name could be a param
	config = load_config_file(str2char(ubx_conf_path));//"swm_zyre_config.json");
	
	if (config == NULL) {
	  ROS_INFO("Unable to load config");
	  return;
	}	
	self = new_component(config);
	if (self == NULL) {
		ROS_INFO("Unable to initialize component");
		return;
	}
	//---

	agent_initialized = false;

	//Initialize agent
	double matrix[16] = { 1, 0, 0, 0,
		               			0, 1, 0, 0,
		               			0, 0, 1, 0,
		               			0, 0, 0, 1}; // y,x,z,1 remember this is column-major!
	
	//string agent_name = "busy_genius";
	assert(add_agent(self, matrix, 0.0, str2char(ns) ));
	

	cout << "NS: " << ns << endl;
}

void SwmRosInterfaceNodeClass::readGeopose_publishSwm(const geographic_msgs::GeoPose::ConstPtr& msg){
	//ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
	//utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
	gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!
	
	string agent_name = "busy_genius";
	update_pose(self, matrix, utcTimeInMiliSec, str2char(agent_name) );
}

void SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp(const geographic_msgs::GeoPose::ConstPtr& msg){
	gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	//ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
	//utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!
	
	update_pose(self, matrix, utcTimeInMiliSec, str2char(ns) );
}

void SwmRosInterfaceNodeClass::readCameraObservations_publishSwm(const camera_handler_sherpa::Camera::ConstPtr& msg){
	gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->geopose.orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->geopose.position.latitude, msg->geopose.position.longitude, msg->geopose.position.altitude, 1}; // y,x,z,1 remember this is column-major!
	//assert(add_image(self, matrix, utcTimeInMiliSec, str2char(ns), str2char(msg->path_photo)));		TODO uncomment when Sebastian solves
}


void quat2DCM(double (&rot_matrix)[9], geometry_msgs::Quaternion quat){
	
	rot_matrix[0] = 1-2*(quat.y*quat.y+quat.z*quat.z);
	rot_matrix[1] = 2*(quat.x*quat.y-quat.w*quat.z);
	rot_matrix[2] = 2*(quat.x*quat.z+quat.w*quat.y);
	rot_matrix[3] = 2*(quat.x*quat.y+quat.w*quat.z);
	rot_matrix[4] = 1-2*(quat.x*quat.x+quat.z*quat.z);
	rot_matrix[5] = 2*(quat.y*quat.z-quat.w*quat.x);
	rot_matrix[6] = 2*(quat.x*quat.z-quat.w*quat.y);
	rot_matrix[7] = 2*(quat.y*quat.z+quat.w*quat.x);
	rot_matrix[8] = 1-2*(quat.x*quat.x+quat.y*quat.y);
	
}

void SwmRosInterfaceNodeClass::main_loop()
{

	ros::Rate r(rate);

	//---msgs
	geographic_msgs::GeoPose gp;
	//---

	while( ros::ok() ) {

		ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
		utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;

		counter_print++;
		for (int i=0; i<publishers.size(); i++){
			
			counter_publishers[i]++;

			switch (publishers[i]){
				case BG_GEOPOSE:
					if (counter_publishers[i] >= rate/rate_publishers[i]){
						string agent_name = "busy_genius";
						double x,y,z;
						//get_position(self, &gp.position.latitude, &gp.position.longitude, &gp.position.altitude, utcTimeInMiliSec, str2char(agent_name));	
						//get_position(self, &x, &y, &z, utcTimeInMiliSec, "test");
						pubBgGeopose_.publish( gp );
						counter_publishers[i] = 0;
					}
					break;
			}						
		}
	


		r.sleep();
	}
}




void SwmRosInterfaceNodeClass::run() {

	ros::Rate loop_rate(rate);	
	ROS_INFO_ONCE("SIM: RUNNING");

	//start loop handle as a thread. The execution time is specified by rate parameter
	boost::thread loop_handle_t( &SwmRosInterfaceNodeClass::main_loop, this);
	ros::spin();

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "swm_ros_interface");	
	SwmRosInterfaceNodeClass swm_ros_interfaceNode;
	swm_ros_interfaceNode.run();

	return 0;
}
