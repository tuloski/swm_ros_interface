#include "swm_ros_interface.h"

using namespace std;

bool load_param_bool( bool def, string name ) {
	bool p;
	ros::NodeHandle n_param;
	if (!n_param.getParam (name, p)) {
		p = def;
	}
	cout << name << ": " << "\t" << p << endl;

	return p;
}

string load_param_string( string def, string name ) {
	string p;
	ros::NodeHandle n_param; 
	if (!n_param.getParam (name, p)) {
		p = def;
	}
	cout << name << ": " << "\t" << p << endl;

	return p;
}


#define default_namespace "/wasp"

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
	if( ns == "/" ) 
		ns = default_namespace;

	//pezzotto, remove the first 2 //
	ns = ns.substr( 2, ns.size()-1 );

	//---Load params
	bool pub_geopose = load_param_bool( false, nodename + "/pub/geopose" );	
	bool pub_system_status = load_param_bool(false, nodename + "/pub/system_status" );	
	bool pub_wasp_images = load_param_bool(false, nodename + "/pub/wasp_images" );	
	bool pub_artva = 	load_param_bool(false, nodename + "/pub/wasp_artva");
	bool sub_geopose = load_param_bool(false, nodename + "/sub/geopose");

	string swm_zyre_conf = load_param_string( "swm_zyre_config.json", nodename + "/swm_zyre_conf_file");
	//---

	//---publishers (TO SWM)
	if (pub_system_status) { //send the position of the bg to the SWM 
	  	subWaspBattery_ = _nh.subscribe("/" + ns + "/system_status", 0, &SwmRosInterfaceNodeClass::readBattery_publishSwm_wasp,this);
		cout << "Subscribing: \t [" + ns + "/system_status]" << endl; 
	} //battery

	if (pub_geopose) { //send the position of the wasp to the SWM 
		subWaspGeopose_ = _nh.subscribe("/" + ns + "/geopose", 0, &SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp,this);
		cout << "Subscribing: \t [" << ns + "/geopose]" << endl; 
	} //geopose

	if (pub_wasp_images) { //send the images data to the SWM 
		//subWaspCamera_ = _nh.subscribe("/" + ns + "camera_published", 0, &SwmRosInterfaceNodeClass::readCameraObservations_publishSwm,this);
		cout << "Subscribing: \t [" << ns + "/camera]" << endl; 
	} //images

	if( pub_artva ) {
		subWaspArtva_ = _nh.subscribe("/" + ns + "/artva_read", 0, &SwmRosInterfaceNodeClass::readArtva_publishSwm_wasp,this);
		cout << "Subscribing: \t [" << ns + "artva_read" << "]" << endl;
	} // Artva
	//---

	//---read from SWM
	if ( sub_geopose ) {
		pubBgGeopose_ = _nh.advertise<geographic_msgs::GeoPose>("/" + ns + "/geopose", 0);
		publishers.push_back(BG_GEOPOSE);
		rate_publishers.push_back(5);	//rate in Hz at which we want to read from SWM
		counter_publishers.push_back(0);
	} 
	//---

	rate = 100;	//TODO maybe pick rate of node as twice the highest rate of publishers
 
	std::string conf_path = ros::package::getPath( nodename.substr( 1, nodename.size()-1 ) );  
	string ubx_conf_path = conf_path + "/conf/" + swm_zyre_conf;
	config = load_config_file(str2char(ubx_conf_path));
	
	if (config == NULL) {
	  ROS_INFO("Unable to load config");
	  exit(0);
	}	
	self = new_component(config);
	if (self == NULL) {
		ROS_INFO("Unable to initialize component");
		exit(0);
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
}


void SwmRosInterfaceNodeClass::readArtva_publishSwm_wasp(const mavros::ArtvaRead::ConstPtr& msg ) {
	gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

  double rot_matrix[9];
	quat2DCM(rot_matrix, last_agent_pose.orientation); // TODO
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					last_agent_pose.position.latitude, last_agent_pose.position.longitude, last_agent_pose.position.altitude, 1}; // y,x,z,1 remember this is column-major!

  add_artva(self, matrix, msg->rec1_distance, msg->rec1_direction, msg->rec2_distance, msg->rec2_direction, utcTimeInMiliSec, str2char(ns));
}

void SwmRosInterfaceNodeClass::readBattery_publishSwm_wasp(const mms_msgs::Sys_status::ConstPtr& msg){
	gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	string battery_status = "HIGH";
  add_battery(self, msg->voltage_battery, str2char(battery_status),  utcTimeInMiliSec, str2char(ns));
}


void SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp(const geographic_msgs::GeoPose::ConstPtr& msg){
	gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

  last_agent_pose = *msg;

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
