#include "swm_ros_interface.h"

using namespace std;


char* str2char( string str ) {
	char *c = new char[ str.length()+1 ];
	strcpy( c, str.c_str());
	return c;
}

SwmRosInterfaceNodeClass::SwmRosInterfaceNodeClass() {

	counter_print = 0;
	
	if (_nh.hasParam("/swm_interfce/pub/geopose")){
		subSelfGeopose_ = _nh.subscribe("geopose", 10, &SwmRosInterfaceNodeClass::readGeopose_publishSwm,this);
	} else {
	}

	if (_nh.hasParam("/swm_interfce/sub/bg_geopose")){
		pubBgGeopose_ = _nh.advertise<geographic_msgs::GeoPose>("/bg/geopose",10);
		publishers.push_back(BG_GEOPOSE);
		rate_publishers.push_back(10);	//rate in Hz at which we want to read from SWM
		counter_publishers.push_back(0);
	} else {
	}

	rate = 100;	//TODO maybe pick rate of node as twice the highest rate of publishers

	// SWM
	ns = ros::this_node::getNamespace();
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
	agent_initialized = false;

	//Initialize agent
	double matrix[16] = { 1, 0, 0, 0,
		               			0, 1, 0, 0,
		               			0, 0, 1, 0,
		               			0, 0, 0, 1}; // y,x,z,1 remember this is column-major!
	assert(add_agent(self, matrix, 0.0, str2char(ns) ));
	
}

void SwmRosInterfaceNodeClass::readGeopose_publishSwm(const geographic_msgs::GeoPose::ConstPtr& msg){
	ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
	utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!
	
	update_pose(self, matrix, utcTimeInMiliSec, str2char(ns) );
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

void SwmRosInterfaceNodeClass::loop_handle()
{

	ros::Rate r(rate);

	while( ros::ok() ) {

		counter_print++;
		for (int i=0; i<publishers.size(); i++){
			
			counter_publishers[i]++;

			switch (publishers[i]){
				case BG_GEOPOSE:
					if (counter_publishers[i] >= rate/rate_publishers[i]){
						//TODO poll SWM (we wait to have the function for the full pose)
						//TODO publish topic
					}
					break;
			}

							
		}
	
		//---Non mi è molto chiara questa parte!! Di che tipo è srv_query?
		/*if (counter_print>20){
			std::string query = "query_random";
			srv_query.request.query = query;
			srv_query.request.param1 = 1.0;
			srv_query.request.param2 = 2.0;
			srv_query.request.param3 = 3.0;
			srv_query.request.param4 = 4.0;
			srv_query.request.param5 = 5.0;
			srv_query.request.param6 = 6.1;
			srv_query.request.param7 = 7.0;
			if (client_query.call(srv_query))
			{
				ROS_INFO("Query test: %s - %f - %f", srv_query.response.query_answ.c_str(), srv_query.response.answ1, srv_query.response.answ2);
			}
			else
			{
				ROS_ERROR("Failed to call service query_swm");
			}
			counter_print = 0;
		}*/
		//----


		r.sleep();
	}
}




void SwmRosInterfaceNodeClass::run() {

	ros::Rate loop_rate(rate);	
	ROS_INFO_ONCE("SIM: RUNNING");

	//start loop handle as a thread. The execution time is specified by rate parameter
	boost::thread loop_handle_t( &SwmRosInterfaceNodeClass::loop_handle, this);
	ros::spin();

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "swm_ros_interface");	
	SwmRosInterfaceNodeClass swm_ros_interfaceNode;
	swm_ros_interfaceNode.run();

	return 0;
}
