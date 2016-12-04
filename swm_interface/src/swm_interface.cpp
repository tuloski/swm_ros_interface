//ROS
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geographic_msgs/GeoPose.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"

//SWM
#include <zyre.h>
#include <jansson.h>
#include <uuid/uuid.h>
#include <string.h>
#include "swmzyre.h"

//C++
#include <vector>
#include <string>
#include <math.h>

class SwmInterfaceNodeClass {
public:
	SwmInterfaceNodeClass(ros::NodeHandle& node){

		n_=node;
		
		rate = 5;
		counter_print = 0;
		
		if (n_.hasParam("/swm_interfce/pub/geopose")){
			subSelfGeopose_ = n_.subscribe("geopose", 10, &SwmInterfaceNodeClass::readGeopose_publishSwm,this);
		} else {
		}
	
		if (n_.hasParam("/swm_interfce/sub/bg_geopose")){
			pubBgGeopose_ = n_.advertise<geographic_msgs::GeoPose>("/bg/geopose",10);
			publishers.push_back("/bg/geopose");
		} else {
		}
		// SWM
		ns = ros::this_node::getNamespace();
		config_folder[255] = { SWM_ZYRE_CONFIG_DIR };
		config_name[] = "swm_zyre_config.json";
		config_file[512] = {0};
		snprintf(config_file, sizeof(config_file), "%s/%s", config_folder, config_name);
		config = load_config_file(config_file);//"swm_zyre_config.json");
		if (config == NULL) {
		  ROS_INFO("Unable to load config");
		  return -1;
		}
		self = new_component(config);
		if (self == NULL) {
			ROS_INFO("Unable to initialize component");
			return -1;
		}
		agent_initialized = false;
		//Initialize agent
		double matrix[16] = { 1, 0, 0, 0,
			               0, 1, 0, 0,
			               0, 0, 1, 0,
			               0, 0, 0, 1}; // y,x,z,1 remember this is column-major!
		assert(add_agent(self, matrix, 0.0, ns));
	}

	void readGeopose_publishSwm(const geographic_msgs::GeoPose::ConstPtr& msg){
		ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
		utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
		double rot_matrix[9];
		quat2DCM(rot_matrix, msg->orientation);
		double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
							   rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
							   rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
							   msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!
		update_pose(self, matrix, utcTimeInMiliSec, ns);
	}

	void quat2DCM(double (&rot_matrix)[9], geometry_msgs::Quaternion quat){
		matrix[0] = 1-2*(quat.y*quat.y+quat.z*quat.z)
		matrix[1] = 2*(quat.x*quat.y-quat.w*quat.z)
		matrix[2] = 2*(quat.x*quat.z+quat.w*quat.y)
		matrix[3] = 2*(quat.x*quat.y+quat.w*quat.z)
		matrix[4] = 1-2*(quat.x*quat.x+quat.z*quat.z)
		matrix[5] = 2*(quat.y*quat.z-quat.w*quat.x)
		matrix[6] = 2*(quat.x*quat.z-quat.w*quat.y)
		matrix[7] = 2*(quat.y*quat.z+quat.w*quat.x)
		matrix[8] = 1-2*(quat.x*quat.x+quat.y*quat.y)
	}

	void loop_handle()
	{
		counter_print++;
		
		for (int i=0; i<publishers.size; i++){
			switch (publishers[i]){
				case "/bg/geopose":
				//TODO poll SWM (we wait to have the function for the full pose)
				//TODO publish topic
				break;
			}
			
		}
		
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
}




void run() {
	ros::Rate loop_rate(rate);
	
	while (ros::ok())
	{
		ROS_INFO_ONCE("SIM: RUNNING");

		loop_handle();
		ros::spinOnce();

		loop_rate.sleep();
	}
}

protected:
/*state here*/
ros::NodeHandle n_;

ros::Subscriber subSelfGeopose_;

ros::Publisher pubBgGeopose_;

std::vector<string> publishers;

double utcTimeInMiliSec;

// swm
std::string ns;
char config_folder[255];
char config_name[];
char config_file;
json_t * config;
component_t *self;
bool agent_initialized;
//

int rate;

uint16_t counter_print;

private:

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swm_interface");
	ros::NodeHandle node;

	SwmInterfaceNodeClass swm_interfaceNode(node);

	swm_interfaceNode.run();
	return 0;
}
