/****************************************************************************
 *
 * swm_ros_interface
 *
 * This software was developed at:
 * PRISMA Lab.
 * Napoli, Italy
 *
 * Description: 
 *
 * Authors:
 * Michele Furci   <michele.furci@unibo.it>
 * Jonathan Cacace <jonathan.cacace@gmail.com>
 *
 * Created in 16/12/2016.
 *
 *
 * Copyright (C) 2016 PRISMA Lab. All rights reserved.
 *****************************************************************************/



//ROS
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geographic_msgs/GeoPose.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <zyre.h>
#include <jansson.h>
#include <uuid/uuid.h>
#include <string.h>
#include <sys/time.h>
#include <ros/package.h>
#include <sbox_msgs/Sbox_msg_status.h>
//UNIBO
#include <mms_msgs/Sys_status.h>  // Added by NIcola
#include <mavros/ArtvaRead.h> // Added by NIcola
#include "camera_handler_sherpa/Camera.h"

extern "C" {
	#include "swmzyre.h"
}

//C++
#include <vector>
#include <string>
#include <math.h>
#include "boost/thread.hpp"


using namespace std;

enum publishers_code {
	BG_GEOPOSE,
	WASP_GEOPOSE
};

class SwmRosInterfaceNodeClass {
	public:
		SwmRosInterfaceNodeClass();
		void run();
		void main_loop();

		//---Callbacks		
		void readGeopose_publishSwm_wasp(const geographic_msgs::GeoPose::ConstPtr& msg);
		void readCameraObservations_publishSwm(const camera_handler_sherpa::Camera::ConstPtr& msg);
		void readBattery_publishSwm_wasp(const mms_msgs::Sys_status::ConstPtr& msg);
		void readArtva_publishSwm_wasp(const mavros::ArtvaRead::ConstPtr& msg);
		void readSboxStauts_publishSwm_wasp( sbox_msgs::Sbox_msg_status msg );
		void readVictims_publishSwm(const geographic_msgs::GeoPose::ConstPtr& msg);
		//---

	protected:
		
		/*state here*/
		ros::NodeHandle _nh;
		ros::Subscriber subSelfGeopose_;
		ros::Subscriber subWaspGeopose_;
		ros::Subscriber subWaspCamera_;
		ros::Subscriber subWaspBattery_;
		ros::Subscriber subWaspArtva_;
		ros::Subscriber subSboxStatus_;
		ros::Subscriber subVictims_;
		ros::Publisher pubBgGeopose_;
		std::vector<publishers_code> publishers;
		std::vector<uint16_t> rate_publishers;
		std::vector<uint16_t> counter_publishers;

		double utcTimeInMiliSec;
		std::string ns;

		struct timeval tp;

		// swm
		char config_folder[255];
		char config_name[];
		char config_file;
		json_t * config;
		component_t *self;
		bool agent_initialized;
		//	

		geographic_msgs::GeoPose last_agent_pose;

		int rate;
		uint16_t counter_print;

private:

};


//---helper functions
void quat2DCM(double (&rot_matrix)[9], geometry_msgs::Quaternion quat);
std::string load_param_string( string def, string name );
bool load_param_bool( string def, string name );
//---
