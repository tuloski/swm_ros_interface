#include "ros/ros.h"
#include "mavros/ArtvaRead.h"
#include "geographic_msgs/GeoPose.h"
#include "mms_msgs/MMS_status.h"
#include "mms_msgs/Sys_status.h"
#include "camera_handler_sherpa/Camera.h"

using namespace std;


class swm_testbag {
	
	public:
		swm_testbag();
		void run();

	private:

		ros::NodeHandle _nh;
		ros::Publisher geo_pose_pub;
		ros::Publisher artva_pub;
		ros::Publisher status_pub;
		ros::Publisher bat_pub;
		ros::Publisher imgs_pub;
		ros::Publisher victims_pub;
		string ns;


		double initial_lat;
		double initial_lon;
		double initial_alt;

	
};
