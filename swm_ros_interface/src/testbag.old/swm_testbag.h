#include "ros/ros.h"
#include "mavros/ArtvaRead.h"
#include "geographic_msgs/GeoPose.h"
#include "mms_msgs/MMS_status.h"
#include "mms_msgs/Sys_status.h"

class swm_testbag {
	
	public:
		swm_testbag();
		void run();

	private:

		ros::NodeHandle _nh;
		ros::Publisher w0pose_pub, w1pose_pub, w2pose_pub;
		ros::Publisher fwpose_pub, roverpose_pub;										//agent poses
		ros::Publisher w0artva_pub, w1artva_pub, w2artva_pub;				//wasp artva pub
		ros::Publisher w0status_pub, w1status_pub, w2status_pub;		//wasp status 
		ros::Publisher w0bat_pub, w1bat_pub, w2bat_pub;							//wasp status 

	
};
