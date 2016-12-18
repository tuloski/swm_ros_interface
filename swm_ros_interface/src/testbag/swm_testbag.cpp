#include "swm_testbag.h"

#define PUB_POSE 						1
#define PUB_ARTVA						1
#define PUB_STATUS					1
#define PUB_BAT							1
#define default_namespace "wasp"

using namespace std;


void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param ("~");
	if (!n_param.getParam (name, p)) 
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

swm_testbag::swm_testbag() {

	ns = ros::this_node::getNamespace();
	string nodename = ros::this_node::getName();	
	if( ns == "/" ) 
		ns = default_namespace;
	else {
		//pezzotto, remove the first 2 //
		ns = ns.substr( 1, ns.size()-1 );
	}

	geo_pose_pub = _nh.advertise< geographic_msgs::GeoPose > ("/" + ns + "/geopose", 0);	
	artva_pub = _nh.advertise<mavros::ArtvaRead>("/" + ns + "/artva_read", 0);
	status_pub = _nh.advertise< mms_msgs::MMS_status > ( "/" + ns + "/mms_status", 0);
	bat_pub = _nh.advertise < mms_msgs::Sys_status > ("/" + ns + "/system_status", 0);
	imgs_pub = _nh.advertise< camera_handler_sherpa::Camera > ("/" + ns + "/camera_published", 0);
	victims_pub = _nh.advertise < geographic_msgs::GeoPose > ("/" + ns + "/victims", 0);
	

	load_param(initial_lat, 44.492423, "initial_lat" );
	load_param(initial_lat, 11.300068, "initial_lon" );
	load_param(initial_lat, 150, "initial_alt" );

}


void swm_testbag::run() {

	geographic_msgs::GeoPose pose;
	pose.orientation.w = 1;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;

	mavros::ArtvaRead artva;
	artva.rec2_distance = 	artva.rec3_distance = 	artva.rec4_distance = -1;
	artva.rec2_direction = 	artva.rec3_direction = 	artva.rec4_direction = 0;

	mms_msgs::Sys_status status;

	ros::Rate r(10);

	double t = 0.0;
	double step = 1/10.0;
	double AMP = 0.01;
	double TAU = 1/5.0;

	vector < camera_handler_sherpa::Camera > camera_msgs;
	camera_handler_sherpa::Camera camera_m;
	camera_m.geopose.orientation.w = 1;
	camera_m.geopose.orientation.x = 0;
	camera_m.geopose.orientation.y = 0;
	camera_m.geopose.orientation.z = 0;
	camera_m.path_photo = ns + "/home/odroid/pictures/img001.png";
	camera_m.geopose.position.latitude = 44.492423;
	camera_m.geopose.position.longitude = 11.300068;
	camera_m.geopose.position.altitude = 100;
	camera_msgs.push_back( camera_m );

	camera_m.path_photo = ns + "/home/odroid/pictures/img002.png";
	camera_m.geopose.position.latitude = 44.492423;
	camera_m.geopose.position.longitude = 11.320068;
	camera_m.geopose.position.altitude = 100;
	camera_msgs.push_back( camera_m );

	camera_m.path_photo = ns + "/home/odroid/pictures/img003.png";
	camera_m.geopose.position.latitude = 44.492423;
	camera_m.geopose.position.longitude = 11.310068;
	camera_m.geopose.position.altitude = 100;
	camera_msgs.push_back( camera_m );

	camera_m.path_photo = ns + "/home/odroid/pictures/img004.png";
	camera_m.geopose.position.latitude = 44.492423;
	camera_m.geopose.position.longitude = 11.380068;
	camera_m.geopose.position.altitude = 100;
	camera_msgs.push_back( camera_m );

	vector< geographic_msgs::GeoPose > victims;
	geographic_msgs::GeoPose v_pose;
	v_pose.orientation.w = 1;
	v_pose.orientation.x = v_pose.orientation.y = v_pose.orientation.z = 0;
	v_pose.position.latitude = 44.462423;
	v_pose.position.longitude = 11.33009;
	victims.push_back( v_pose );
	v_pose.position.latitude = 44.442423;
	v_pose.position.longitude = 11.31009;
	victims.push_back( v_pose );
	v_pose.position.latitude = 44.412423;
	v_pose.position.longitude = 11.29009;
	victims.push_back( v_pose );

	bool observation_published = false;
	status.voltage_battery = 1400;

	while( ros::ok() ) {
	
		double sinu = AMP*sin( TAU*t );

		pose.position.latitude = initial_lat + sinu;
		pose.position.longitude = initial_lon + sinu;
		pose.position.altitude = initial_alt;
		geo_pose_pub.publish( pose );
	
		artva.rec1_distance = rand() % 4000;
		artva.rec1_direction = rand() % 90;
		
		artva_pub.publish( artva );
	
		status.voltage_battery -= 0.001;
		
		bat_pub.publish( status );
		
		t += step;
	
		//after 5 seconds publish victims and pictures	
		if ( !observation_published && t > 5) {
			cout << "Publish observation" << endl;
			observation_published = true;
			for(int i=0; i<victims.size(); i++ ) {
				victims_pub.publish( victims[i] );
				sleep(1);
			}

			for(int i=0; i<camera_msgs.size(); i++ ) {
				imgs_pub.publish( camera_msgs[i] );
				sleep(1);	
			}
			
		}


		r.sleep();
	}
}





int main( int argc, char** argv ) {
	ros::init( argc, argv, "swm_testbag");

	swm_testbag tbag;
	tbag.run();

	return 0;
}


