#include "swm_testbag.h"

#define default_namespace "wasp"

using namespace std;


swm_testbag::swm_testbag() {

	w0pose_pub = _nh.advertise< geographic_msgs::GeoPose > ("/wasp0/geopose", 0);
	w1pose_pub = _nh.advertise< geographic_msgs::GeoPose > ("/wasp1/geopose", 0);
	w2pose_pub = _nh.advertise< geographic_msgs::GeoPose > ("/wasp2/geopose", 0);
	fwpose_pub = _nh.advertise< geographic_msgs::GeoPose > ("/fw/geopose", 0);
	roverpose_pub = _nh.advertise< geographic_msgs::GeoPose > ("/donkey/geopose", 0);

	w0artva_pub = _nh.advertise<mavros::ArtvaRead>("/wasp0/artva_read", 0);
	w1artva_pub = _nh.advertise<mavros::ArtvaRead>("/wasp1/artva_read", 0);
	w2artva_pub = _nh.advertise<mavros::ArtvaRead>("/wasp2/artva_read", 0);

	w0status_pub = _nh.advertise< mms_msgs::MMS_status > ( "/wasp0/mms_status", 0);
	w1status_pub = _nh.advertise< mms_msgs::MMS_status > ( "/wasp1/mms_status", 0);
	w2status_pub = _nh.advertise< mms_msgs::MMS_status > ( "/wasp2/mms_status", 0);

	w0bat_pub = _nh.advertise < mms_msgs::Sys_status > ("/wasp0/system_status", 0);
	w1bat_pub = _nh.advertise < mms_msgs::Sys_status > ("/wasp1/system_status", 0);
	w2bat_pub = _nh.advertise < mms_msgs::Sys_status > ("/wasp2/system_status", 0);

	//add images!

}


void swm_testbag::run() {


	geographic_msgs::GeoPose w0p, w1p, w2p;
	geographic_msgs::GeoPose fwp, rp;

	w0p.orientation.w = 1;
	w0p.orientation.x = 0;
	w0p.orientation.y = 0;
	w0p.orientation.z = 0;

	rp = fwp = w1p = w2p = w0p;
	
	mavros::ArtvaRead w0a, w1a, w2a;
		
	w0a.rec2_distance = 	w0a.rec3_distance = 	w0a.rec4_distance = -1;
	w0a.rec2_direction = 	w0a.rec3_direction = 	w0a.rec4_direction = 0;
	w1a = w2a = w0a;

	mms_msgs::Sys_status w0s, w1s, w2s;

	ros::Rate r(10);

	double t = 0.0;
	double step = 1/10.0;
	double AMP = 0.01;
	double TAU = 1/5.0;

	
	while( ros::ok() ) {
	

		double sinu = AMP*sin( TAU*t );

		w0p.position.latitude = 44.492423 + sinu;
		w0p.position.longitude = 11.330068 + sinu;
		w0p.position.altitude = 20.0;
		
		w1p.position.latitude = 44.46423 + sinu;
		w1p.position.longitude = 11.310068 + sinu;
		w1p.position.altitude = 25.0;

		w2p.position.latitude = 44.452423 + sinu;
		w2p.position.longitude = 11.300068 + sinu;
		w2p.position.altitude = 30.0;


		fwp.position.latitude = 44.442423 + sinu;
		fwp.position.longitude = 11.300068 + sinu;
		fwp.position.altitude = 90.0;

		rp.position.latitude = 44.442423 + sinu;
		rp.position.longitude = 11.320068 + sinu;
		rp.position.altitude = 0.0;

		t += step;

		w0pose_pub.publish( w0p );
		w1pose_pub.publish( w1p );
		w2pose_pub.publish( w2p );

		fwpose_pub.publish( fwp );
		roverpose_pub.publish( rp );

		w0a.rec1_distance = rand() % 4000;
		w0a.rec1_direction = rand() % 90;
		w1a.rec1_distance = rand() % 4000;
		w1a.rec1_direction = rand() % 90;
		w2a.rec1_distance = rand() % 4000;
		w2a.rec1_direction = rand() % 90;

		w0artva_pub.publish( w0a );
		w1artva_pub.publish( w1a );
		w2artva_pub.publish( w2a );

		w0s.voltage_battery = rand() % 2000 + 1400;
		w1s.voltage_battery = rand() % 2000 + 1400;
		w2s.voltage_battery = rand() % 2000 + 1400;

		w0bat_pub.publish( w0s );
		w1bat_pub.publish( w1s );
		w2bat_pub.publish( w2s );

		r.sleep();
	}
}





int main( int argc, char** argv ) {
	ros::init( argc, argv, "swm_testbag");

	swm_testbag tbag;
	tbag.run();

	return 0;
}


