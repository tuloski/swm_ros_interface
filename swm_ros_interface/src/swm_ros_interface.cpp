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
	counter_status = 0;
	counter_geopose = 0;
	counter_battery = 0;
	
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
	pub_geopose = load_param_bool( false, nodename + "/pub/geopose" );
	pub_system_status = load_param_bool(false, nodename + "/pub/system_status" );
	pub_mms_status = load_param_bool(false, nodename + "/pub/mms_status" );  // Added by NIcola
	pub_wasp_images = load_param_bool(false, nodename + "/pub/wasp_images" );
	pub_artva = load_param_bool(false, nodename + "/pub/wasp_artva");
	sub_geopose_bg = load_param_bool(false, nodename + "/sub/geopose_bg");
	pub_victims = load_param_bool(false, nodename + "/pub/victims");

	string swm_zyre_conf = load_param_string( "swm_zyre_config.json", nodename + "/swm_zyre_conf_file");
	//---

	//---publishers (TO SWM)  // Added by NIcola
	if (pub_mms_status) { //send the finite state machine status to the SWM 		  	
		subWaspMmsStatus_ = _nh.subscribe("/" + ns + "/mms_status", 0, &SwmRosInterfaceNodeClass::readMmsStatus_publishSwm_wasp,this);
		cout << "Subscribing: \t [" + ns + "/mms_status]" << endl; 
	} //MMS

	if (pub_system_status) { //send the position of the bg to the SWM 
	  	subWaspBattery_ = _nh.subscribe("/" + ns + "/system_status", 0, &SwmRosInterfaceNodeClass::readBattery_publishSwm_wasp,this);
		cout << "Subscribing: \t [" + ns + "/system_status]" << endl; 
	} //battery

	if (pub_geopose) { //send the position of the wasp to the SWM 
		subWaspGeopose_ = _nh.subscribe("/" + ns + "/geopose", 0, &SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp,this);
		cout << "Subscribing: \t [" << ns + "/geopose]" << endl; 
	} //geopose

	if (pub_wasp_images) { //send the images data to the SWM 
		subWaspCamera_ = _nh.subscribe("/" + ns + "/camera_published", 0, &SwmRosInterfaceNodeClass::readCameraObservations_publishSwm,this);
		cout << "Subscribing: \t [" << ns + "/camera]" << endl; 
	} //images

	if( pub_victims ) {
		subVictims_ = _nh.subscribe("/" + ns + "/victims", 0, &SwmRosInterfaceNodeClass::readVictims_publishSwm, this );
	} //victims

	if( pub_artva ) {
		subWaspArtva_ = _nh.subscribe("/" + ns + "/artva_read", 0, &SwmRosInterfaceNodeClass::readArtva_publishSwm_wasp,this);
		cout << "Subscribing: \t [" << ns + "artva_read" << "]" << endl;
	} // Artva
	//---

	//---read from SWM busy genius geopose
	if ( sub_geopose_bg ) {
		pubBgGeopose_ = _nh.advertise<geographic_msgs::GeoPose>("/bg/geopose", 0);
		publishers.push_back(BG_GEOPOSE);
		rate_publishers.push_back(5);	//rate in Hz at which we want to read from SWM
		counter_publishers.push_back(0);
	} 
	//---

	rate = 50;	//TODO maybe pick rate of node as twice the highest rate of publishers
 
	std::string conf_path = ros::package::getPath( "swm_ros_interface" );  
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
	//assert(add_agent(self, matrix, 0.0, str2char(ns) ));
	add_agent(self, matrix, 0.0, str2char(ns));
}

 // Added by NIcola
void SwmRosInterfaceNodeClass::readMmsStatus_publishSwm_wasp(const mms_msgs::MMS_status::ConstPtr& msg){
	// gettimeofday(&tp, NULL);
	// utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	// STATES DEFINITION
	_status = *msg;
	updated_status = true;

	/*wasp_flight_status status;
	if (msg->mms_state < 50)
			status.flight_state = str2char("ON_GROUND_PROP_OFF");
	else if (msg->mms_state == 50)
			status.flight_state = str2char("ON_GROUND_PROP_ON");
	else
			status.flight_state = str2char("IN_FLIGHT");
	ROS_WARN("MMS_STATUS %s", status.flight_state);
		//status.flight_state = msg->mms_state;
	assert(add_wasp_flight_status(self, status, str2char(ns)));
	//	add_wasp_flight_status(self, status, str2char(ns));*/
}


 // Added by NIcola
void SwmRosInterfaceNodeClass::readArtva_publishSwm_wasp(const mavros::ArtvaRead::ConstPtr& msg ) {
	_artva = *msg;
	if ((_artva.rec1_distance < ARTVA_H_THRESHOLD && _artva.rec1_distance > 0) ||
		(_artva.rec2_distance < ARTVA_H_THRESHOLD && _artva.rec2_distance > 0) ||
		(_artva.rec3_distance < ARTVA_H_THRESHOLD && _artva.rec3_distance > 0) ||
		(_artva.rec4_distance < ARTVA_H_THRESHOLD && _artva.rec4_distance > 0)){
		updated_artva = true;
	}
	/*gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

	artva_measurement artva;
	artva.signal0 = msg->rec1_distance;
	artva.signal1 = msg->rec2_distance;
	artva.signal2 = msg->rec3_distance;
	artva.signal3 = msg->rec4_distance;
	artva.angle0 = msg->rec1_direction;
	artva.angle1 = msg->rec2_direction;
	artva.angle2 = msg->rec3_direction;
	artva.angle3 = msg->rec4_direction;
	add_artva_measurement(self, artva, str2char(ns));*/

	/*  add_artva_measurement(self, msg->rec1_distance, msg->rec1_direction, msg->rec2_distance, msg->rec2_direction, msg->rec3_distance, msg->rec3_direction, msg->rec4_distance, msg->rec4_direction, utcTimeInMiliSec, str2char(ns));*/
}

 // Added by NIcola
void SwmRosInterfaceNodeClass::readBattery_publishSwm_wasp(const mms_msgs::Sys_status::ConstPtr& msg){
	counter_battery++;
	if (counter_battery>=10){
		updated_battery = true;
		counter_battery = 0;
	}
	_sys_status = *msg;
	//gettimeofday(&tp, NULL);
	//utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	//string battery_status = "HIGH";
  //add_battery(self, msg->voltage_battery, str2char(battery_status),  utcTimeInMiliSec, str2char(ns));
}


void SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp(const geographic_msgs::GeoPose::ConstPtr& msg){
	_geopose = *msg;
	counter_geopose++;
	if (counter_geopose >=10){
		updated_geopose = true;
		counter_geopose = 0;
	}


	/*gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

  last_agent_pose = *msg;

	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!
	
	update_pose(self, matrix, utcTimeInMiliSec, str2char(ns) );*/
}

void SwmRosInterfaceNodeClass::readCameraObservations_publishSwm(const camera_handler_sherpa::Camera::ConstPtr& msg){
	updated_camera = true;
	_camera = *msg;
	/*gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->geopose.orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->geopose.position.latitude, msg->geopose.position.longitude, msg->geopose.position.altitude, 1}; // y,x,z,1 remember this is column-major!
	add_image(self, matrix, utcTimeInMiliSec, str2char(ns), str2char(msg->path_photo));*/
}


void SwmRosInterfaceNodeClass::readVictims_publishSwm(const geographic_msgs::GeoPose::ConstPtr& msg) {
		updated_victims = true;
		_victims = *msg;
    /*gettimeofday(&tp, NULL);
    utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

  last_agent_pose = *msg;

    double rot_matrix[9];
    quat2DCM(rot_matrix, msg->orientation);
    double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
                                               rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
                                               rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
                                               msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!

  add_victim(self, matrix, utcTimeInMiliSec, str2char( ns ));*/
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



void DCM2quat(float Rot[3][3], geometry_msgs::Quaternion & quat){
	
	float Quat[4];
	double tr = Rot[0][0]+ Rot[1][1]+ Rot[2][2];
	int ii;
	ii=0;
	if (Rot[1][1] > Rot[0][0]) ii=1;
	if (Rot[2][2] > Rot[ii][ii]) ii=2;
	double s;
	if (tr >= 0){
		s = sqrt((tr + 1));
		Quat[0] = s * 0.5;
		s = 0.5 / s;
		Quat[1] = (Rot[2][1] - Rot[1][2]) * s;
		Quat[2] = (Rot[0][2] - Rot[2][0]) * s;
		Quat[3] = (Rot[1][0] - Rot[0][1]) * s;
	}
	else {
		switch(ii) {
			case 0:
				s = sqrt((Rot[0][0]-Rot[1][1]-Rot[2][2]+1));
				Quat[1] = s * 0.5;
				s = 0.5 / s;
				Quat[2] = (Rot[1][0] + Rot[0][1]) * s;//Update pose estimation
				Quat[3] = (Rot[2][0] + Rot[0][2]) * s;
				Quat[0] = (Rot[2][1] - Rot[1][2]) * s;
			break;
			case 1:
				s = sqrt((Rot[1][1]-Rot[2][2]-Rot[0][0]+1));
				Quat[2] = s * 0.5;
				s = 0.5 / s;
				Quat[3] = (Rot[2][1] + Rot[1][2]) * s;
				Quat[1] = (Rot[0][1] + Rot[1][0]) * s;
				Quat[0] = (Rot[0][2] - Rot[2][0]) * s;
			break;
				case 2:
				s = sqrt((Rot[2][2]-Rot[0][0]-Rot[1][1]+1));
				Quat[3] = s * 0.5;
				s = 0.5 / s;
				Quat[1] = (Rot[0][2] + Rot[2][0]) * s;
				Quat[2] = (Rot[1][2] + Rot[2][1]) * s;
				Quat[0] = (Rot[1][0] - Rot[0][1]) * s;
			break;
		}
	}
	quat.w = Quat[0];
	quat.x = Quat[1];
	quat.y = Quat[2];
	quat.z = Quat[3];

}







	//TODO
	/*den = np.array([ 1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2],
	                       1.0 - matrix[0][0] + matrix[1][1] - matrix[2][2],
	                       1.0 - matrix[0][0] - matrix[1][1] + matrix[2][2],
	                       1.0 + matrix[0][0] + matrix[1][1] + matrix[2][2]])
	    max_idx = np.flatnonzero(den == max(den))[0]
	    q = np.zeros(4)
	    q[max_idx] = 0.5 * math.sqrt(max(den))
	    denom = 4.0 * q[max_idx]
	    if (max_idx == 0):
	        q[1] =  (matrix[1][0] + matrix[0][1]) / denom
	        q[2] =  (matrix[2][0] + matrix[0][2]) / denom
	        q[3] = -(matrix[2][1] - matrix[1][2]) / denom
	    if (max_idx == 1):
	        q[0] =  (matrix[1][0] + matrix[0][1]) / denom
	        q[2] =  (matrix[2][1] + matrix[1][2]) / denom
	        q[3] = -(matrix[0][2] - matrix[2][0]) / denom
	    if (max_idx == 2):
	        q[0] =  (matrix[2][0] + matrix[0][2]) / denom
	        q[1] =  (matrix[2][1] + matrix[1][2]) / denom
	        q[3] = -(matrix[1][0] - matrix[0][1]) / denom
	    if (max_idx == 3):
	        q[0] = -(matrix[2][1] - matrix[1][2]) / denom
	        q[1] = -(matrix[0][2] - matrix[2][0]) / denom
	        q[2] = -(matrix[1][0] - matrix[0][1]) / denom
	    q_out = Quaternion(q[0],q[1],q[2],q[3])
	    return q_out*/


void SwmRosInterfaceNodeClass::main_loop()
{

	ros::Rate r(rate);

	//---msgs
	geographic_msgs::GeoPose gp;
	//---

        utcTimeInMiliSec = 1.0;

	while( ros::ok() ) {

		//ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
		//utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
		gettimeofday(&tp, NULL);
		utcTimeInMiliSec = tp.tv_sec * 1000ULL + tp.tv_usec / 1000ULL; //get current timestamp in milliseconds
		//utcTimeInMiliSec = 1000.0;
		if( utcTimeInMiliSec < 0.0 ) {
			ROS_ERROR("TIME LESS THE Zero!!");
			r.sleep();
			continue;
		}
		
		if (pub_system_status){
			counter_status++;
		}

		if (updated_battery){
			updated_battery = false;
			string battery_status = "HIGH";		//TODO
			ROS_INFO("SWM_ROS_INTERFACE: Published battery");
			//add_battery(self, _sys_status.voltage_battery, str2char(battery_status),  utcTimeInMiliSec, str2char(ns));
		}
		if (updated_geopose){
			updated_geopose = false;
			//last_agent_pose = *msg;
			double rot_matrix[9];
			quat2DCM(rot_matrix, _geopose.orientation);
			double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
									 					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
									 					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
									 					_geopose.position.latitude, _geopose.position.longitude, _geopose.position.altitude, 1}; // y,x,z,1 remember this is column-major!
			update_pose(self, matrix, utcTimeInMiliSec, str2char(ns) );
			ROS_INFO("SWM_ROS_INTERFACE: Published geopose");
		}
		if (updated_camera){
			updated_camera = false;
			double rot_matrix[9];
			quat2DCM(rot_matrix, _camera.geopose.orientation);
			double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
									 					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
									 					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
									 					_camera.geopose.position.latitude, _camera.geopose.position.longitude, _camera.geopose.position.altitude, 1}; // y,x,z,1 remember this is column-major!
			add_image(self, matrix, utcTimeInMiliSec, str2char(ns), str2char(_camera.path_photo));
			ROS_INFO("SWM_ROS_INTERFACE: Published photo");
		}
		if (updated_victims){
			updated_victims = false;
			double rot_matrix[9];
			quat2DCM(rot_matrix, _victims.orientation);
			double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
		                                             rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
		                                             rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
		                                             _victims.position.latitude, _victims.position.longitude, _victims.position.altitude, 1}; // y,x,z,1 remember this is column-major!
			add_victim(self, matrix, utcTimeInMiliSec, str2char( ns ));
			ROS_INFO("SWM_ROS_INTERFACE: Published victims");
		}
		if (updated_artva){
			updated_artva = false;
			artva_measurement artva;
			artva.signal0 = _artva.rec1_distance < ARTVA_H_THRESHOLD ? _artva.rec1_distance : 0 ;
			artva.signal1 = _artva.rec2_distance < ARTVA_H_THRESHOLD ? _artva.rec2_distance : 0 ;
			artva.signal2 = _artva.rec3_distance < ARTVA_H_THRESHOLD ? _artva.rec3_distance : 0 ;
			artva.signal3 = _artva.rec4_distance < ARTVA_H_THRESHOLD ? _artva.rec4_distance : 0 ;
			artva.angle0 = _artva.rec1_direction;
			artva.angle1 = _artva.rec2_direction;
			artva.angle2 = _artva.rec3_direction;
			artva.angle3 = _artva.rec4_direction;
			add_artva_measurement(self, artva, str2char(ns));
			ROS_INFO("SWM_ROS_INTERFACE: Published artva");
		}
		if (counter_status >= rate/1){	//publishing at 1 Hz
			counter_status = 0;
			wasp_flight_status status;
			if (_status.mms_state < mms_msgs::MMS_status::ON_GROUND_ARMED)
					status.flight_state = str2char("ON_GROUND_PROP_OFF");
			else if (_status.mms_state == mms_msgs::MMS_status::ON_GROUND_ARMED)
					status.flight_state = str2char("ON_GROUND_PROP_ON");
			else
					status.flight_state = str2char("IN_FLIGHT");
			//ROS_WARN("MMS_STATUS %s", status.flight_state);
			//status.flight_state = msg->mms_state;
			add_wasp_flight_status(self, status, str2char(ns));
			//	add_wasp_flight_status(self, status, str2char(ns));
			ROS_INFO("SWM_ROS_INTERFACE: Published status");
		}
		

		counter_print++;
		for (int i=0; i<publishers.size(); i++){
			
			counter_publishers[i]++;

			switch (publishers[i]){
				case BG_GEOPOSE:
					if (counter_publishers[i] >= rate/rate_publishers[i]){
						string agent_name = "op0";
						geometry_msgs::Quaternion quat;
						quat.x = 0;
						quat.y = 0;
						quat.z = 0;
						quat.w = 1;
						geographic_msgs::GeoPoint geopoint;
						gettimeofday(&tp, NULL);
						utcTimeInMiliSec = tp.tv_sec * 1000ULL + tp.tv_usec / 1000ULL; //get current timestamp in milliseconds
						get_pose(self, transform_matrix_bg, utcTimeInMiliSec, str2char(agent_name));
						double rot_matrix[9] = { transform_matrix_bg[0], transform_matrix_bg[1], transform_matrix_bg[2],
												 transform_matrix_bg[4], transform_matrix_bg[5], transform_matrix_bg[6],
												 transform_matrix_bg[8], transform_matrix_bg[9], transform_matrix_bg[10]}; // y,x,z,1 remember this is column-major!

						float M[3][3];
						M[0][0] = rot_matrix[0];
						M[0][1] = rot_matrix[1];
						M[0][2] = rot_matrix[2];

						M[1][0] = rot_matrix[3];
						M[1][1] = rot_matrix[4];
						M[1][2] = rot_matrix[5];

						M[2][0] = rot_matrix[6];
						M[2][1] = rot_matrix[7];
						M[2][2] = rot_matrix[8];

						DCM2quat(M, quat);
	

						geopoint.latitude = transform_matrix_bg[12];
						geopoint.longitude = transform_matrix_bg[13];
						geopoint.altitude = transform_matrix_bg[14];
						gp.orientation = quat;
						gp.position = geopoint;
						//if ((gp.position.latitude != _old_geopose_bg.position.latitude) || (gp.position.longitude != _old_geopose_bg.position.longitude) || (gp.position.altitude != _old_geopose_bg.position.altitude)){
							//Publish only if geopose is different, TODO use also attitude
							//_old_geopose_bg = gp; 
							pubBgGeopose_.publish(gp);
						//}
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
