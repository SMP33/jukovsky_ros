#include <fstream>

#include "logger.h"

#include <jukovsky_ros/juk_dji_gps_msg.h>
#include <jukovsky_ros/juk_dji_device_status_msg.h>
#include <jukovsky_ros/juk_control_dji_msg.h>
#include <jukovsky_ros/juk_position_data_msg.h>
#include <jukovsky_ros/juk_set_target_data_msg.h>
#include <jukovsky_ros/reach_msg.h>
#include <jukovsky_ros/juk_aruco_module_data.h>



#define pr(x,y) <<  x <<": "<<y<<endl

#define ss <<",\n"<<

using namespace std;

bool is_exist(const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "JUK_LOGGER");
	ros::NodeHandle nh; 	
	
	std::string key;
	int host_port=20045;
	
	if (ros::param::search("port", key))
	{
	  ros::param::get(key, host_port);
	}
	
	ros::Rate r(5);
	ros::Time start_time = ros::Time::now();
	
	Log_msg<jukovsky_ros::juk_dji_gps_msg> gps(&nh, "JUK/DJI/GPS", start_time);
	Log_msg<jukovsky_ros::juk_dji_device_status_msg> device_status(&nh, "JUK/DJI/DEVICE_STATUS", start_time);
	
	Log_msg<jukovsky_ros::juk_control_dji_msg> control_dji(&nh, "JUK/CONTROL_DJI", start_time);
	Log_msg<jukovsky_ros::juk_position_data_msg> position_data(&nh, "JUK/POSITION_DATA", start_time);
	
	Log_msg<jukovsky_ros::juk_set_target_data_msg> target(&nh, "JUK/TARGET", start_time);
	
	Log_msg<jukovsky_ros::reach_msg> reach(&nh, "REACH_EMLID_DATA", start_time);
	Log_msg<jukovsky_ros::juk_aruco_module_data> aruco(&nh, "JUK/ARUCO/DATA", start_time);
	
	ofstream log_file;
	
	int i=0;
	
	while(is_exist("/home/pi//logs/log._"+to_string(i))) i++;
	
	log_file.open("/home/pi/logs/log._"+to_string(i));
	
	cout<<"Logger started on port "<< host_port << " "<< key <<endl;
	
	SimpleServer host(host_port);
	
	unsigned long id=0;
	
	while (ros::ok())
	{
		stringstream str;
		
		str.flags(std::ios::fixed);
		str.precision(10);
		
		str << "MESSAGE_BEGIN" << endl;
		
		str << "{" << endl;
		str << "\"HEADER\": {" << endl;
		str << "\t\"ID\": " << id << "," << endl;
		str << "\t\"TIME\": " << (ros::Time::now() - start_time) << endl;
		str << "}," << endl;
		str << "\"DATA\":{" << endl;
		
		str << gps.json() ss
			device_status.json() ss
			control_dji.json() ss
			position_data.json() ss
			target.json() ss
			aruco.json() ss
			reach.json() << endl;
		
		str << "}\n}" << endl;
		
		str << "MESSAGE_END" << endl<<endl;
		
		log_file<<str.str();
		host.set_response(str.str());
		
		ros::spinOnce();
		r.sleep();
		id++;
	}
	return 0;
}