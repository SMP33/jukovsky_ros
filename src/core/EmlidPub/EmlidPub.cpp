#include <iostream>
#include "RegexEmlidReachParser.h"
#include <ros/ros.h>
#include <jukovsky_ros/reach_msg.h>

using namespace std;


int main(int argc, char *argv[])
{
	LLHparser            reach("/dev/custom/REACH", LLHparser::manual);
	ros::init(argc, argv, "Reach_Emlid_Node");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<jukovsky_ros::reach_msg>("REACH_EMLID_DATA", 1);
	
	ros::Rate r(20);
	
	jukovsky_ros::reach_msg msg;
	usleep(2000000);
	pub.publish(msg);
	
	while (ros::ok())
	{
		if (reach.manualUpd())
		{
			auto d = reach.getData();
			msg.lat = d.lat;
			msg.lng = d.lng;
			msg.alt = d.height;
			
			msg.quality = d.Q;
			msg.time_Y = d.YYYY;
			msg.time_M = d.MM;
			msg.time_D = d.DD;
			msg.time_h = d.hh;
			msg.time_m = d.mm;
			msg.time_s = d.ss;
			pub.publish(msg);
		}
		
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}