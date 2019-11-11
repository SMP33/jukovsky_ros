#include <iostream>
#include <sstream>
#include <stdio.h>
#include <fstream>
#include <typeinfo>

#include <ros/ros.h>
#include <GeoMath.h>

#include <pthread.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <queue> 

#include <jukovsky_ros/juk_dji_gps_msg.h>
#include <jukovsky_ros/juk_dji_device_status_msg.h>
#include <jukovsky_ros/juk_control_dji_msg.h>
#include <jukovsky_ros/juk_set_target_data_msg.h>
#include <jukovsky_ros/juk_position_data_msg.h>
#include <jukovsky_ros/juk_dji_camera_control_msg.h>
#include <jukovsky_ros/juk_navigation_actions_msg.h>
#include <jukovsky_ros/reach_msg.h>

#include <sensor_msgs/LaserScan.h>

#include "server.h"


#define c(color,str)  "\x1B["<<color<<"m" << str << "\033[0m"
#define cyan_b(str)   "\x1B[46;1m" << str << "\033[0m" 
#define gray_b(str)   "\x1B[40m" << str << "\033[0m" 
#define red_b(str)   "\x1B[41;1m" << str << "\033[0m" 
#define green_b(str)   "\x1B[42;1m" << str << "\033[0m" 

#define grn(str)  "\x1B[32;1m" << str << "\x1B[39m" 
#define mgt(str)  "\x1B[35;1m" << str << "\x1B[39m" 
#define red(str)  "\x1B[31;1m" << str << "\x1B[39m" 

#define CLEAR(rows) printf("\033[%02dA\033[0J",rows+1)

using namespace std;
using namespace GeoMath;
using namespace jukovsky_ros;

bool DEBUG_MODE = false;

class TimeCounter
{
private:
	ros::Time start_time;
	
public:
	TimeCounter()
	{
		start_time = ros::Time::now();
	}
	
	ros::Duration time_passed()
	{
		return ros::Time::now() - start_time;
	}
	
	bool is_passed(unsigned long long int nsec)
	{
		return (time_passed().toNSec() > nsec);
	}
	
	void reset()
	{
		start_time=ros::Time::now();
	}
};

int deg_180_180(int deg)
{
	while (deg < -180)
		deg = deg + 360;
	
	while (deg > 180)
		deg = deg - 360;
	
	return deg;
}

int deg_0_360(int deg)
{
	while (deg < 0)
		deg = deg + 360;
	
	while (deg >= 360)
		deg = deg - 360;
	
	return deg;
}

vector<string> split(const string& str, const string& delim)
{
	vector<string> tokens;
	size_t prev = 0, pos = 0;
	do
	{
		pos = str.find(delim, prev);
		if (pos == string::npos) pos = str.length();
		string token = str.substr(prev, pos - prev);
		if (!token.empty()) tokens.push_back(token);
		prev = pos + delim.length();
	} while (pos < str.length() && prev < str.length());
	return tokens;
}

class JukFlyNode
{
public:
	enum TASK_TYPE
	{
		POINT,
		ACTION
	};
	
struct task
	{
		task()
		{
		}
		
		task(int action_, string block_description_)
			:type(ACTION)
			,action(action_)
			,block_description(block_description_)
		{
		}
		
		task(jukovsky_ros::juk_set_target_data_msg target_msg_ ,
		jukovsky_ros::juk_dji_camera_control_msg cam_msg_ ,
		int requaired_stable_time_ ,
		string block_description_ ,
		string point_description_ )
			:type(POINT)
			,target_msg(target_msg_)
			,cam_msg(cam_msg_)
			,requaired_stable_time(requaired_stable_time_)
			,block_description(block_description_)
			,point_description(point_description_)
			,action(0)
		{
		}
		
		TASK_TYPE type ;
		
		jukovsky_ros::juk_set_target_data_msg target_msg;
		jukovsky_ros::juk_dji_camera_control_msg cam_msg;
		int requaired_stable_time;
		string block_description;
		string point_description;
		
		
		int action=0;
	};
	
ros::NodeHandle nh;
	
	queue<task> points;
	
	ros::Subscriber pos_data_sub;
	ros::Subscriber lidar_sub;
	ros::Subscriber gps_a3_sub;
	
	jukovsky_ros::juk_position_data_msg pos_data;
	
	sensor_msgs::LaserScan scan_msg;
	
	task current_point;
	
	ros::Publisher target_pub;
	ros::Publisher camera_pub;
	ros::Publisher action_pub;
	
	ros::Time last_send_target_time;
	ros::Time last_photo_time;
	TimeCounter last_lidar_upd;
	TimeCounter no_obstacle_time;
	
	ros::Timer timer_telemetry;
	v3geo last_photo_pos;
	
	vector<int> scan_angles;
	
	int required_stable_time;
	int total_points = 0;
	int telem_heigth=0;
	
	float min_dist = 2;
	float dist = 999;
	
	SimpleServer host;
	quat q;
	
	void next_point()
	{
		auto point = points.front();
		points.pop();
		
		switch (point.type)
		{
		case POINT:
			
			current_point = point;
			target_pub.publish(current_point.target_msg);
			
			break;
		case ACTION:
			juk_navigation_actions_msg action_msg;
			current_point = point;
			current_point.requaired_stable_time = 3;
			action_msg.action = current_point.action;
			action_pub.publish(action_msg);
			break;
		}

		required_stable_time = current_point.requaired_stable_time;
		last_send_target_time = ros::Time::now();
		
	}
	
	void gps_a3_cb(const jukovsky_ros::juk_dji_gps_msg::ConstPtr& m)
	{
		q = { m->q0, m->q1, m->q2, m->q3 };
	}
	
	void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan_res)
	{
		scan_msg = *scan_res;
		
		auto& ranges = scan_msg.ranges;
		
		for (auto & r : ranges)
		{
			if (r > 999)
				r = 999;
		}
		
		dist = 999;
		
		stringstream out;
		
		out.flags(std::ios_base::fixed);
		out.precision(10);
		
		for (const auto& i : scan_angles)
		{
			if (ranges[i] < dist)
				dist = ranges[i];	
			
			if (ranges[i]<100)
			{
				v3 dir(-ranges[i]*sin(i*CONST.DEG2RAD), -ranges[i]*cos(i*CONST.DEG2RAD), 0);
				v3 rvec = q.to_eul();
				dir = dir.rotate({ rvec.z, rvec.y, 0 });	
				dir = dir + v3(20,0,-8);
				out << dir.x << " , " << dir.y << " , " << dir.z << " ; ";
				//out << rvec << endl;
			}
		}
		
		host.set_response(out.str());
		
		if (dist <= min_dist)
			no_obstacle_time.reset();
				
		
		

		
		last_lidar_upd.reset();
	}
	
	void pos_data_cb(const jukovsky_ros::juk_position_data_msg::ConstPtr& pos)
	{
		pos_data = *pos;
		
		auto now = ros::Time::now();
		
		if (!no_obstacle_time.is_passed(5e9) && !pos_data.paused)
		{
			juk_navigation_actions_msg action_msg;
			action_msg.action = juk_navigation_actions_msg::pause;
			action_pub.publish(action_msg);
		}
		else
		{
			if (no_obstacle_time.is_passed(5e9) && pos_data.paused)
			{
				juk_navigation_actions_msg action_msg;
				action_msg.action = juk_navigation_actions_msg::unpause;
				action_pub.publish(action_msg);
			}
			
			if (pos_data.stable_time > required_stable_time && (now - last_send_target_time).toNSec() > 5e9)
			{
				if (!points.empty())
				{
					cout << mgt("Next point:") << endl;
					next_point();
				}
				else
				{
					cout << mgt("Flight completed successfully!") << endl;
					exit(1);
				}
			
			}
		
			if ((last_photo_pos - v3geo(pos_data.lat, pos_data.lng, pos_data.alt)).length_xyz() >= 1)
			{
				current_point.cam_msg.action = jukovsky_ros::juk_dji_camera_control_msg::take_photo;
				camera_pub.publish(current_point.cam_msg);
				current_point.cam_msg.action = 0;
				last_photo_pos = v3geo(pos_data.lat, pos_data.lng, pos_data.alt);
				last_photo_time = now;
			}
			else
			{
				camera_pub.publish(current_point.cam_msg);
			}	
		}
	}
	

	void print_telemetry(const ros::TimerEvent& event)
	{
		CLEAR(telem_heigth + 1);
		
		stringstream out;
		out.flags(std::ios_base::fixed);
		out.precision(2);
		
		auto now = ros::Time::now();
		
		bool new_point = (now - last_send_target_time).toNSec() < 3e9;
		bool shoot_photo = (now - last_photo_time).toNSec() < 0.5*1e9;
//#define TEST_LIDAR
#ifndef TEST_LIDAR
				 
		out << "Point " << mgt(total_points - points.size()) << " from " << mgt(total_points) << endl;
		
		out << endl;
		
		out << "Description:" << endl;
		out << "\tBlock: " << grn(current_point.block_description) << endl;
		out << "\tPoint: " << current_point.point_description << endl;
		
		out << endl;
		
		out << "Lidar distance: " << dist<<endl;
		
		out << endl;
		
		switch (current_point.type)
		{
		case JukFlyNode::ACTION:
			out << endl;
			out << green_b("~~~~~~~~~~~~~\nSET HOMEPOINT\n~~~~~~~~~~~~~") << endl;
			break;
		case JukFlyNode::POINT:
			out << current_point.target_msg;
			out << "gimbal pitch: " << current_point.cam_msg.pitch / 10 << endl << endl;
			break;
		default:
			break;
		}
		
		out << "required stable time: " << required_stable_time - 1  << endl;
		
#endif // !TEST_LIDAR

	
		
		if (pos_data.paused)
		{
			out << endl;
			out << red_b("~~~~~~\nPAUSED\n~~~~~~") << endl;
		}
		
		
		if (new_point)
		{
			out << endl;
			out << cyan_b("~~~~~~~~~~\nNEXT POINT\n~~~~~~~~~~") << endl;
		}
		
		if (shoot_photo)
		{
			out << endl;
			out << gray_b("~~~~~~~~~~\nTAKE PHOTO\n~~~~~~~~~~") << endl;
		}
			

	
			
		std::string telem_txt = out.str();
		
		telem_heigth = 0;
		
		for (const auto& c : telem_txt)
		{
			if (c == '\n')
				telem_heigth++;
		}
		
		
		std::cout << out.str() << std::endl;
		
	}
	
	
	JukFlyNode(int argc, char *argv[])
		:scan_angles(0)
		,host(5000)
	{		
		
		int a1 = -20;
		int a2 = 0;
		
		cout << "Angle range: [ " << mgt(a1) << " ; " << mgt(a2) << " ]" << endl;
		
		while (deg_180_180(a1) != deg_180_180(a2+1))
		{
			scan_angles.push_back(deg_0_360(a1));
			a1++;
		}
		
		std::string key;
		std::string route_file_path;
		
		if (ros::param::search("route_file", key))
		{
		  ros::param::get(key, route_file_path);
		}
		else
		{
			cout << red("Too few arguments") << endl<<red("Please enter the path to the route file in .launch file")<<endl;
			cout << grn("Route file example:")<<endl;

			auto txt= "\t#fly mode; break mode; course mode; system; speed; data x; data y; data z; course; acc; yaw; pitch; roll; stable time\n"           
			"\n"
			"\t#взлет\n"
			"\t@SET_HOMEPOINT\n"
			"\t1;0;1;5;0.5;0;0;5;0;0.3;0;30;0;2;взлет\n"
			"\n"
			"\t#полет по точкам\n"
			"\t1;0;1;5;1;10;0;0;10;0.3;0;20;0;2;первая точка\n"
			"\t1;0;1;5;1;10;0;0;30;0.3;0;00;0;2;вторая точка\n" 
			"\t1;0;1;5;1;10;0;0;90;0.3;0;-60;0;2\n" 
			"\n"
			"\t#возвращение домой и посадка\n"
			"\t2;0;1;2;1;0;0;8;0;0.3;0;00;0;2;полет на безопасной высоте\n"
			"\t4;0;1;2;1;0;0;8;0;0.3;0;00;0;2;посадка по маркеру\n"
			"\t@SET_HOMEPOINT\n";
			cout<<txt<<endl;


			//return -1;
			exit(-1);
		}
	
		ifstream route_file(route_file_path);

		if (!route_file.is_open())
		{
			cout << red("File does not exist") << endl;
			exit(-1);
		}
	
		string line;	
		string last_block_description = " ";
		int line_count = 0;
		int required_params_count = 10 + 3 + 1;
		
		while (std::getline(route_file, line))
		{
			line_count++;
			size_t space_index = 0;
			while ((space_index = line.find(' ')) == 0)
				line.erase(space_index, 1);
			
			
			if (line[0] == '#')
			{
				line.erase(0, 1);
				last_block_description = line;
				continue;
			}
			
			if (line[0] == '@')
			{
				line.erase(0, 1);
				
				size_t space_index = 0;
				while ((space_index = line.find(' ')) != string::npos)
					line.erase(space_index, 1);
				
				if (line == "SET_HOMEPOINT")
				{
					points.push(task(1, last_block_description));
					continue;
				}
				
				continue;
			}
			
			if (line.length() < 2)
			{
				continue;
			}
			
			auto tokens = split(line, ";");
		
			
			
			if (tokens.size() < required_params_count)
			{
				cout << "\nLine " << mgt(line_count) << ":\n" << grn(line) <<endl<< red("Read file error") << endl;
				exit(-1);
			}
		
			jukovsky_ros::juk_set_target_data_msg msg_target;
			
			for (int i = 0; i < required_params_count; i++)
			{
				size_t space_index = 0;
				while ((space_index = tokens[i].find(' ')) != string::npos)
					tokens[i].erase(space_index, 1);
			}
		
			msg_target.fly_mode = stoi(tokens[0].c_str());		
			msg_target.break_distance_mode = stoi(tokens[1].c_str());		
			msg_target.course_mode = stoi(tokens[2].c_str());		
			msg_target.system = stoi(tokens[3].c_str());		
		
			msg_target.speed = stod(tokens[4].c_str());
		
			msg_target.data_x = stod(tokens[5].c_str());
			msg_target.data_y = stod(tokens[6].c_str());
			msg_target.data_z = stod(tokens[7].c_str());
			msg_target.course = stod(tokens[8].c_str());
			msg_target.acc = stod(tokens[9].c_str());
	
			jukovsky_ros::juk_dji_camera_control_msg msg_cam;
		
			msg_cam.yaw = 10*stod(tokens[10].c_str());
			msg_cam.pitch = 10*stod(tokens[11].c_str());
			msg_cam.roll = 10*stod(tokens[12].c_str());
			
			string point_description = " ";
			
			if (tokens.size() > required_params_count)
			{
				point_description = tokens[tokens.size() - 1] + " ";
				
				size_t space_index = 0;
				while ((space_index = point_description.find(' ')) == 0)
					point_description.erase(space_index, 1);				
			}
			
			int required_stable_time = 1 + stoi(tokens[13].c_str());
		
			points.push(task(msg_target, msg_cam, required_stable_time, last_block_description, point_description));
		}
		
		total_points = points.size();
		
		cout << "Total points: " << mgt(total_points) << endl<<endl;
		
		usleep(1000000);
		
//		for (int i = 0; i < 30; i++) 
//			cout << endl;
		
		pos_data_sub = nh.subscribe("JUK/POSITION_DATA", 1, &JukFlyNode::pos_data_cb, this);
		lidar_sub = nh.subscribe("scan", 1, &JukFlyNode::lidar_cb, this);
		gps_a3_sub = nh.subscribe("JUK/DJI/GPS", 1, &JukFlyNode::gps_a3_cb, this);
		
		target_pub = nh.advertise<jukovsky_ros::juk_set_target_data_msg>("JUK/TARGET", 1);
		camera_pub = nh.advertise<jukovsky_ros::juk_dji_camera_control_msg>("JUK/DJI_GIMBAL", 1);
		action_pub = nh.advertise<jukovsky_ros::juk_navigation_actions_msg>("JUK/NAVIGATION_ACTIONS", 1);
		
		timer_telemetry = nh.createTimer(ros::Duration(0.2), &JukFlyNode::print_telemetry, this);
		
		usleep(3000000);
		
		//cout << mgt("First point:") << endl;
		
		next_point();
		
		
	}

private:
	
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "JUK_FLY");
	
	JukFlyNode jf(argc,argv);
	
	ros::spin();
	return 0;
}