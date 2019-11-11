#pragma once
#ifndef __NAVIGATION_NODE__
#define __NAVIGATION_NODE__
				 
#include <ros/ros.h>

#include <tuple>
#include <functional>
#include <map>

#include <GeoMath.h>
#include "jukovsky_ros/juk_dji_gps_msg.h"
#include "jukovsky_ros/juk_dji_device_status_msg.h"
#include "jukovsky_ros/juk_control_dji_msg.h"
#include "jukovsky_ros/juk_set_target_data_msg.h"
#include "jukovsky_ros/juk_position_data_msg.h"
#include "jukovsky_ros/juk_navigation_actions_msg.h"
#include <jukovsky_ros/juk_aruco_module_action.h>
#include <jukovsky_ros/juk_aruco_module_data.h>
#include <jukovsky_ros/reach_msg.h>

#include "std_msgs/String.h"

#include "ArgParser.h"


#define c(color,str)  "\x1B["<<color<<"m" << str << "\033[0m"
#define cyan_b(str)   "\x1B[46;1m" << str << "\033[0m" 
#define gray_b(str)   "\x1B[40m" << str << "\033[0m" 
#define red_b(str)   "\x1B[41;1m" << str << "\033[0m" 
#define green_b(str)   "\x1B[42;1m" << str << "\033[0m" 


#define grn(str)  "\x1B[32;1m" << str << "\x1B[39m" 
#define mgt(str)  "\x1B[35;1m" << str << "\x1B[39m" 
#define red(str)  "\x1B[31;1m" << str << "\x1B[39m" 

#define CLEAR(rows)               printf("\033[%02dA\033[0J",rows+1)




class NavigationNode
{
public:
	
	enum STATES ///< �������� ���������
	{
		IDLE = 0,
		///< ������� ���������� �� �������, � ������� ��� ��������� ��� �������� ����������
	  FLY_SIMPLE = 1,
		///< ������� ����� � �������� ����� �� ������
	FLY_SAFE = 2,
		///< ������� ����� � �������� ����� �� ���������� ������ (�������� � ���������� �������)
	LANDING_SIMPLE = 3,
		///< ������� ����� � ��������� ����� �� ������, � ����� ��������� � ��� ������������ �������
	  LANDING_ARUCO = 4,
		///< ������� ����� � ��������� ����� �� ������, � ����� ��������� � ��� �������, ������������ �� ArUco �������

	};
	enum SUB_STATES ///< ������������
	{  
		BASIC_STATE = 0, 
		///< ������������ �� ���������
		
	  FLY_SAFE_UP = 201,
		///< ������������ FLY_SAFE - ������ �� ���������� ������
	  FLY_SAFE_CENTER = 202,
		///< ������������ FLY_SAFE - ����� � ����� �� ������, ���� ������� ��������� ��� ������ � �������� �������� 2�
		
	  LANDING_SIMPLE_FLY = 301,
		///< ������������ LANDING_SIMPLE - ����� � ����� �� ������
	  LANDING_SIMPLE_LAND = 302,
		///< ������������ LANDING_SIMPLE - ������� � ��������� �����
			
	  LANDING_ARUCO_FLY = 401, 
		///< ������������ LANDING_ARUCO - ����� � ����� �� ������
	   LANDING_ARUCO_LAND = 402,
		///< ������������ LANDING_ARUCO - ����� �� ��������� ����� � ������� 
		 LANDING_ARUCO_HOLD_CENTER = 403,
		 LANDING_ARUCO_RUSH = 404
	};
	
	enum GPS_STATES
	{
		NO_SIGNAL = 0,
		BASIC_GPS = 1,
		RTK = 2
	};
	
enum ACTIONS
	{
		SET_HOMEPOINT = 1  ///< ��������� �������� �����
	};
	
	NavigationNode(int argc, char** argv);

private:
	
	ArgParser_Int params;    ///< ������ ���������� �������
	const int max_precision_uptime = 1000000000;    ///< ������������ ����� �������� ���������� ������������ ���������
	void gps_callback(const jukovsky_ros::juk_dji_gps_msg::ConstPtr& input);    ///< ���������� ������� ��������� ������ GPS A3
	void precision_gps_callback(const jukovsky_ros::reach_msg::ConstPtr& in);    ///< ���������� ������� ��������� ������������ ������ GPS
	void set_target_callback(const jukovsky_ros::juk_set_target_data_msg::ConstPtr& input);    ///< ���������� ������� ��������� ����� ������� �����
	void action_process_callback(const jukovsky_ros::juk_navigation_actions_msg::ConstPtr& input);    ///< ���������� ������� ���������� ��������
	void aruco_callback(const jukovsky_ros::juk_aruco_module_data::ConstPtr& input);    ///< ���������� ������� ��������� ������ � ��������� ArUco �������

	//! ���������� ������ ����������� ����������
	/*!
	    \param offset �������� ������������ ���� � ������
	    \param current_velocity ������� ��������
	    \param course_need ����������� ���� (�������)
	    \param course_current ������� ���� (�������)
	    \param speed ��������� ��������
	    \param ctrl_mode ����� ����������
	    \return ���������, ���������� ��������� ���������� �3
	*/
	
	jukovsky_ros::juk_control_dji_msg calculateControl(GeoMath::v3 offset,
		GeoMath::v3 current_velocity,
		double course_need,
		double course_current,
		double speed,
		uint8_t ctrl_mode); 
	
	
	ros::Time node_start_time;    ///< ����� ������� ����
	bool set_homepoint_flag = true;    ///< ����, ������������, ����� �� ������� ���������
	
	GeoMath::v3geo	homepoint;    ///< ���������� ���������� (�3)
	GeoMath::v3geo	homepoint_precision;    ///< ���������� ���������� (������������)
	GeoMath::v3geo	current_point_abs;    ///< ������� ����������
	GeoMath::v3		current_velocity;    ///< ������� ��������
	GeoMath::v3     velocity_need; 
	GeoMath::v3     current_point_home;     ///< ������� ����� ������������ ����
	
	double homepoint_course;
	
	struct Target ///< ��������� ������� �����
	{
		GeoMath::v3geo	point_abs;     ///< ���������� ���������� ����
		uint8_t break_mode;    ///< ����� ����������
		float cruising_speed;    ///< ����������� ��������
		float accurancy;    ///< �������� ��������� �����
		float course;    ///< ���� ������������ ������
	}
	;
	
	struct ArUcoTarget ///< ��������� ��������� ������������ �������
	{
		GeoMath::v3 offset;    ///< �������� ������������ �������
		double course;    ///< ����
		ros::Time uptime;    ///<����� ���������� ���������� ������
		GeoMath::v3geo abs;
	};
	
	struct CtrlStatus
	{
		jukovsky_ros::juk_control_dji_msg msg;
		bool stable_now;
	};
	
	Target target;    ///< ����(���������� �� �3)
	Target current_target;    ///< ���� (��������� �������� ���� target, ���� target_precision )
	Target sub_target;    ///< �������
	Target pause_target;
	
	ArUcoTarget aruco_land;    ///< ������, �� ������� ����� ������������
	
	int STATE = STATES::IDLE;     ///< ������� ���������
	int SUB_STATE = SUB_STATES::BASIC_STATE;    ///<  ������� ������������
	int GPS_STATE = GPS_STATES::NO_SIGNAL;
	
	int flight_status=-1;   ///< ������ ������
	
	float yaw_rate;    ///< ������� ������� ��������
	
	bool stable_now;    ///< ����, ����������, ��������� �� ������� � ������� ����� � ������ ������
	bool stable_last;   ///< ����, ����������, ��������� �� ������� � ������� ����� �� ���������� �������� �����
	
	bool paused=false;
	
	ros::Time stable_start;
	ros::Time home_uptime;
	ros::Time precision_pos_uptime;    ///< ����� ���������� ���������� ������������ ���������
	ros::Time last_telemetry;
	
	double stable_time;
	
	ros::NodeHandle nh;
	ros::Publisher pub_dji_control;
	ros::Publisher pub_position_data;
	ros::Publisher pub_aruco_action;
	
	ros::Subscriber sub_dji_gps;
	ros::Subscriber sub_set_target;
	ros::Subscriber sub_precision_gps;
	ros::Subscriber sub_action_process;
	ros::Subscriber sub_aruco_data;
	
	GeoMath::v3 correction_RTK;
	
	uint8_t ctrl_mode; 	 ///< ������� ����� ����������
	
	uint8_t ctrl_flag;   ///< ����� ���������� �3 (��������)
	
	
	jukovsky_ros::juk_position_data_msg position_data;  ///< ������ ������ � ������� ���������
	jukovsky_ros::juk_control_dji_msg output_dji;  ///< ��������� ����������, ������������ �� �3
	
	GeoMath::v3geo precision_position;  ///< ������� ���������� (������������)
	GeoMath::v3geo a3_position;  ///< ������� ���������� (������������)

	
	int  precision_pos_quality;  ///< ������� �������� ��� ����������� ������������ ���������
	
	std::map<int, std::function<CtrlStatus()>> state_handlers;    ///< ���� - ���������, �������� - �������-���������� ��� ������� ���������
	
	void init_handlers();
	
	void print_telemetry(const ros::TimerEvent& event);
	
	std::map<const char*, std::stringstream> additional_telem_out;
	
	
	ros::Timer timer_telemetry;
	
	std::map<int, std::string> state_map = { { 0, "IDLE" }, { 1, "FLY_SIMPLE" }, { 2, "FLY_SAFE" }, { 3, "LAND_SIMPLE" }, { 4, "LAND_ARUCO" } };
	std::map<int, std::string> sub_state_map = {
		{ 0, "BASIC_STATE" },
	{ 201,"FLY_SAFE_UP"}, { 202,"FLY_SAFE_CENTER"},
	{ 301,"LANDING_SIMPLE_FLY"}, { 302,"LANDING_SIMPLE_LAND"},
	{ 401,"LANDING_ARUCO_FLY"}, { 402,"LANDING_ARUCO_LAND"},{ 403, "LANDING_ARUCO_HOLD_CENTER"},{ 404, "LANDING_ARUCO_RUSH"}
	};
	
	std::map<int, std::string> gps_state_map = {
		{0,"no signal"},
	{1,"basic GPS"},
	{2,"RTK"}
	};
	
	std::map<int, std::string> flight_status_map = { 
		{-1,"no data"},	
		{0,"disarmed"},	
		{1,"ready to flight"},	
		{2,"in the air"}
	};
	
};

#endif // !__NAVIGATION_NODE__
