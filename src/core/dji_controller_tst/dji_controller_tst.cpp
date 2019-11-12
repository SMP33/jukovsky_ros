#include <dji_status.hpp> 
#include <dji_vehicle.hpp>
#include "common/dji_linux_helpers.hpp"

#include <jukovsky_ros/juk_dji_gps_msg.h>
#include <jukovsky_ros/juk_dji_device_status_msg.h>
#include <jukovsky_ros/juk_control_dji_msg.h>
#include <jukovsky_ros/juk_dji_camera_control_msg.h>

#include <iostream>
#include <fstream>
#include <LinuxChrono.h>
#include <ros/ros.h>

#include <GeoMath.h>
#include <cstdlib>

#include "gimbal/dji_gimbal.h"
#include "../include/ArgParser.h"

#define c(color,str)  "\x1B["<<color<<"m" << str << "\033[0m"
#define cyan_b(str)   "\x1B[46;1m" << str << "\033[0m" 
#define gray_b(str)   "\x1B[40m" << str << "\033[0m" 
#define red_b(str)   "\x1B[41;1m" << str << "\033[0m" 
#define green_b(str)   "\x1B[42;1m" << str << "\033[0m" 


#define grn(str)  "\x1B[32;1m" << str << "\x1B[39m" 
#define mgt(str)  "\x1B[35;1m" << str << "\x1B[39m" 
#define red(str)  "\x1B[31;1m" << str << "\x1B[39m" 

#define CLEAR(rows)               printf("\033[%02dA\033[0J",rows+1)


//#define DEBUG_ROS
//
//#define NO_DJI_HARDWARE
//
//#define NO_GIMBAL


using namespace std;
using namespace GeoMath;

class DJI_Controller
{
public:
	struct Telemetry
	{
		v3geo position_abs;
		v3 velocity;
		v3 orientation;
		bool enable_SDK;
	};
	DJI::OSDK::Control::CtrlData            default_ctrlData;
	DJI::OSDK::Control::CtrlData            current_ctrlData;
	RotationAngle nAngle, cAngle; //углы подвеса: требуемые, текущие в формате DJI SDK
	
	ros::Time last_ctrl_update_time;   //время последнего переключения источника управления
	
	int ctrl_flag = jukovsky_ros::juk_control_dji_msg::flag_break;
	
	void ctrl_callback(const jukovsky_ros::juk_control_dji_msg::ConstPtr& msg) //обратный вызов для обновления параметров управления полетом 
	{
		last_ctrl_update_time = ros::Time::now();
		current_ctrlData.x = msg->data_x;
		current_ctrlData.y = msg->data_y;
		current_ctrlData.z = msg->data_z;
		current_ctrlData.yaw = msg->course;	
		ctrl_flag = int(msg->flag);
	}
	
	void gimbal_camera_callback(const jukovsky_ros::juk_dji_camera_control_msg::ConstPtr& msg)// обработчик команд указания положения подвеса и управления камерой
	{
		//TODO добавить выключалку подвеса
		if(true)
		{
			switch (msg->action)
			{
			case jukovsky_ros::juk_dji_camera_control_msg::take_photo:
				v->camera->shootPhoto();
				break;
			case jukovsky_ros::juk_dji_camera_control_msg::start_video:
				v->camera->videoStart();
				break;
			case jukovsky_ros::juk_dji_camera_control_msg::stop_video:
				v->camera->videoStop();
				break;
			}
		}
		
		nAngle.yaw = msg->yaw;
		nAngle.pitch = msg->pitch;
		nAngle.roll = msg->roll; 
		
	}
	
	ros::NodeHandle nh;
	ros::Publisher pub_GPS = nh.advertise<jukovsky_ros::juk_dji_gps_msg>("JUK/DJI/GPS", 1);
	ros::Publisher pub_device_status = nh.advertise<jukovsky_ros::juk_dji_device_status_msg>("JUK/DJI/DEVICE_STATUS", 1);
	
	ros::Subscriber sub = nh.subscribe("JUK/CONTROL_DJI", 1, &DJI_Controller::ctrl_callback,this);
	ros::Subscriber sub_camera = nh.subscribe("JUK/DJI_GIMBAL", 1, &DJI_Controller::gimbal_camera_callback, this);
	
	jukovsky_ros::juk_dji_gps_msg msg_GPS;
	jukovsky_ros::juk_dji_device_status_msg msg_device_status;
	
	DJI_Controller(Vehicle*   v_);
	Telemetry data;
	void upd();

private:
	Vehicle*   v;
	const long max_mute_duration = 5e8;  	//максимальное время ожидания обновления параметров управления, нс
	
	static int calc_gimbal_speed(int current, int need) //пропорциональный регулятор скорости
	{
		int speed = 800;
		int break_dist = 300;
		int dist = need - current;
		if (abs(dist) < 30) return 0;
	
		if (abs(dist) < break_dist) speed = speed / 2;
		if (abs(dist) < break_dist / 2) speed = speed / 3;
	
		if (dist > 0) return speed;
		return -speed;
	}
	
	void upd_gimbal()
	{
		auto gimbal = v->broadcast->getGimbal();
			
		cAngle.roll  = gimbal.roll * 10;
		cAngle.pitch = gimbal.pitch * 10;
		cAngle.yaw   = gimbal.yaw * 10;
		
		DJI::OSDK::Gimbal::SpeedData gimbalSpeed;   //скорость, которую необходимо задать подвесу
		
		nAngle.yaw = this->data.orientation.x * 10*GeoMath::CONST.RAD2DEG;
			
		gimbalSpeed.roll  = calc_gimbal_speed(cAngle.roll, nAngle.roll);
		gimbalSpeed.pitch = calc_gimbal_speed(cAngle.pitch, nAngle.pitch);
		gimbalSpeed.yaw   = calc_gimbal_speed(cAngle.yaw, nAngle.yaw);
			
		gimbalSpeed.gimbal_control_authority = 1;
		gimbalSpeed.disable_fov_zoom = 0;
		gimbalSpeed.ignore_user_stick = 1;
		gimbalSpeed.extend_control_range = 0;
		gimbalSpeed.ignore_aircraft_motion = 0;
		gimbalSpeed.yaw_return_neutral = 0;
			
		gimbalSpeed.reserved0 = 0;
		gimbalSpeed.reserved1 = 0;
		
		v->gimbal->setSpeed(&gimbalSpeed);
	}
	
	void subscribe_dji_topics()
	{
		DJI::OSDK::Telemetry::TopicName topicList50Hz[] = {
			DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA,
			DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO,
			DJI::OSDK::Telemetry::TOPIC_VELOCITY,
			DJI::OSDK::Telemetry::TOPIC_GPS_FUSED,
			DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE,
			DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT
		};
	
		ACK::ErrorCode subscribeStatus;
		subscribeStatus = v->subscribe->verify(5000);
	
		int                             pkgIndex        = 0;
		int                             freq            = 50;

		int  numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
		bool enableTimestamp = false;

		bool pkgStatus = v->subscribe->initPackageFromTopicList(
		  pkgIndex,
			numTopic,
			topicList50Hz,
			enableTimestamp,
			freq);
	
		subscribeStatus = v->subscribe->startPackage(pkgIndex, 50000);
	}
};

DJI_Controller::DJI_Controller(Vehicle*   v_)
	:v(v_)
	,default_ctrlData(Control::VERTICAL_VELOCITY | Control::HORIZONTAL_VELOCITY |
     Control::YAW_RATE | Control::STABLE_ENABLE | Control::HORIZONTAL_GROUND,
		0,
		0,
		0,
		0)
	, current_ctrlData(default_ctrlData)
{
	DJI::OSDK::Log::instance().disableDebugLogging();
	DJI::OSDK::Log::instance().enableStatusLogging();
	DJI::OSDK::Log::instance().enableErrorLogging();
	
	subscribe_dji_topics();
	
}

void
DJI_Controller::upd()
{
	auto now = ros::Time::now();
	
	auto data_RC =
		v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA>();
	auto data_Bat =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO>();
	auto data_FlightStatus = 
		v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
	auto data_Status =
		v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>();
	auto data_GPS = v->broadcast->getGlobalPosition();
	auto data_Velocity =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
	auto data_Q = v->broadcast->getQuaternion();
	
	this->data.position_abs = v3geo(data_GPS.latitude, data_GPS.longitude, data_GPS.altitude);
	this->data.velocity = v3(data_Velocity.data.x, data_Velocity.data.y, data_Velocity.data.z);
	this->data.orientation = GeoMath::quat(data_Q.q0, data_Q.q1, data_Q.q2, data_Q.q3).to_eul();
	
	//=====Чекаем, кто управляем=====//
	switch(data_RC.lb2.mode)
	{
	case 1684: //режим F
		{
		
		if(data_Status.deviceStatus != 2)	//если управление идет не через SDK, забираем его
		{
			v->obtainCtrlAuthority();
			msg_device_status.changeTime = now;
		}
		this->data.enable_SDK = true;
		
		
		if ((now - last_ctrl_update_time).toNSec() > max_mute_duration) //если параметры управления не былы обновлены допустимое время назад, тормозим
			{
				current_ctrlData = default_ctrlData;
				v->control->emergencyBrake();
			}
				
		else // иначе посылаем данные на А3 определенным способом в зависимости от выбранного режима
			{
				//cout << "M: " << ctrl_flag << endl;
				switch(ctrl_flag)
				{
				case 5: //управляем напрямую скоростью
					current_ctrlData.flag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_VELOCITY |
											Control::YAW_RATE | Control::STABLE_ENABLE | Control::HORIZONTAL_GROUND;
					v->control->flightCtrl(current_ctrlData);
					break;
				case 7: //управляем скоростью по вертикали и задаем расстояние до цели по горизонтали
					current_ctrlData.flag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_POSITION |
											Control::YAW_RATE | Control::STABLE_ENABLE | Control::HORIZONTAL_GROUND;
					v->control->flightCtrl(current_ctrlData);
					break;
				case 8: //управляем скоростью по вертикали и задаем расстояние до цели по горизонтали (относительно тела аппарата)
				current_ctrlData.flag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_POSITION |
										Control::YAW_RATE | Control::STABLE_ENABLE | Control::HORIZONTAL_BODY;
					v->control->flightCtrl(current_ctrlData);
					break;
				case 13: //экстренное торможение
					v->control->emergencyBrake();
					break;
				default:
					v->control->emergencyBrake();
					break;
				}			
				
			}
		msg_device_status.authority = jukovsky_ros::juk_dji_device_status_msg::CONTROL_BY_SDK;
	}
		break;
	default:
		if (data_Status.deviceStatus == 2) //если управление идет через SDK, возвращаем его
			{
				v->releaseCtrlAuthority(5000);
				msg_device_status.changeTime = now;
			}
		data.enable_SDK = false;
		msg_device_status.authority = jukovsky_ros::juk_dji_device_status_msg::CONTROL_BY_RC;
		
		break;
	}
	
	msg_GPS.flight_status = data_FlightStatus;  //состояние, в котором находится аппарат (на земле, в полете и т.д.)
	
	msg_GPS.lat = data_GPS.latitude;
	msg_GPS.lng = data_GPS.longitude;
	msg_GPS.alt = data_GPS.altitude;
	msg_GPS.quality = data_GPS.health;
	msg_GPS.satellites = 0;
	msg_GPS.vx = data_Velocity.data.x;
	msg_GPS.vy = data_Velocity.data.y;
	msg_GPS.vz = data_Velocity.data.z;
	msg_GPS.course = this->data.orientation.x;
	msg_GPS.q0 = data_Q.q0;
	msg_GPS.q1 = data_Q.q1;
	msg_GPS.q2 = data_Q.q2;
	msg_GPS.q3 = data_Q.q3;
	msg_GPS.angle_x = this->data.orientation.x;
	msg_GPS.angle_y = this->data.orientation.y;
	msg_GPS.angle_z = this->data.orientation.z;
	
	
	msg_device_status.armed = (data_FlightStatus != 0);
	msg_device_status.voltage = data_Bat.voltage;
	
	pub_GPS.publish(msg_GPS);
	pub_device_status.publish(msg_device_status);
	
	upd_gimbal();
	
}

void generate_UserConfig()
{
	string usr_config_path;
	std::string key;
	if (ros::param::search("usr_config_path", key))
	{
		ros::param::get(key, usr_config_path);
	}
	else
	{
		cout << red("ERROR: no UserGonfig file path") << endl;
		exit(-1);
	}
	
	ifstream UserConfig_file_in(usr_config_path);
	std::string line;
	std::string UserConfig_data;
	
	cout << grn("User configuration data is:") << endl;
	
	while (UserConfig_file_in) {
		std::getline(UserConfig_file_in, line);
		UserConfig_data = UserConfig_data + line + "\n";
		cout << mgt("\t" + line) << endl;
	}
	UserConfig_file_in.close();
	std::ofstream UserConfig_file("UserConfig.txt");
	UserConfig_file << UserConfig_data;
	UserConfig_file.close();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "JUK_DJI_CORE_NODE");
	
	generate_UserConfig();
	
	LinuxSetup ls(argc, argv); 
	Vehicle* v = ls.getVehicle();
	
	DJI_Controller A3(v);
	
	ros::Rate rate(30);
	
	while (ros::ok())
	{
		A3.upd();
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}