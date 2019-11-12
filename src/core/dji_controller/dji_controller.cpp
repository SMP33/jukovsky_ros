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

ArgParser_Int params;

Vehicle*   v;
const int F_MODE = 1684;  //числовое значение, которое соответствует f-позиции тумблера
auto default_ctrlFlag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_VELOCITY |
     Control::YAW_RATE | Control::STABLE_ENABLE | Control::HORIZONTAL_GROUND;


DJI::OSDK::Control::CtrlData            default_ctrlData(default_ctrlFlag, 0, 0, 0, 0);  //параметры управления полетом  по умолчанию
DJI::OSDK::Control::CtrlData            current_ctrlData(default_ctrlData);    //параметры управления полетом 


//Телеметрия с A3
DJI::OSDK::Telemetry::RCFullRawData  data_RC; 	// соостояние всех элементов на пульте управления
DJI::OSDK::Telemetry::Battery        data_Bat; 	// информация о заряде батареи
DJI::OSDK::Telemetry::GlobalPosition data_GPS; 	// координаты GPS
DJI::OSDK::Telemetry::Velocity       data_Velocity; 	// текущая скорость
DJI::OSDK::Telemetry::SDKInfo        data_Status; 	// содержит информацию, кто управляет аппаратом
double                               data_Course; 	// курс

jukovsky_ros::juk_dji_gps_msg msg_GPS;
jukovsky_ros::juk_dji_device_status_msg msg_device_status;

ros::Time last_ctrl_update_time;  //время последнего переключения источника управления

int ctrl_flag = jukovsky_ros::juk_control_dji_msg::flag_break;  //режим управления
#ifndef NO_GIMBAL
				 RotationAngle iAngle, nAngle, cAngle;   //углы подвеса при инициализации, требуемые, текущие в формате DJI SDK
 
#endif // !NO_GIMBAL

common_things::Time t;

struct
{
	float yaw;
	float pitch;
	float roll;
} initAngle, needAngle, currentAngle;//углы подвеса при инициализации, требуемые, текущие

#ifdef NO_DJI_HARDWARE


GeoMath::v3geo sim_pos(3.14 / 4, 3.14 / 3, 100);
const int freq = 5;
GeoMath::v3 vel(2 / (double)freq, 0, 0);
#else
const int freq = 30;  //частота общения с A3
#endif // NO_DJI_HARDWARE

int calc_gimbal_speed(int current, int need) //пропорциональный регулятор скорости
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

void ctrl_callback(const jukovsky_ros::juk_control_dji_msg::ConstPtr& msg) //обратный вызов для обновления параметров управления полетом 
{
	last_ctrl_update_time = ros::Time::now();
	current_ctrlData.x = msg->data_x;
	current_ctrlData.y = msg->data_y;
	current_ctrlData.z = msg->data_z;
	current_ctrlData.yaw = msg->course;	
	ctrl_flag = int(msg->flag);
	
	
}

void gimbal_camera_callback(const jukovsky_ros::juk_dji_camera_control_msg::ConstPtr& msg)// обратный вызов для указания положения подвеса и управления камерой
{

	

	if (params.args["enable_camera_gimbal"] != 0)
	{
		
		switch (msg->action)
		{
		case jukovsky_ros::juk_dji_camera_control_msg::take_photo:
			v->camera->shootPhoto();
			//cout << "Shoot Photo" << endl;
			break;
		case jukovsky_ros::juk_dji_camera_control_msg::start_video:
			v->camera->videoStart();
			break;
		case jukovsky_ros::juk_dji_camera_control_msg::stop_video:
			v->camera->videoStop();
			break;
		}
	}
	
#ifndef NO_GIMBAL
	nAngle.yaw = msg->yaw;
	nAngle.pitch = msg->pitch;
	nAngle.roll = msg->roll; 
#endif // !NO_GIMBAL


	
	
}

void update_data() //обновление телеметрии с A3
{
#ifndef NO_DJI_HARDWARE
	data_RC =
   v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA>();
	data_Bat =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO>();
	data_Velocity =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
	data_GPS = v->broadcast->getGlobalPosition();

	data_Status =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>();
	
	auto data_FlightStatus = 
		v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
	
	//пересчет квантернионов в углы эйлера
	Telemetry::Quaternion quat = v->broadcast->getQuaternion();         //
	
	if(params.args["print_quat"])
	{
		cout << "w: " << quat.q0 << "\tx: " << quat.q1 << "\ty: " << quat.q2 << "\tz: " << quat.q3  << endl;
	}
	
	double   q2sqr = quat.q2 * quat.q2; 								   //
	double   t0    = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0; 		   //
	double   t1    = + 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);   //
	double   t2    = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);    //
	double   t3    = + 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);   //
	double   t4    = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0; 		   //
																	   //
	t2 = (t2 > 1.0) ? 1.0 : t2; 										   //
	t2 = (t2 < -1.0) ? -1.0 : t2; 									   //
	data_Course = atan2(t1, t0);  									   //
	
	
	msg_GPS.flight_status = data_FlightStatus; //состояние, в котором находится аппарат (на земле, в полете и т.д.)
	
	msg_GPS.lat = data_GPS.latitude;
	msg_GPS.lng = data_GPS.longitude;
	msg_GPS.alt = data_GPS.altitude;
	msg_GPS.quality = data_GPS.health;
	msg_GPS.satellites = 0;
	msg_GPS.vx = data_Velocity.data.x;
	msg_GPS.vy = data_Velocity.data.y;
	msg_GPS.vz = data_Velocity.data.z;
	msg_GPS.course = data_Course;
	msg_GPS.q0 = quat.q0;
	msg_GPS.q1 = quat.q1;
	msg_GPS.q2 = quat.q2;
	msg_GPS.q3 = quat.q3;
	

	
	
#else 
	msg_GPS.vx = vel.x / 6370000.0;
	msg_GPS.vy = vel.y / 6370000.0;
	msg_GPS.vz = 0;
	
	sim_pos = sim_pos + vel;
	double r = (3.14 * 57)* (double)(-10 + rand() % 20) / 200;
	cout << "R: " << r << vel << endl;
	cout << sim_pos << endl;
	vel = vel.rotateXY(r*GeoMath::CONST.DEG2RAD);
	msg_GPS.lat = sim_pos.lat*GeoMath::CONST.DEG2RAD;
	msg_GPS.lng = sim_pos.lng*GeoMath::CONST.DEG2RAD;
	msg_GPS.alt = 200;
	msg_GPS.quality = 1;
	msg_GPS.satellites = 0;

	msg_GPS.course = 3.14;
	
	data_RC.lb2.mode = F_MODE;
#endif
	const long max_mute_duration = 500000000; 	//время ожидания обновления параметров управления
	
	switch(data_RC.lb2.mode)	
	{
	case F_MODE:	//управление передается автопилоту, если тумблер находится в позиции f
		{
			auto now = ros::Time::now();
#ifndef NO_DJI_HARDWARE
			
			if (data_Status.deviceStatus != 2)	//если управление идет не через SDK, забираем его
				{
					v->obtainCtrlAuthority();
					msg_device_status.changeTime = now; 
				}
			
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
			
			
			
			
#ifndef NO_GIMBAL
			Telemetry::Gimbal  gimbal;
			
			gimbal = v->broadcast->getGimbal();  //информация о подвесе а дыннй момент
			
			
			
			cAngle.roll  = gimbal.roll * 10 - iAngle.roll;
			cAngle.pitch = gimbal.pitch * 10;
			
			cAngle.yaw   = gimbal.yaw * 10;
		
			DJI::OSDK::Gimbal::SpeedData gimbalSpeed;  //скорость, которую необходимо задать подвесу
		
			nAngle.yaw = data_Course * 10*GeoMath::CONST.RAD2DEG;
			
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
		
			v->gimbal->setSpeed(&gimbalSpeed);  //задаем скорость подвесу
#endif
			
#endif
			
			msg_device_status.authority = jukovsky_ros::juk_dji_device_status_msg::CONTROL_BY_SDK;
			msg_device_status.armed = (data_FlightStatus != 0);
			
			
#ifdef DEBUG_ROS
			cout << "TIME: " << (now - last_ctrl_update_time).toNSec() << endl;
			cout << "DATA: " << current_ctrlData.x << " " << current_ctrlData.y << " " << current_ctrlData.z << " " << ctrl_flag << endl;
#endif // !DEBUG_ROS
			
			
		}
		break;
      
	default:
		{	
#ifndef NO_DJI_HARDWARE
			if (data_Status.deviceStatus == 2) //если управление идет через SDK, отпускаем его
				{
					v->releaseCtrlAuthority(5000);
					msg_device_status.changeTime = ros::Time::now(); 
				}
#endif
			
			msg_device_status.authority = jukovsky_ros::juk_dji_device_status_msg::CONTROL_BY_RC;
			msg_device_status.changeTime = ros::Time::now();
		
			break;
		}
	}
	
	msg_device_status.voltage = data_Bat.voltage;
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "JUK_DJI_CORE_NODE");
	
	DJI::OSDK::Log::instance().disableDebugLogging();
	
	DJI::OSDK::Log::instance().disableStatusLogging();
	DJI::OSDK::Log::instance().enableErrorLogging();
	//DJI::OSDK::Log::instance().disableErrorLogging();
	
	params.args["enable_camera_gimbal"] = 1;
	params.args["print_quat"] = 0;
	
	params.parse(argc, argv);
	std::cout << c(32, "@Parameters JUK_DJI_CORE_NODE: ") << std::endl;
	for (auto arg : params.args)
	{
		std::cout << c(32, "~~") << arg.first << ": " << c(32, arg.second) << std::endl;
	}
	std::cout << std::endl;
	
	usleep(1000000);
	
	string usr_config_path;
	
	std::string key;
	if (ros::param::search("usr_config_path", key))
	{
		ros::param::get(key, usr_config_path);
	}
	else
		exit(-1);
	
	ifstream UserConfig_file_in(usr_config_path);
	std::string line;
	std::string UserConfig_data;
	
	cout << grn("SDK configuration data is:") << endl;
	
	while (UserConfig_file_in) {
		std::getline(UserConfig_file_in, line);
		UserConfig_data = UserConfig_data + line + "\n";
		cout << mgt("\t"+line)<<endl;
	}
	UserConfig_file_in.close();
	
	
	
	std::ofstream UserConfig_file("UserConfig.txt");
	
	UserConfig_file << UserConfig_data;
	
	UserConfig_file.close();

	
	ros::NodeHandle nh;
	last_ctrl_update_time = ros::Time::now();
	ros::Publisher pub_GPS = nh.advertise<jukovsky_ros::juk_dji_gps_msg>("JUK/DJI/GPS", 1);
	ros::Publisher pub_device_status = nh.advertise<jukovsky_ros::juk_dji_device_status_msg>("JUK/DJI/DEVICE_STATUS", 1);
	
	ros::Subscriber sub = nh.subscribe("JUK/CONTROL_DJI", 1, ctrl_callback);
	ros::Subscriber sub_camera = nh.subscribe("JUK/DJI_GIMBAL", 1, gimbal_camera_callback);
	
#ifndef NO_DJI_HARDWARE
	LinuxSetup ls(argc, argv); 
	v = ls.getVehicle();

	auto st = v->broadcast->getStatus();

	//===============Подписываемся на нужные топики==========//
	ACK::ErrorCode subscribeStatus;
	subscribeStatus = v->subscribe->verify(5000);

	int                             pkgIndex        = 0;
	int                             freq            = 50;
	DJI::OSDK::Telemetry::TopicName topicList50Hz[] = {
		DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA,
		DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO,
		DJI::OSDK::Telemetry::TOPIC_VELOCITY,
		DJI::OSDK::Telemetry::TOPIC_GPS_FUSED,
		DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE,
		DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT
	};

	int  numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
	bool enableTimestamp = false;

	bool pkgStatus = v->subscribe->initPackageFromTopicList(
	  pkgIndex,
		numTopic,
		topicList50Hz,
		enableTimestamp,
		freq);

	subscribeStatus = v->subscribe->startPackage(pkgIndex, 50000);
	
#endif 

	//==========Основной цикл==========//
	ros::Rate r(freq);
	
	auto gimbal = v->broadcast->getGimbal();
		
	iAngle.roll  = gimbal.roll * 10;
	iAngle.pitch = gimbal.pitch * 10;
	iAngle.yaw   = gimbal.yaw * 10;
	
#ifndef NO_GIMBAL
	nAngle.yaw = 0;
	nAngle.pitch = 0;
	nAngle.roll = 0; 
#endif // !NO_GIMBAL

	
	DJI::OSDK::Gimbal::AngleData gimbalAngle;
	gimbalAngle.roll     = 0;
	gimbalAngle.pitch    = 0;
	gimbalAngle.yaw      = 0;
	gimbalAngle.duration = 20;
	gimbalAngle.mode |= 0;
	gimbalAngle.mode |= 0;
	gimbalAngle.mode |= 0 << 1;
	gimbalAngle.mode |= 0 << 2;
	gimbalAngle.mode |= 0 << 3;

	v->gimbal->setAngle(&gimbalAngle);
	
	usleep(3e6);
	
	
	while (ros::ok())
	{
		update_data();
		msg_GPS.deb = "A3";
		pub_GPS.publish(msg_GPS);
		pub_device_status.publish(msg_device_status);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}