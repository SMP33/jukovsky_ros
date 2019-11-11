#include <ros/ros.h>

#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <sstream>
#include <iostream>
#include <pthread.h>


std::vector<std::string> split(const std::string& str, const std::string& delim)
{
	std::vector<std::string> tokens;
	size_t prev = 0, pos = 0;
	do
	{
		pos = str.find(delim, prev);
		if (pos == std::string::npos) pos = str.length();
		std::string token = str.substr(prev, pos - prev);
		if (!token.empty()) tokens.push_back(token);
		prev = pos + delim.length();
	} while (pos < str.length() && prev < str.length());
	return tokens;
}

template<class T>
	
	class Log_msg
	{
	private:
		std::string topic_name;
		T msg;
		
		ros::Time last_upd_time;
		ros::Time start_time;
		ros::Subscriber sub;
		
		bool upd;
		bool first_upd;
		
		void callback(const typename T::ConstPtr& inp)
		{
			msg = *inp;
			last_upd_time = ros::Time::now();
			upd = true;
			first_upd = true;
		}
		
	public:
		Log_msg(ros::NodeHandle* nh, std::string topic_name_, ros::Time start_time_)
			: topic_name(topic_name_)
			, start_time(start_time_)
			, upd(false)
			, first_upd(false)
		{
			sub = nh->subscribe(topic_name, 1, &Log_msg::callback, this);
		}
		
		std::string log()
		{
			std::stringstream buf;
			
			buf.precision(12);
			
			buf << "last_upd_time: " << (first_upd ? (last_upd_time - start_time) : ros::Duration(-1.0)) << std::endl;
			buf << "upd: " << (upd ? "True" : "False") << std::endl;
			buf << msg;
			
			upd = false;
			
			return buf.str();
		}
		
		std::string json()
		{
			std::string txt = log();
			
			auto lines = split(txt, "\n");
			
			std::stringstream buf;
			
			buf << "\t\"" << topic_name << "\":{\n";
			
			buf.precision(12);
			
			
			for (int i = 0; i < lines.size()-1; i++)
			{
				auto& line = lines[i];
				int dd = line.find(":");
				buf << "\t\t\"" << line.substr(0, dd + 1) << "\": " << "\"" << line.substr(dd + 2, line.length() - 1) << "\"" << ",\n";
			}
			
			auto& line = lines[lines.size() - 1];
			int dd = line.find(":");
			buf << "\t\t\"" << line.substr(0, dd + 1) << "\": " << "\"" << line.substr(dd + 2, line.length() - 1) << "\"" << "\n";
			
			buf << "}";
			return buf.str();
		}
		
	};



class SimpleServer
{
public:
	SimpleServer(int  PORT);
	~SimpleServer();
	void* run();
	void set_response(std::string str);

private:
	int server_fd, client, valread;
	struct sockaddr_in address; 
	int opt = 1; 
	int addrlen = sizeof(address); 
	uint PORT ;
	std::string response;
	pthread_t thr;
	pthread_mutex_t mutex;
	
	static void*run_helper(void* context)
	{
		((SimpleServer*)context)->run();
	}
};

SimpleServer::SimpleServer(int  PORT_):
	PORT(PORT_),
	response(" ")
{
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
	{ 
		perror("socket failed"); 
		exit(EXIT_FAILURE); 
	} 
	
	// Forcefully attaching socket to the port 8080 
	if(setsockopt(server_fd,
		SOL_SOCKET,
		SO_REUSEADDR | SO_REUSEPORT, 
		&opt,
		sizeof(opt))) 
	{ 
		perror("setsockopt"); 
		exit(EXIT_FAILURE); 
	} 
	address.sin_family = AF_INET; 
	address.sin_addr.s_addr = INADDR_ANY; 
	address.sin_port = htons(PORT); 
	
	// Forcefully attaching socket to the port 8080 
	if(bind(server_fd,
		(struct sockaddr *)&address, 
		sizeof(address)) < 0) 
	{ 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	} 
	if (listen(server_fd, 3) < 0) 
	{ 
		perror("listen"); 
		exit(EXIT_FAILURE); 
	} 
	pthread_mutex_init(&mutex, NULL);
	pthread_create(&thr,NULL,&SimpleServer::run_helper,this);
}

void*
SimpleServer::run()
{
	while (1)
	{
		client = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
		char buffer[4096] = { 0 }; 
		read(client, buffer, 4096); 
		pthread_mutex_lock(&mutex);
		std::string str = response;
		pthread_mutex_unlock(&mutex);
		send(client, str.c_str(), str.length(), 0); 
		close(client);
	
	}
}

void
SimpleServer::set_response(std::string str)
{
	pthread_mutex_lock(&mutex);
	response = "HTTP/1.1 200 OK\n\n" + str;
	pthread_mutex_unlock(&mutex);
}

SimpleServer::~SimpleServer()
{
}