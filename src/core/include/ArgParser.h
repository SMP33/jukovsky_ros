#pragma once

#ifndef __ARG_PARSER__
#define __ARG_PARSER__

#include <iostream>
#include <map>  
#include <vector>
#include <string>

class ArgParser_Int
{
public:
	ArgParser_Int();
	std::map< const char*, int> args;
	void add(const char* name, int base_value = 0);
	void add(std::pair<const char*, int> arg);
	void add(std::map<const char*, int> arg);
	void parse(std::string str);
	void parse(int argc, char** argv);


};


#endif // !__ARG_PARSER__
