#include "ArgParser.h"

ArgParser_Int::ArgParser_Int()
	: args()
{
}



void ArgParser_Int::add(const char* name, int base_value)
{
	args[name] = base_value;
}

void ArgParser_Int::add(std::pair<const char*, int> arg)
{
	
}

void ArgParser_Int::add(std::map<const char*, int> arg_map)
{
	for (auto arg : arg_map)
	{
		args[arg.first] = arg.second;
	}
}

void ArgParser_Int::parse(std::string str)
{
	for (auto& elem : args)
	{
		std::string needle = std::string(elem.first) + "=";

		int sl = str.length(), nl = needle.length();

		int pos_find = (int)str.find(needle);

		if (pos_find == 0 && sl > nl)
		{
			elem.second = std::stoi(str.substr(nl, sl - nl));
		}
	}

}

void ArgParser_Int::parse(int argc, char** argv)
{
	for (int i = 0; i < argc; i++)
	{
		std::string str = argv[i];
		parse(str);
	}
} 