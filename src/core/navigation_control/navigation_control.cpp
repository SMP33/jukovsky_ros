#include <iostream>
#include "NavigationNode.h"
#include <vector>


using namespace std;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "JUK_NAVIGATION_NODE");
	vector<string> args(0);
	
	for (size_t i = 1; i < argc; i++)
	{
		args.push_back(string(argv[i]));
	}
	
		
	NavigationNode navigationNode(argc,argv);
	ros::spin();
	return 0;
}