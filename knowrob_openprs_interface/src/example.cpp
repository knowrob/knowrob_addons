#include "ros/ros.h"
#include "bremen_toulouse/Database_string.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "database_publisher");

	ros::NodeHandle n;

	ros::Publisher database_pub = n.advertise<bremen_toulouse::Database_string>("database_publisher", 1000);

	while (ros::ok())
	{
		bremen_toulouse::Database_string array;
		//Clear array
		array.text.clear();
		//for loop, pushing data in the size of the array
		for (int i = 0; i < 90; i++)
		{
			array.text.push_back("CNRS-LAARS and Institute for AI, Uni-Bremen");
		}
		//Publish array
		database_pub.publish(array);
		//Let the world know
		ROS_INFO("I published something!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}


	return 0;
}

