#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define FILTER 1/3
#define Ls sensor_msgs::LaserScan
#define INIT ros::init(argc, argv, "lidar")

Ls ls;
int len, f;

void filter(Ls::ConstPtr data)
{
	if(!len)
	{
		len = data->ranges.size();
		len -= (f = (len * FILTER) >> 1) << 1;
	}
	ls = *data;
	std::vector<float> ranges(len), intensities(len);
	for(int angle = 0; angle < len; angle++)
	{
		ranges[angle] = data->ranges[angle + f];
		intensities[angle] = data->intensities[angle + f];
	}
	ls.ranges = ranges;
	ls.intensities = intensities;
	ls.angle_min += f * ls.angle_increment;
	ls.angle_max = ls.angle_min + (len - 1) * ls.angle_increment;
}

int main(int argc, char* argv[])
{
	INIT;
	ros::Time::init();
	ros::Rate sleeper(10);
	ros::NodeHandle handle;
	ros::Publisher publisher = handle.advertise<Ls>("scan", 1);
	ros::Subscriber subscriber = handle.subscribe<Ls>("laser_scan", 1, filter);
	while(ros::ok())
	{
		ros::spinOnce();
		publisher.publish(ls);
		sleeper.sleep();
	}
	return 0;
}
