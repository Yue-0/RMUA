#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define LEN 360
#define FILTER 120
#define NODE "lidar_filter"
#define Ls sensor_msgs::LaserScan

Ls ls;

void filter(Ls::ConstPtr data)
{
	const int f = FILTER >> 1;
	const int len = LEN - FILTER;
	float increment = data->angle_increment;
	std::vector<float> ranges(len), intensities(len);
	ls.header = data->header;
	ls.range_min = data->range_min;
	ls.range_max = data->range_max;
	ls.scan_time = data->scan_time;
	ls.time_increment = data->time_increment;
	for(int angle = 0; angle < len; angle++)
	{
		ranges[angle] = data->ranges[angle + f];
		intensities[angle] = data->intensities[angle + f];
	}
	ls.ranges = ranges;
	ls.intensities = intensities;
	ls.angle_increment = increment;
	ls.angle_min = data->angle_min + f * increment;
	ls.angle_max = ls.angle_min + (len - 1) * increment;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, NODE);
	ros::Time::init();
	ros::Rate sleeper(10);
	ros::NodeHandle handle;
	ros::Publisher publisher = handle.advertise<Ls>("scan", 1);
	ros::Subscriber subscriber = handle.subscribe<Ls>("base_scan", 1, filter);
	while(ros::ok())
	{
		ros::spinOnce();
		publisher.publish(ls);
		sleeper.sleep();
	}
	return 0;
}
