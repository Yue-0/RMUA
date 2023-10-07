#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char* argv[])
{
    std_msgs::Empty msg;
    ros::init(argc, argv, "loop");
    ros::Time::init();
	ros::Rate sleeper(10);
	ros::NodeHandle handle;
    ros::Publisher publisher = handle.advertise<std_msgs::Empty>("loop", 1);
    while(ros::ok())
    {
        publisher.publish(msg);
        sleeper.sleep();
    }
}
