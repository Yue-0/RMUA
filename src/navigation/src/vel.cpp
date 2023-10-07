#include <cmath>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"

#include "navigation/Position.h"

#define INIT ros::init(argc, argv, "vel")

#define Path nav_msgs::Path
#define Position navigation::Position
#define Velocity geometry_msgs::Twist

Path path;
Velocity vel, v;
Position position;

void speed(Velocity::ConstPtr v0)
{
    v = *v0;
}

void points(Path::ConstPtr points)
{
    path = *points;
}

void locate(Position::ConstPtr pos)
{
    position = *pos;
}

int main(int argc, char* argv[])
{
    INIT;
    ros::Time::init();
    ros::NodeHandle nh;
    ros::Rate sleeper(100);
    vel.linear.z = vel.angular.x = vel.angular.y = 0;
    ros::Publisher pub = nh.advertise<Velocity>("cmd_vel", 1);
    ros::Subscriber _1 = nh.subscribe<Path>("path", 1, points);
    ros::Subscriber _2 = nh.subscribe<Velocity>("cmd_vel", 1, speed);
    ros::Subscriber _3 = nh.subscribe<Position>("position", 1, locate);
    while(ros::ok())
    {
        ros::spinOnce();
        if(position.x && position.y && path.poses.size())
        {
            
        }
        sleeper.sleep();
    }
    return 0;
}
