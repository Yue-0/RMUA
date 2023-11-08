/* @author: YueLin */

#include <string.h>

#include "ros/ros.h"

#include "decision/RobotID.h"

#define RED -1
#define BLUE 1
#define INIT ros::init(argc, argv, "robot_id")

using namespace decision;

short id, num, color;

bool srv(RobotID::Request &req, RobotID::Response &res)
{
    if(req.num && req.color)    res.id = id;
    else if(req.num)            res.id = num;
    else if(req.color)          res.id = color;
    else return false;          return true;
}

int main(int argc, char* argv[])
{
    INIT; ros::Time::init(); ros::NodeHandle nh;
    ros::ServiceServer _ = nh.advertiseService("robot_id", srv);
    if(!strcmp(argv[2], "1"))         num = 1;
    else if(!strcmp(argv[2], "2"))    num = 2;
    else ROS_ERROR(
        "Robot number error: Expect 1 or 2, but got %s", argv[1]
    );
    if(!strcmp(argv[1], "red"))       color = RED;
    else if(!strcmp(argv[1], "blue")) color = BLUE;
    else ROS_ERROR(
        "Robot color error: Expect 'red' or 'blue', but got %s", argv[1]
    );
    id = color * num;                 ros::spin();
}
