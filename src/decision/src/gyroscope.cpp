#include <cmath>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"

#define MIN 0.2
#define OMIGA 2
#define NODE "gyroscope"
#define NOW ros::Time::now().toSec()
#define omiga(dt) OMIGA * std::cos((dt) * PI)

#define Bool std_srvs::SetBool
#define Velocity geometry_msgs::Twist

double t;
bool defense = false;
const double PI = std::acos(-1);

bool gyroscope(Bool::Request &req, Bool::Response &res)
{
    if(defense = req.data)
    {
        t = NOW;
    }
    res.success = true;
    return true;
}

int main(int argc, char* argv[])
{
    Velocity v;
    ros::init(argc, argv, NODE);
	ros::Time::init();
	ros::Rate sleeper(10);
	ros::NodeHandle handle;
	ros::Publisher pub = handle.advertise<Velocity>("cmd_vel", 10);
    v.linear.z = v.angular.x = v.angular.y = v.linear.x = v.linear.y = 0;
    ros::ServiceServer srv = handle.advertiseService("gyroscope", gyroscope);
	while(ros::ok())
	{
		ros::spinOnce();
        if(defense)
        {
            double w = omiga(NOW - t);
            v.angular.z = (w >= 0? std::max(w, MIN): std::min(w, -MIN));
            pub.publish(v);
        }
		sleeper.sleep();
	}
	return 0;
}
