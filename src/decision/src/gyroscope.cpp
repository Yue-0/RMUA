/* @author: YueLin */

#include <cmath>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"

#define MIN 0.2
#define OMIGA 2
#define NOW ros::Time::now().toSec()
#define omiga(dt) OMIGA * std::cos((dt) * PI)
#define INIT ros::init(argc, argv, "gyroscope")

typedef geometry_msgs::Twist Velocity;
typedef std_srvs::SetBool::Request Request;
typedef std_srvs::SetBool::Response Response;

double t;
bool defense = false;
const double PI = std::acos(-1);

bool gyroscope(Request &req, Response &res)
{
    if(defense = req.data)
        t = NOW;
    return res.success = true;
}

int main(int argc, char* argv[])
{
    INIT;
    Velocity v;
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
