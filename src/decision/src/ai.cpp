/* @author: YueLin */

#include "tf/tf.h"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "bfs.hpp"
#include "sentry/plan.h"

#define MOVE 20
#define NOW ros::Time::now().toSec()
#define INIT ros::init(argc, argv, "ai")

typedef sentry::plan Plan;
typedef std_srvs::SetBool Gyroscope;
typedef geometry_msgs::PoseWithCovarianceStamped InitalPose;

bool start = false;

bool go(Gyroscope::Request &req, Gyroscope::Response &res)
{
    start = req.data;
    return res.success = true;
}

InitalPose initial(int x, int y, float yaw)
{
    InitalPose pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = x * 1e-2;
    pose.pose.pose.position.y = y * 1e-2;
    pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pose;
}

int main(int argc, char* argv[])
{
    INIT;
    Map map;
    Plan target;
    Position back;
    Gyroscope gyr;
    double yaw = 0;
    int x = 0, y = 0;
    ros::Time::init();
    ros::NodeHandle nh;
    ros::Subscriber _1 = nh.subscribe<Map>(
        "costmap", 1, [&map](Map::ConstPtr cost)
        {
            map = *cost;
        }
    );
    ros::Subscriber _2 = nh.subscribe<Position>(
        "enemy", 1, [&x, &y, &yaw](Position::ConstPtr pos)
        {
            x = pos->x; y = pos->y; yaw = pos->yaw;
        }
    );
    tf::TransformListener wait;
    ros::ServiceClient nav = nh.serviceClient<Plan>("plan");
    ros::ServiceServer _0 = nh.advertiseService("start", go);
    ros::Publisher init = nh.advertise<InitalPose>("initialpose", 1);
    ros::ServiceClient gyroscope = nh.serviceClient<Gyroscope>("gyroscope");
    nav.waitForExistence(); gyroscope.waitForExistence();
    wait.waitForTransform(
        "map", "base_link", ros::Time(0), ros::Duration(10)
    );
    int now = NOW;
    init.publish(initial(90, 73, std::acos(-1) / 2));
WAIT:
    while(!start) ros::spinOnce();
    while(ros::ok())
    {
        ros::spinOnce();
        if(!start) goto WAIT;
        if(!map.data.size())
            continue;
        back = search::back(map, x, y, yaw);
        if(std::max(std::abs(x - back.x), std::abs(y - back.y)) > MOVE)
        {
            gyr.request.data = false;
            target.request.x = back.x; target.request.y = back.y;
            gyroscope.call(gyr); nav.call(target);
        }
        else if(!gyr.request.data)
        {
            gyr.request.data = true;
            gyroscope.call(gyr);
        }
    }
}
