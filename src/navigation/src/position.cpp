/* @author: YueLin */

#include <string.h>

#include "tf/tf.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "sentry/RobotID.h"
#include "sentry/Position.h"

#define RED -1
#define BLUE 1
#define SCALE 1e2
#define INIT ros::init(argc, argv, "position")

using namespace sentry;

int main(int argc, char* argv[])
{
    INIT;
    RobotID rid;
    Position position;
    ros::Time::init();
    ros::NodeHandle nh;
    ros::Rate sleeper(50);
    tf::StampedTransform transform;
    tf::TransformListener listener;
    ros::Publisher publisher = nh.advertise<Position>("position", 1);
    ros::ServiceClient robot = nh.serviceClient<RobotID>("robot_id");
    listener.waitForTransform(
        "map", "base_link", ros::Time(0), ros::Duration(10)
    );
    ros::service::waitForService("robot_id");
    rid.request.num = rid.request.color = true;
    if(robot.call(rid))
        position.id = rid.response.id;
    while(ros::ok())
    {
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        position.x = std::round(transform.getOrigin().x() * SCALE);
        position.y = std::round(transform.getOrigin().y() * SCALE);
        position.yaw = tf::getYaw(transform.getRotation());
        publisher.publish(position);
        sleeper.sleep();
    }
    return 0;
}
