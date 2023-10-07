#include "tf/tf.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "navigation/Position.h"

#define SCALE 1e2
#define INIT ros::init(argc, argv, "position")

using namespace navigation;

int main(int argc, char* argv[])
{
    INIT;
    Position position;
    ros::Time::init();
    ros::NodeHandle nh;
    ros::Rate sleeper(50);
    tf::StampedTransform transform;
    tf::TransformListener listener;
    ros::Publisher publisher = nh.advertise<Position>("position", 1);
    listener.waitForTransform(
        "map", "base_link", ros::Time(0), ros::Duration(10)
    );
    while(ros::ok())
    {
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        position.x = std::round(transform.getOrigin().x() * SCALE);
        position.y = std::round(transform.getOrigin().y() * SCALE);
        position.yaw = tf::getYaw(transform.getRotation());
        publisher.publish(position);
        sleeper.sleep();
    }
}
