/* @author: YueLin */

#include <vector>

#include "tf/tf.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"

#include "sentry/Points.h"

#define FRAME "map"
#define INIT ros::init(argc, argv, "points")

typedef sentry::Points Points;
typedef tf::TransformListener Listener;
typedef sensor_msgs::LaserScan LaserScan;

int main(int argc, char* argv[])
{
	INIT;
    Points points;
    LaserScan ls[2];
	ros::Time::init();
    points.frame = FRAME;
	ros::Rate sleeper(100);
	ros::NodeHandle handle;
    bool pub[2] = {false, false};
    tf::StampedTransform transform[2];
    tf::TransformListener listener[2];
	ros::Publisher publisher = handle.advertise<Points>("scan", 1);
    ros::Subscriber subscriber1 = handle.subscribe<LaserScan>(
        "scan1", 1, [&ls, &pub](LaserScan::ConstPtr data)
        {
            ls[0] = *data; pub[0] = true;
        }
    );
    ros::Subscriber subscriber2 = handle.subscribe<LaserScan>(
        "scan2", 1, [&ls, &pub](LaserScan::ConstPtr data)
        {
            ls[1] = *data; pub[1] = true;
        }
    );
    for(char frame[7] = "laser1"; frame[5] <= '2'; frame[5]++)
    {
        listener[frame[5] - '1'].waitForTransform(
            FRAME, frame, ros::Time(0), ros::Duration(10)
        );
    }
	while(ros::ok())
	{
        ros::spinOnce();
        if(!pub[0] || !pub[1])
            continue;
        std::vector<double> x, y;
        for(char frame[7] = "laser1"; frame[5] <= '2'; frame[5]++)
        {
            int id = frame[5] - '1';
            listener[id].lookupTransform(
                FRAME, frame, ros::Time(0), transform[id]
            );
            double angle = ls[id].angle_min,
            x0 = transform[id].getOrigin().x(),
            y0 = transform[id].getOrigin().y(),
            yaw = tf::getYaw(transform[id].getRotation());
            for(double d: ls[id].ranges)
            {
                if(d <= ls[id].range_max && d >= ls[id].range_min)
                {
                    x.push_back(x0 + d * std::cos(yaw + angle));
                    y.push_back(y0 + d * std::sin(yaw + angle));
                }
                angle += ls[id].angle_increment;
            }
        }
        points.x = x;
        points.y = y;
        points.len = x.size();
		publisher.publish(points);
		sleeper.sleep();
	}
	return 0;
}
