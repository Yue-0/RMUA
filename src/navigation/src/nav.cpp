#include <cmath>

#include "tf/tf.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalID.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

#include "navigation/Position.h"

#define DX 0.3
#define DY 0.05
#define NODE "nav"
#define NOW ros::Time::now().toSec()
#define goal "move_base_simple/goal"
#define SqrtL2(x1,x2) std::sqrt((x1)*(x1) + (x2)*(x2))

#define Empty std_msgs::Empty
#define Bool std_srvs::SetBool
#define Cancel actionlib_msgs::GoalID
#define Velocity geometry_msgs::Twist
#define Position navigation::Position
#define Target geometry_msgs::PoseStamped

Target pose;
Velocity vel;
bool stop, start = false;
double X, Y, YAW, x, y, yaw;

void empty(Empty::ConstPtr _){;}

void velocity(Velocity::ConstPtr v)
{
    vel.linear.x = v->linear.x;
    vel.linear.y = v->linear.y;
    vel.angular.z = v->angular.z;
}

void target(Target::ConstPtr tgt)
{
    double r, p, y;
    tf::Quaternion q;
    pose.pose = tgt->pose;
    start = !(stop = false);
    X = tgt->pose.position.x;
    Y = tgt->pose.position.y;
    tf::quaternionMsgToTF(tgt->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(r, p, y); YAW = y;
}

void position(Position::ConstPtr pos)
{
    x = pos->x * 1e-2; y = pos->y * 1e-2; yaw = pos->yaw;
}

int main(int argc, char* argv[])
{
    double t;
    Bool gyro;
    Cancel cancel;
    ros::init(argc, argv, NODE);
	ros::Time::init();
	ros::Rate sleeper(20);
	ros::NodeHandle handle;
    vel.linear.z = vel.angular.x = vel.angular.y = 0;
	ros::Publisher pub1 = handle.advertise<Velocity>("cmd_vel", 10);
	ros::Publisher pub2 = handle.advertise<Cancel>("move_base/cancel", 1);
    ros::Subscriber sub1 = handle.subscribe<Target>(goal, 1, target);
    ros::Subscriber sub2 = handle.subscribe<Empty>("loop", 10, empty);
    ros::Subscriber sub3 = handle.subscribe<Velocity>("vel", 10, velocity);
    ros::Subscriber sub4 = handle.subscribe<Position>("position", 10, position);
    ros::ServiceClient client = handle.serviceClient<Bool>("gyroscope");
    ros::service::waitForService("gyroscope");
	while(ros::ok())
	{
        vel.linear.x = 
        vel.linear.y = 
        vel.angular.z = 0;
		ros::spinOnce();
        if(start && !stop && SqrtL2(X-x, Y-y) < DX && std::fabs(yaw - YAW) < DY)
        {
            stop = true;
            pub2.publish(cancel);
            gyro.request.data = true;
            ros::Duration(0.1).sleep();
            client.call(gyro);
        }
        if(vel.linear.x || vel.linear.y || vel.angular.z)
        {
            t = NOW;
            if(gyro.request.data)
            {
                gyro.request.data = false;
                client.call(gyro);
            }
            pub1.publish(vel);
        }
		sleeper.sleep();
	}
	return 0;
}
