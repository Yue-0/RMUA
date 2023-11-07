#include <cmath>

#include "tf/tf.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"

#include "navigation/Position.h"

#define VEL 1.0
#define RATE 100
#define SCALE 1e-2
#define abs std::fabs
#define ACC (1.0 / RATE)
#define sgn(x) (x > 0? 1: -1)
#define max(x, y) (x > y? x: y)
#define INIT ros::init(argc, argv, "vel")
#define distance2(x1, y1, x2, y2) pow2(x1 - x2) + pow2(y1 - y2)

#define Path nav_msgs::Path
#define Position navigation::Position
#define Velocity geometry_msgs::Twist

const double PI = std::acos(-1);

double pow2(double x)
{
    return x * x;
}

double clip(double v, double v0)
{
    double a = v - v0;
    if(abs(a) > ACC)
        v = v0 + ACC * sgn(a);
    if(abs(v) > VEL)
        v = VEL * sgn(v);
    return v;
}

int main(int argc, char* argv[])
{
    INIT;
    Path path;
    Velocity vel;
    Position position;
    ros::Time::init();
    ros::NodeHandle nh;
    ros::Rate sleeper(RATE);
    float cmd_x, cmd_y, cmd_z;
    ros::Subscriber _1 = nh.subscribe<Path>(
        "path", 1, [&path](Path::ConstPtr points){path = *points;}
    );
    ros::Subscriber _2 = nh.subscribe<Position>(
        "position", 1, [&position](Position::ConstPtr p){position = *p;}
    );
    ros::Subscriber _3 = nh.subscribe<Velocity>(
        "cmd_vel", 1, [&cmd_x, &cmd_y, &cmd_z](Velocity::ConstPtr cmd_vel)
        {
            cmd_x = cmd_vel->linear.x;
            cmd_y = cmd_vel->linear.y;
            cmd_z = cmd_vel->angular.z;
        }
    );
    vel.linear.x = vel.linear.y = vel.angular.z = 0;
    vel.linear.z = vel.angular.x = vel.angular.y = 0;
    ros::Publisher pub = nh.advertise<Velocity>("cmd_vel", 1);
    while(ros::ok())
    {
        ros::spinOnce();
        if(position.x && position.y && path.poses.size())
        {
            double yaw = position.yaw,
            dx = path.poses[1].pose.position.x - position.x * SCALE,
            dy = path.poses[1].pose.position.y - position.y * SCALE;
            double theta = yaw; // 暂时设为yaw，将来用辅助瞄准代替
            theta = tf::getYaw(path.poses[1].pose.orientation);  // 测试
            theta = std::atan2(0.7 - position.y * SCALE, 0.7 - position.x * SCALE);
            vel.angular.z = clip(
                abs(theta - yaw) > PI? yaw - theta: theta - yaw, cmd_z
            );
            if(abs(vel.angular.z) < ACC)
            {
                double sin = std::sin(theta), cos = std::cos(theta);
                vel.linear.x = cos * dx + sin * dy;
                vel.linear.y = cos * dy - sin * dx;
                vel.angular.z = 0;
            }
            else
            {
                // theta = yaw + vel.angular.z / RATE;
                // double i1 = (std::cos(yaw) - std::cos(theta)) / vel.angular.z;
                // double i2 = (std::sin(theta) - std::sin(yaw)) / vel.angular.z;
                // double i = pow2(i1) + pow2(i2);
                // vel.linear.x = (dx * i2 + dy * i1) / i;
                // vel.linear.y = (dy * i2 - dx * i1) / i;
                double sin = std::sin(yaw), cos = std::cos(yaw);
                vel.linear.x = cos * dx + sin * dy;
                vel.linear.y = cos * dy - sin * dx;
            }
            short sx = sgn(vel.linear.x), sy = sgn(vel.linear.y);
            float vx = abs(vel.linear.x), vy = abs(vel.linear.y);
            if(path.poses.size() > 2 && distance2(
                path.poses.back().pose.position.x,
                path.poses.back().pose.position.y,
                position.x * SCALE, position.y * SCALE
            ) > 0.5)
            {
                if(vx > max(vy, 1e-2))
                {
                    double scale = VEL / vx;
                    vel.linear.x = sx * VEL;
                    vel.linear.y *= scale;
                }
                else if(vy > max(vx, 1e-2))
                {
                    double scale = VEL / vy;
                    vel.linear.y = sy * VEL;
                    vel.linear.x *= scale;
                }
            }
            vel.linear.x = clip(vel.linear.x, cmd_x);
            vel.linear.y = clip(vel.linear.y, cmd_y);
        }
        else
        {
            vel.linear.x = vel.linear.y = vel.angular.z = 0;
        }
        if(
            vel.linear.x != cmd_x ||
            vel.linear.y != cmd_y ||
            vel.angular.z != cmd_z
        )
            pub.publish(vel);
        sleeper.sleep();
    }
    return 0;
}
