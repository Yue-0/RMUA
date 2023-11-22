/* @author: YueLin */

#include "ros/ros.h"

#include "sentry/Position.h"
#include "sentry/Positions.h"

#define INF 0xFFFFFFF
#define pow2(n) (n) * (n)
#define abs(n) (n > 0? n: -n)
#define sgn(n) (n > 0? 1: -1)
#define INIT ros::init(argc, argv, "aim")
#define distance2(x1, y1, x2, y2) pow2(x1 - x2) + pow2(y1 - y2)

using namespace sentry;

Position decision(int x, int y, int color, const Positions& robots)
{
    Position target;
    int d2, enemy = -1, m = INF;
    for(int robot = 0; robot < robots.len; robot++)
    {
        if(color * robots.id[robot] < 0)
        {
            d2 = distance2(x, y, robots.x[robot], robots.y[robot]);
            if(d2 < m)
            {
                m = d2; enemy = robot;
            }
            else if(d2 == m)
                enemy = std::max(abs(enemy), abs(robots.id[robot]));
        }
    }
    if(enemy < 0)
        target.yaw = target.id = target.x = target.y = 0;
    else
    {
        target.x = robots.x[enemy];
        target.y = robots.y[enemy];
        target.id = robots.id[enemy];
        target.yaw = robots.yaw[enemy];
    }
    return target;
}

int main(int argc, char* argv[])
{
    INIT;
    Positions robots;
    ros::Time::init();
    ros::NodeHandle nh;
    int x, y, color = 0;
    ros::Subscriber _1 = nh.subscribe<Position>(
        "position", 1, [&x, &y, &color](Position::ConstPtr pos)
        {
            x = pos->x; y = pos->y;
            if(!color) color = sgn(pos->id);
        }
    );
    ros::Subscriber _2 = nh.subscribe<Positions>(
        "sentry", 1, [&robots](Positions::ConstPtr pos){robots = *pos;}
    );
    ros::Publisher publisher = nh.advertise<Position>("enemy", 1);
    while(ros::ok())
    {
        ros::spinOnce();
        if(color && robots.len)
            publisher.publish(decision(x, y, color, robots));
    }
}
