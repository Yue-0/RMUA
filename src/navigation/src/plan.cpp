/* @author: YueLin */

#include <cmath>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "bfs.hpp"
#include "plan.hpp"
#include "sentry/plan.h"
#include "sentry/Points.h"
#include "sentry/Position.h"
#include "sentry/Positions.h"

#define SELF 5
#define DELTA 30
#define SCALE 1e-2
#define EXPANSION 40
#define INIT ros::init(argc, argv, "plan")
#define swap(a, b) {a += b; b = a - b; a -= b;}

typedef nav_msgs::Path Path;
typedef sentry::Points PointCloud;
typedef sentry::Position Position;
typedef sentry::Positions Positions;
typedef nav_msgs::OccupancyGrid Map;

cv::Point3f goal;
const double PI = std::acos(-1);
bool planning = false, pub = false;

int max(int x, int y)
{
    return x > y? x: y;
}

int rad2deg(double rad)
{
    return 180 * rad / PI;
}

double subtraction(double rad1, double rad2)
{
    float sub = std::fabs(rad1 - rad2);
    return sub > PI? 2 * PI - sub: sub;
}

bool srv(sentry::plan::Request& req, sentry::plan::Response& res)
{
    if(planning = req.x + req.y)
    {
        goal.x = req.x;
        goal.y = req.y;
    }
    else
        pub = true;
    return res.result = true;
}

Path plan(const cv::Mat& map, Point start, Point end, int* self)
{
    Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    if(!map.at<uchar>(end.y, end.x)) return path;
    for(Point point: as(map, start, Point(end.x, end.y)))
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = path.header.stamp;
        pose.pose.position.x = point.x * SCALE;
        pose.pose.position.y = point.y * SCALE;
        pose.header.frame_id = path.header.frame_id;
        path.poses.push_back(pose);
    }
    // if(!path.poses.size())
    // {
    //     *self += SELF;
    //     *self = max(*self, SELF << 2);
    //     return path;
    // }
    // *self = max(*self - 1, SELF);
    return path;
}

int main(int argc, char* argv[])
{
    INIT;
    Map cost;
    int self = SELF;
    cv::Mat MAP, map;
    bool init = false;
    Position position;
    ros::Time::init();
    ros::NodeHandle nh;
    Positions positions;
    ros::Rate sleeper(100);
    ros::Publisher planner = nh.advertise<Path>("path", 1);
    ros::Publisher costmap = nh.advertise<Map>("costmap", 1);
    ros::ServiceServer _0 = nh.advertiseService("plan", srv);
    ros::Subscriber _1 = nh.subscribe<Map>(
        "map", 1, [&cost, &init, &MAP](Map::ConstPtr data)
        {
            if(!init)
            {
                init = true; cost = *data; MAP = cv::Mat(
                    data->info.height, data->info.width, CV_8UC1, 0xFF
                );
                cv::rectangle(MAP, Point(0, 0), Point(
                    data->info.width, data->info.height
                ), 0, EXPANSION);
            }
        }
    );
    ros::Subscriber _2 = nh.subscribe<Position>(
        "position", 1, [&position](Position::ConstPtr pos){position = *pos;}
    );
    ros::Subscriber _3 = nh.subscribe<Positions>(
        "sentry", 1, [&positions](Positions::ConstPtr pos){positions = *pos;}
    );
    ros::Subscriber _4 = nh.subscribe<PointCloud>(
        "scan", 1, [&](PointCloud::ConstPtr pc)
        {
            map = MAP.clone();
            for(int i = 0; i < pc->len; i++)
            {
                int x = std::round(pc->x[i] / SCALE);
                int y = std::round(pc->y[i] / SCALE);
                cv::rectangle(map, Point(
                    x - EXPANSION, y - EXPANSION
                ), Point(
                    x + EXPANSION, y + EXPANSION
                ), 0, -1);
            }
            for(short i = 0; i < positions.len; i++)
            {
                if(positions.id[i] == position.id)
                    continue;
                cv::rectangle(map, Point(
                    positions.x[i] - 3 * EXPANSION / 2,
                    positions.y[i] - 3 * EXPANSION / 2
                ), Point(
                    positions.x[i] + 3 * EXPANSION / 2,
                    positions.y[i] + 3 * EXPANSION / 2
                ), 0, -1);
            }
            // cv::circle(map, Point(position.x, position.y), self, 0xFF, -1);
            int xs = position.x;//baselink在map的坐标
            int ys = position.y;
            xs = std::min(std::max(xs, 0), map.cols - 1);
            ys = std::min(std::max(ys, 0), map.rows - 1);
            if (!map.at<uchar>(start.y,start.x))
            {
                pp::int2D p = pp::bfs(map, start.x, start.y, 0);//step;
                int x = p.first, y = p.second;
                int xss = start.x, yss = start.y;
                if(xss > x) swap(x, xss);
                if(yss > y) swap(y, yss);
               
                cv::rectangle(
                    map, cv::Point2i(xss, yss), cv::Point2i(x + 1, y + 1), 0xFF, -1
                );
            }
            for(int y = 0; y < map.rows; y++)
            {
                int z = y * cost.info.width;
                for(int x = 0; x < map.cols; x++)
                    cost.data[z + x] = map.at<uchar>(y, x)? 0: 100;
            }
        }
    );
    while(ros::ok())
    {
        ros::spinOnce();
        if(init)
        {
            if(goal.x + goal.y < SELF)
            {
                goal.x = goal.y = 0;
                planning = !(pub = true);
            }
            if(planning)
            {
                float z = position.yaw;
                int x = position.x, y = position.y;
                if(max(std::abs(x - goal.x), std::abs(y - goal.y)) > DELTA)
                    planner.publish(plan(
                        map, Point(x, y), Point(goal.x, goal.y), &self
                    ));
                else
                    goal.x = goal.y = planning = !(pub = true);
            }
            if(pub)
            {
                Path path;
                pub = false;
                path.header.frame_id = "map";
                path.header.stamp = ros::Time::now();
                planner.publish(path);
            }
            cost.header.stamp = ros::Time::now();
            costmap.publish(cost);
        }
        sleeper.sleep();
    }
    return 0;
}
