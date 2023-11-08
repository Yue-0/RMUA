/* @author: YueLin */

#include <cmath>

#include "tf/tf.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#include "plan.hpp"
#include "sentry/plan.h"
#include "sentry/Position.h"
#include "sentry/Positions.h"

#define DELTA 30
#define SCALE 1e-2
#define EXPANSION 38
#define INIT ros::init(argc, argv, "plan")

typedef nav_msgs::Path Path;
typedef sentry::Position Position;
typedef sentry::Positions Positions;
typedef nav_msgs::OccupancyGrid Map;
typedef sensor_msgs::LaserScan Laser;

int self;
Map cost;
cv::Mat MAP, map;
cv::Point3f goal;
Position position;
Positions positions;
tf::StampedTransform transform;
const double PI = std::acos(-1);
bool planning = false, init = false, pub = false;

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
        goal.z = req.yaw;
    }
    else
        pub = true;
    return res.result = true;
}

Path plan(const cv::Mat& map, Point start, cv::Point3f end)
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
    if(!path.poses.size())
    {
        self += 2;
        return path;
    }
    self = max(self - 1, 5);
    geometry_msgs::Pose pose2, pose1 = path.poses[0].pose;
    for(int p = 1; p < path.poses.size(); p++)
    {
        pose2 = path.poses[p].pose;
        pose1.orientation = tf::createQuaternionMsgFromYaw(
            std::atan2(
                pose2.position.y - pose1.position.y,
                pose2.position.x - pose1.position.x
            )
        );
        path.poses[p - 1].pose = pose1; pose1 = pose2;
    }
    pose2.orientation = tf::createQuaternionMsgFromYaw(end.z);
    path.poses[path.poses.size() - 1].pose = pose2;
    return path;
}

void sp(Positions::ConstPtr pos)
{
    positions = *pos;
}

void scan(Laser::ConstPtr laser)
{
    map = MAP.clone();
    double angle = laser->angle_min,
    x = transform.getOrigin().x() / SCALE,
    y = transform.getOrigin().y() / SCALE,
    yaw = tf::getYaw(transform.getRotation());
    for(double d: laser->ranges)
    {
        if(d <= laser->range_max && d >= laser->range_min)
        {
            d /= SCALE;
            int x0 = std::round(x + d * std::cos(yaw + angle));
            int y0 = std::round(y + d * std::sin(yaw + angle));
            cv::rectangle(map, Point(
                x0 - EXPANSION, y0 - EXPANSION
            ), Point(
                x0 + EXPANSION, y0 + EXPANSION
            ), 0, -1);
        }
        angle += laser->angle_increment;
    }
    for(short i = 0; i < positions.len; i++)
    {
        if(positions.id[i] == position.id)
            continue;
        cv::rectangle(map, Point(
            positions.x[i] - 2 * EXPANSION,
            positions.y[i] - 2 * EXPANSION
        ), Point(
            positions.x[i] + 2 * EXPANSION,
            positions.y[i] + 2 * EXPANSION
        ), 0, -1);
    }
    cv::circle(map, Point(position.x, position.y), self, 0xFF, -1);
    for(int y = 0; y < map.rows; y++)
        for(int x = 0; x < map.cols; x++)
            cost.data[y * cost.info.width + x] = map.at<uchar>(y, x)? 0: 100;
}

void locate(Position::ConstPtr pos)
{
    position = *pos;
}

void initinalize(Map::ConstPtr map)
{
    if(init) return;
    cv::Mat image(map->info.height, map->info.width, CV_8UC1);
    for(int y = 0; y < map->info.height; y++)
        for(int x = 0; x < map->info.width; x++)
            image.at<uchar>(y, x) = map->data[y * map->info.width + x] == 0;
    cv::erode(0xFF * image, MAP, cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(EXPANSION << 1, EXPANSION << 1)
    ));
    self = 5;
    cost = *map;
    init = true;
}

int main(int argc, char* argv[])
{
    INIT;
    ros::Time::init();
    ros::NodeHandle nh;
    ros::Rate sleeper(100);
    tf::TransformListener listener;
    ros::Publisher planner = nh.advertise<Path>("path", 1);
    ros::Publisher costmap = nh.advertise<Map>("costmap", 1);
    ros::ServiceServer _0 = nh.advertiseService("plan", srv);
    ros::Subscriber _1 = nh.subscribe<Laser>("scan", 1, scan);
    ros::Subscriber _2 = nh.subscribe<Map>("map", 1, initinalize);
    ros::Subscriber _3 = nh.subscribe<Positions>("sentry", 1, sp);
    ros::Subscriber _4 = nh.subscribe<Position>("position", 1, locate);
    listener.waitForTransform("map", "laser", ros::Time(0), ros::Duration(10));
    while(ros::ok())
    {
        listener.lookupTransform("map", "laser", ros::Time(0), transform);
        ros::spinOnce();
        if(init)
        {
            if(planning)
            {
                float z = position.yaw;
                int x = position.x, y = position.y;
                if(max(
                    rad2deg(subtraction(z, goal.z)) << 1,
                    max(std::abs(x - goal.x), std::abs(y - goal.y))
                ) > DELTA)
                    planner.publish(plan(map, Point(x, y), goal));
                else
                    planning = !(pub = true);
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
