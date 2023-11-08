/* @author: YueLin */

#include <cmath>
#include <queue>
#include <vector>

#include "tf/tf.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#include "navigation/plan.h"
#include "navigation/Position.h"
#include "navigation/Positions.h"

#define DX 25
#define DELTA 30
#define SCALE 1e-2
#define EXPANSION 35
#define INF 0xFFFFFFF
#define pow2(n) (n)*(n)
#define INIT ros::init(argc, argv, "plan")

#define Point cv::Point2i
#define Path nav_msgs::Path
#define Points std::vector<Point>
#define Map nav_msgs::OccupancyGrid
#define Laser sensor_msgs::LaserScan
#define Position navigation::Position
#define Positions navigation::Positions
#define IntMatrix std::vector<std::vector<int>>

int self;
Map cost;
cv::Mat MAP, map;
cv::Point3f goal;
Position position;
Positions positions;
tf::StampedTransform transform;
bool planning = false, init = false, pub = false;

class Node
{
    public:
        int x, y, g, h, f;
        Node(int x = 0, int y = 0, int g = 0, int h = 0):
            x(x), y(y), g(g), h(h), f(g + h) {}
};

bool operator<(const Node& node1, const Node& node2)
{
    return node1.f > node2.f;
}

Points a(const cv::Mat& map, Point start, Point end)
{
    int X = map.cols, Y = map.rows;
    std::vector<Node> nodes(X * Y);
    std::priority_queue<Node> queue;
    Node node(start.x, start.y, 0, 0);
    std::vector<int> parent(X * Y, -1);
    std::vector<bool> visited(X * Y, false);
    int index = X * node.y + node.x;
    visited[index] = true;
    nodes[index] = node;
    queue.push(node);
    while(!queue.empty())
    {
        node = queue.top(); queue.pop();
        index = X * node.y + node.x;
        if(node.x == end.x && node.y == end.y)
        {
            Points path;
            while(parent[index] != -1)
            {
                path.push_back(Point(node.x, node.y));
                node = nodes[parent[index]];
                index = X * node.y + node.x;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        for(int dy = -1; dy <= 1; dy++)
        {
            int y = node.y + dy;
            if(y < 0 || y >= Y) continue;
            for(int dx = -1; dx <= 1; dx++)
            {
                int x = node.x + dx;
                int idx = X * y + x;
                if(x < 0 || x >= X || !map.at<uchar>(y, x) || visited[idx])
                    continue;
                int g = node.g + 1;
                int h = std::abs(x - end.x) + std::abs(y - end.y);
                Node next(x, y, g, h);
                visited[idx] = true;
                parent[idx] = index;
                nodes[idx] = next;
                queue.push(next);
            }
        }
    }
    return Points();
}

Points keypoint(const Points& path)
{
    Points keypoints;
    int n = path.size();
    if(!n) return keypoints;
    keypoints.push_back(path[0]);
    Point p0 = path[0], p1 = path[1], p2 = path[2];
    for(int i = 3; i < path.size(); i++)
    {
        if((p1.x - p0.x) * (p2.y - p1.y) - (p1.y - p0.y) * (p2.x - p1.x))
            keypoints.push_back(p1);
        p0 = p1; p1 = p2; p2 = path[i];
    }
    keypoints.push_back(p2);
    return keypoints;
}

IntMatrix graph(const cv::Mat& map, const Points& points)
{
    int n = points.size();
    IntMatrix g(n, std::vector<int>(n, 0));
    for(int i = 0; i < n; i++)
    {
        Point p1 = points[i];
        for(int j = i + 1; j < n; j++)
        {
            Point p2 = points[j];
            bool connected = true;
            if(j - i > 1)
            {
                cv::LineIterator line(map, p1, p2);
                for(int k = 0; k < line.count; k++, line++)
                {
                    Point p = line.pos();
                    if(!map.at<uchar>(p.y, p.x))
                    {
                        connected = false; break;
                    }
                }
            }
            if(connected)
                g[i][j] = pow2(p1.x - p2.x) + pow2(p1.y - p2.y);
        }
    }
    return g;
}

Points dijkstra(const IntMatrix& g, const Points& points)
{
    int n = g.size();
    std::vector<int> dist(n, INF);
    std::vector<int> parent(n, -1);
    std::vector<bool> visited(n, false);
    std::priority_queue<std::pair<int, int>> queue;
    queue.push(std::make_pair(dist[0] = 0, 0));
    while(!queue.empty())
    {
        int u = queue.top().second; queue.pop();
        if(u == n - 1)
        {
            Points path;
            path.push_back(points[n - 1]);
            while(u = parent[u])
                path.push_back(points[u]);
            path.push_back(points[0]);
            std::reverse(path.begin(), path.end());
            return path;
        }
        for(int v = u + 1; v < n; v++)
            if(!visited[v] && g[u][v])
            {
                int d = dist[u] + g[u][v];
                if(d < dist[v])
                {
                    dist[v] = d;
                    parent[v] = u;
                    queue.push(std::make_pair(d, v));
                }
            }
        visited[u] = true;
    }
    return Points();
}

Points as(const cv::Mat& map, Point start, Point end)
{
    Points path = keypoint(a(map, start, end));
    return path.size()? dijkstra(graph(map, path), path): path;
}

bool srv(navigation::plan::Request& req, navigation::plan::Response& res)
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
        self++;
        return path;
    }
    self = 5;
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
            // cv::circle(map, Point(x0, y0), EXPANSION, 0, -1);
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
    // cv::rectangle(
    //     map, Point(position.x - 5, position.y - 5), 
    //     Point(position.x + 5, position.y + 5), 0xFF, -1
    // );
    cv::circle(map, Point(position.x, position.y), self, 0xFF, -1);
    for(int y = 0; y < map.rows; y++)
        for(int x = 0; x < map.cols; x++)
            cost.data[y * cost.info.width + x] = map.at<uchar>(y, x)? 0: 100;
}

void locate(Position::ConstPtr pos)
{
    position = *pos;
}

void sentry(Positions::ConstPtr pos)
{
    positions = *pos;
}

void initinalize(Map::ConstPtr map)
{
    if(init) return;
    cv::Mat image(map->info.height, map->info.width, CV_8UC1);
    for(int y = 0; y < map->info.height; y++)
        for(int x = 0; x < map->info.width; x++)
            image.at<uchar>(y, x) = map->data[y * map->info.width + x] == 0;
    cv::erode(0xFF * image, MAP, cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(EXPANSION, EXPANSION)
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
    ros::Subscriber _3 = nh.subscribe<Positions>("sentry", 1, sentry);
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
                int x = position.x, y = position.y;
                if(std::max(std::abs(x - goal.x), std::abs(y - goal.y)) > DELTA)
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
