/* @author: YueLin */

#include <queue>
#include <vector>

#include "opencv2/opencv.hpp"

typedef cv::Point2i Point;
typedef std::vector<Point> Points;

namespace pp
{
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
}