/* @author: YueLin */

#include "as.hpp"

#define INF 0xFFFFFFF
#define pow2(n) (n)*(n)

typedef std::vector<std::vector<int>> IntMatrix;

namespace pp
{
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
}

Points as(const cv::Mat& map, Point start, Point end)
{
    Points path = pp::keypoint(pp::a(map, start, end));
    return path.size()? pp::dijkstra(pp::graph(map, path), path): path;
}
