/* @author: YueLin */

#include <cmath>
#include <queue>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"

#include "sentry/Position.h"

#define DIS 10
#define BACK 100
#define INF 0xFFFFFFFF

typedef std::vector<int> Vector;
typedef sentry::Position Position;
typedef std::vector<Vector> Matrix;
typedef nav_msgs::OccupancyGrid Map;

namespace search
{
    Position point2position(int x, int y)
    {
        Position position;
        position.x = x;
        position.y = y;
        position.id = 0;
        position.yaw = 0;
        return position;
    }

    Position bfs(const Map& map, int x0, int y0)
    {
        std::queue<int> queue;
        int h = map.info.height, w = map.info.width;
        Matrix graph(map.info.height, Vector(map.info.width, 0));
        Matrix visited(map.info.height, Vector(map.info.width, 0));
        Matrix distance(map.info.height, Vector(map.info.width, INF));
        for(int y = 0; y < h; y++)
        {
            int z = y * w;
            for(int x = 0; x < w; x++)
                graph[y][x] = map.data[z + x] / 100;
        }
        visited[y0][x0] = -1;
        distance[y0][x0] = 0;
        queue.push(y0 * w + x0);
        while(!queue.empty())
        {
            int z0 = queue.front();
            int x0 = z0 % w, y0 = z0 / w;
            int dis = distance[y0][x0];
            if(dis > DIS)
                return point2position(x0, y0);
            visited[y0][x0] = 1; queue.pop();
            for(int dx = -1; dx <= 1; dx++)
            {
                int x = x0 + dx;
                if(x < 0 || x >= w)
                    continue;
                for(int dy = -1; dy <= 1; dy++)
                {
                    int y = y0 + dy;
                    if(y < 0 || y >= h)
                        continue;
                    if(!visited[y][x])
                    {
                        visited[y][x] = -1;
                        queue.push(y * w + x);
                        distance[y][x] = (graph[y][x]? 0: dis + 1);
                    }
                }
            }
        }
        return point2position(x0, y0);
    }

    Position back(const Map& map, int x0, int y0, double yaw)
    {
        int width = map.info.width, height = map.info.height;
        double sin = std::sin(yaw), cos = std::cos(yaw);
        int x = x0 - BACK * sin, y = y0 - BACK * cos;
        y = std::min(std::max(y, 0), height - 1);
        x = std::min(std::max(x, 0), width - 1);
        if(map.data[y * width + x])
            return bfs(map, x, y);
        return point2position(x, y);
    }
}
