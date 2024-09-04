#include <queue>
#include <vector>
#include <utility>
#include "opencv2/opencv.hpp"

#define INF 0xFFFFFFF

namespace pp
{
    typedef unsigned int uint;
    typedef std::pair<int, int> int2D;

    int2D bfs(const cv::Mat& map, int xs, int ys, int d_thr)
    {
        int2D t;
        std::queue<int2D> q;
        int h =map.rows, w = map.cols;
        std::vector<std::vector<uint>>
        dist(h, std::vector<uint>(w, INF));
        std::vector<std::vector<bool>>
        visited(h, std::vector<bool>(w, false));

       
        xs = std::min(std::max(0, xs), w-1);
        ys = std::min(std::max(0, ys), h-1);

        dist[ys][xs] = 0;
        visited[ys][xs] = true;
        q.push(std::make_pair(xs,ys));

        while(!q.empty())
        {
            t = q.front();
            q.pop();
            int x = t.first;
            int y = t.second;
            if(dist[y][x] > d_thr)
            {
                break;
            }
            for(int dx = -1;dx <= 1; dx++)
            {
                int xx = x + dx;
                if( xx < 0|| xx >= w )
                continue;
                for(int dy = -1;dy <= 1; dy++)
                {
                    int yy = y + dy;
                    if( yy < 0|| yy >= h )
                    continue;
                    if(!visited[yy][xx])
                    {
                        visited[yy][xx] = true;
                        q.push(std::make_pair(xx,yy));
                        dist[yy][xx] = map.at<uchar>(yy,xx) ? dist[y][x]+1 :0;
                    }
                }
            }

        }
        return t;
    
         
    }
}
