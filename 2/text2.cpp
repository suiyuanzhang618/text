#include <iostream>
#include <vector>
#include <queue>
#include <iomanip>
#include <cmath>
#include <algorithm> 

using namespace std;

// 定义坐标结构体，捆绑xy。
class Point {
    public:
    int x;
    int y;
};

// 计算两点之间的曼哈顿距离，abs（）是一个绝对值函数，引用节约了内存，const保证不变
int manhattanDistance(const Point& p1, const Point& p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

// 寻找膨胀地图中两点的最短路径，
void shortestPathWithInflation(vector<vector<int>>& grid, double t, const Point& start, const Point& end) {
    int n = grid.size();
    int m = grid[0].size();

    // 定义四个方向的偏移量：上、下、左、右
    vector<Point> directions = { {0, 1}, {0, -1}, {1, 0}, {-1, 0} };

    // 创建膨胀地图
    vector<vector<double>> inflatedGrid(n, vector<double>(m, 0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++i) {
            inflatedGrid[i][j] = grid[i][j];
            for (const auto& dir : directions) {
                int newX = i + dir.x;
                int newY = j + dir.y;
                if (newX >= 0 && newX < n && newY >= 0 && newY < m) {
                    inflatedGrid[newX][newY] += t * grid[i][j];
                }//也就是这里把那些大的数视为障碍，然后少碰到它们
            }
        }
    }

    // 定义优先队列，存储待处理的节点及其到起点的距离
    priority_queue<pair<double, Point>, vector<pair<double, Point>>, greater<pair<double, Point>>> pq;
    // 定义距离数组，存储从起点到每个点的最短距离
    vector<vector<double>> dist(n, vector<double>(m, numeric_limits<double>::infinity()));
    // 起点到自身的距离为0
    dist[start.x][start.y] = 0;
    pq.push({ 0, start });

    // 主循环，处理优先队列中的节点
    while (!pq.empty()) {
        double d = pq.top().first;
        Point current = pq.top().second;
        pq.pop();

        if (current.x == end.x && current.y == end.y) {
            break;
        }

        if (d > dist[current.x][current.y]) {
            continue;
        }

        for (const auto& dir : directions) {
            int newX = current.x + dir.x;
            int newY = current.y + dir.y;
            if (newX >= 0 && newX < n && newY >= 0 && newY < m) {
                double newD = d + inflatedGrid[newX][newY];
                if (newD < dist[newX][newY]) {
                    dist[newX][newY] = newD;
                    pq.push({ newD, {newX, newY} });
                }
            }
        }
    }

    // 构建最短路径
    vector<Point> path;
    if (dist[end.x][end.y]!= numeric_limits<double>::infinity()) {
        Point current = end;
        while (current.x!= start.x || current.y!= start.y) {
            path.push_back(current);
            double minD = numeric_limits<double>::infinity();
            Point minNeighbor;
            for (const auto& dir : directions) {
                int newX = current.x + dir.x;
                int newY = current.y + dir.y;
                if (newX >= 0 && newX < n && newY >= 0 && newY < m && dist[newX][newY] < minD) {
                    minD = dist[newX][newY];
                    minNeighbor = {newX, newY};
                }
            }
            current = minNeighbor;
        }
        path.push_back(start);
        reverse(path.begin(), path.end()); // 使用reverse函数反转路径元素顺序
    }

    // 输出最短路径及每个点的代价和总代价
    cout << "最短路径:" << endl;
    for (const auto& p : path) {
        cout << "(" << p.x << ", " << p.y << ") ";
    }
    cout << endl;

    cout << "路径上每个点的代价:" << endl;
    for (const auto& p : path) {
        cout << "位置(" << p.x << ", " << p.y << "): " << setprecision(2) << fixed << dist[p.x][p.y] << endl;
    }

    cout << "总代价: " << setprecision(2) << fixed << dist[end.x][end.y] << endl;
}

int main() {
    //行数和列数
    int n, m;
    cin >> n >> m;
//二维数组表示地图，t是膨胀系数，然后初始化了grid地图。
    vector<vector<int>> grid(n, vector<int>(m));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            cin >> grid[i][j];
        }
    }

    double t;
    cin >> t;
//用结构体定义了起，终点。
    Point start, end;
    cin >> start.x >> start.y >> end.x >> end.y;
//找出最短路径
    shortestPathWithInflation(grid, t, start, end);

    return 0;
}