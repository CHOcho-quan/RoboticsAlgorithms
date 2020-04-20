//
// Created by quan
// 2020.4.19
//
#include "a_star.h"

struct LPNode {
    int x;
    int y;
    float k1;
    float k2;
    LPNode *pre;
    LPNode(int x_c, int y_c, float c1, float c2, LPNode *p=NULL) : x(x_c), y(y_c), k1(c1), k2(c2), pre(p) {}
};

class LPAstarPlanner {
 private:
    static bool cmp(const LPNode *a, const LPNode *b)
    {
        if (a->k1 == b->k1) return a->k2 < b->k2;
        return a->k1 < b->k1; 
    }
    vector<vector<float>> g;
    vector<vector<float>> rhs;
    vector<LPNode*> pq;
    vector<Node> motions = { Node(1, 0, 1.0), Node(0, 1, 1.0), Node(0, -1, 1.0), Node(-1, 0, 1.0), 
                            Node(1, 1, sqrt(2)), Node(1, -1, sqrt(2)), Node(-1, -1, sqrt(2)), Node(-1, 1, sqrt(2))};
    map<pair<int, int>, pair<int, int>> path;

 public:
    void calcPath(LPNode *end, GlobalObstacleMap om)
    {
        /**
         * @brief - calculate the final path finded by LPA*
         * @param goal - goal place; m - obstacle map
        */
        pair<int, int> e(end->x, end->y);
        while (e.first != om.start_x && e.second != om.start_y)
        {
            om.annoteCell(e.first, e.second, PATH);
            cout << e.first << " YYYY " << e.second << endl;
            e = path[e];
        }
        om.annoteCell(e.first, e.second, PATH);
    }

    float k1(GlobalObstacleMap m, int x, int y) { return min(g[x][y], rhs[x][y]) + m.heuristic(x, y); }

    float k2(int x, int y) { return min(g[x][y], rhs[x][y]); }

    void pushNode(LPNode *n) {
        pq.push_back(n);
        sort(pq.begin(), pq.end(), cmp);
    }

    void removeNode(LPNode *n) {
        for (auto iter = pq.begin();iter != pq.end();iter++) {
            LPNode *tmp = *iter;
            if (tmp->x == n->x && tmp->y == n->y) {
                pq.erase(iter);
                break;
            }
        }
    }

    void updateVertex(GlobalObstacleMap m, LPNode *n) {
        if (m.start_x == n->x && m.start_y == n->y) return ;//n;

        // Calculate RHS value
        float mini = 2147483647;
        int xx, yy;
        for (auto motion : motions) {
            int nx = n->x + motion.x, ny = n->y + motion.y;
            if (m.checkCell(nx, ny) == OBSTACLE) continue;
            if (g[nx][ny] != 2147483647) {
                float costs = g[nx][ny] + motion.cost;
                if (costs < mini) {
                    mini = costs;
                    xx = nx;
                    yy = ny;
                }
            }
        }
        rhs[n->x][n->y] = mini;

        removeNode(n);
        n->k1 = k1(m, n->x, n->y);
        n->k2 = k2(n->x, n->y);
        pair<int, int> original(n->x, n->y);
        pair<int, int> par(xx, yy);
        path[original] = par;

        if (g[n->x][n->y] != rhs[n->x][n->y]) pushNode(n);
    }

    void LPAstarPlanning(GlobalObstacleMap m) {
        // Initialization
        for (int i = 0;i < m.map_size_x;i++) {
            vector<float> tmpg;
            vector<float> tmpr;
            for (int j = 0;j < m.map_size_y;j++) {
                tmpg.push_back(2147483647);
                tmpr.push_back(2147483647);
            }

            g.push_back(tmpg);
            rhs.push_back(tmpr);
        }

        rhs[m.start_x][m.start_y] = 0;
        LPNode *final = NULL, *start = new LPNode(m.start_x, m.start_y, 
            k1(m, m.start_x, m.start_y), k2(m.start_x, m.start_y));
        pushNode(start);

        while (pq.size() > 0 && (rhs[m.goal_x][m.goal_y] == 2147483647 || 
            k2(m.goal_x, m.goal_y) > k2(pq[0]->x, pq[0]->y) || rhs[m.goal_x][m.goal_y] != g[m.goal_x][m.goal_y])) {
            LPNode *cur = pq[0];
            pq.erase(pq.begin());

            int cx = cur->x, cy = cur->y;
            if (g[cx][cy] > rhs[cx][cy]) {
                g[cx][cy] = rhs[cx][cy];
                for (auto motion : motions) {
                    int nx = cx + motion.x, ny = cy + motion.y;
                    if (m.checkCell(nx, ny) == OBSTACLE) continue;
                    LPNode* next = new LPNode(nx, ny, k1(m, nx, ny), k2(nx, ny), cur);
                    updateVertex(m, next);
                    if (nx == m.goal_x && ny == m.goal_y) final = next;
                }
            }
            else {
                cout << "BAD" << endl;
                g[cx][cy] = 2147483647;
                updateVertex(m, cur);
                for (auto motion : motions) {
                    int nx = cx + motion.x, ny = cy + motion.y;
                    if (m.checkCell(nx, ny) == OBSTACLE) continue;
                    LPNode* next = new LPNode(nx, ny, k1(m, nx, ny), k2(nx, ny), cur);
                    updateVertex(m, next);
                }
            }

            if (final != NULL) break;
        }

        calcPath(final, m);
    }
};

int main()
{
    vector<int> o_x, o_y;
    for (int i = 0;i < 50;i++) {
        o_x.push_back(0);
        o_y.push_back(i);
    }
    for (int i = 0;i < 50;i++) {
        o_x.push_back(i);
        o_y.push_back(0);
    }
    for (int i = 0;i < 50;i++) {
        o_x.push_back(i);
        o_y.push_back(49);
    }
    for (int i = 0;i < 50;i++) {
        o_x.push_back(49);
        o_y.push_back(i);
    }
    for (int i = 0;i < 26;i++) {
        o_x.push_back(i);
        o_y.push_back(15);
    }
    for (int i = 0;i < 26;i++) {
        o_x.push_back(50 - i);
        o_y.push_back(35);
    }
    GlobalObstacleMap m(50, 50, 5, 5, 45, 45, o_x, o_y, "LPA*");
    LPAstarPlanner planner;
    planner.LPAstarPlanning(m);
    m.render(0);
    cv::resize(m.background, m.background, cv::Size(200, 200));
    cv::imwrite("../results/planning/lpastar.png", m.background);
}