//
// Created by quan
// 2020.3.20
//
#include "prm.h"

struct Node {
    /**
     * Node for A* planning
     * @x, y - the coordinate of current point
     * @cost - the f cost of the point computed by A*
    */
    int x;
    int y;
    float cost;
    int index;
    Node *pre;
    Node(int x_c, int y_c, float c, int ind, Node *p=NULL) : x(x_c), y(y_c), cost(c), index(ind), pre(p) {}
};

class PRMPlanner {
     /**
     * Robotics PRM Planner
     * @calcPath - calculating the final path and draw it on Obstacle map
     * @PRMPlanning - planning the path from start to end by PRM method
    */
 private:
    int rand_num;
    int neighbor_ro;
    vector<int> close_list;
    vector<float> path_cost;
 public:
    PRMPlanner(int r_n = 50, int n_r = 200) : rand_num(r_n), neighbor_ro(n_r) {
        close_list = vector<int>(rand_num, 0);
        path_cost = vector<float>(rand_num, 2147483647);
    }

    float distance(cv::Point p1, cv::Point p2) {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    void PRMPlanning(GlobalObstacleMap m) {
        srand(time(NULL));
        vector<vector<int>> AdjacentMatrix(rand_num, vector<int>(rand_num, 0));
        vector<cv::Point> points;
        points.push_back(cv::Point(m.start_x, m.start_y));
        points.push_back(cv::Point(m.goal_x, m.goal_y));

        // Generate Random Points
        int tmp_rand_num = 2;
        while (tmp_rand_num < rand_num) {
            int tmp_x = rand() % m.map_size_x;
            int tmp_y = rand() % m.map_size_y;

            if (m.checkCell(tmp_x, tmp_y)) continue;
            else tmp_rand_num++;
            cv::Point current(tmp_x, tmp_y);
            points.push_back(current);
            cv::circle(m.background, current, 5, cv::Scalar(30, 50, 128), -1);

            // Line up if possible
            for (int i = 0;i < tmp_rand_num - 1;i++) {
                if (distance(current, points[i]) < neighbor_ro) {
                    if (DEBUG) cout << "YES IN NEIGHBOR" << endl;
                    // Check if can ligature
                    if (m.checkPath(current, points[i])) {
                        AdjacentMatrix[i][tmp_rand_num - 1] = 1;
                        AdjacentMatrix[tmp_rand_num - 1][i] = 1;
                        cv::line(m.background, current, points[i], cv::Scalar(0, 120, 0), 3);
                    }
                }
                else continue;
            }
        }

        // Show construction of the G(V, E)
        m.render(0);
        
        // Next, do A* on G(V, E)
        auto cmp = [](const Node *a, const Node *b) { return a->cost > b->cost; };
        priority_queue<Node*, vector<Node*>, decltype(cmp)> open(cmp);

        Node *start, *final;
        bool flag = false;
        start = new Node(m.start_x, m.start_y, 0.0, 0);
        path_cost[start->index] = 0.0;
        open.push(start);
        while (!open.empty()) {
            Node *current = open.top();
            if (DEBUG) cout << current->x << ' ' << current->y << endl;
            open.pop();
            if (close_list[current->index]) continue;
            else close_list[current->index] = 1;

            for (int i = 0;i < rand_num;i++) {
                if (AdjacentMatrix[current->index][i]) {
                    Node *next;
                    if (i == 1) {
                        cout << "FIND PATH!" << endl;
                        final = new Node(points[i].x, points[i].y, 0.0, i, current);
                        flag = true;
                        break;
                    }
                    if (DEBUG) cout << points[i].x << ' ' << points[i].y << endl;

                    if (close_list[i]) continue;
                    path_cost[i] = min(path_cost[i], 
                        distance(points[current->index], points[i]) + path_cost[i]);
                    float cost = distance(points[current->index], points[i]) + path_cost[i] 
                                + m.heuristic(points[i].x, points[i].y);
                    next = new Node(points[i].x, points[i].y, cost, i, current);

                    open.push(next);
                }
            }

            if (flag) break;
        }

        calcPath(final, m);
        m.render(0);
    }

    void calcPath(Node *goal, GlobalObstacleMap m)
    {
        while (goal != NULL)
        {
            Node *tmp = goal->pre;
            if (tmp != NULL) cv::line(m.background, cv::Point(goal->x, goal->y), cv::Point(tmp->x, tmp->y), cv::Scalar(128, 0, 128), 3);
            goal = goal->pre;
        }
    }
};

int main()
{
    GlobalObstacleMap om(10, 10, 450, 450);
    PRMPlanner prm;
    prm.PRMPlanning(om);
}