//
// Created by quan
// 2020.3.20
//
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

#define DEBUG 0

class GlobalObstacleMap {
    /**
     * Global Obstacle Map for PRM Planning
     * @brief Containing the map information of obstacle, start and goal
     * Visualize the result
    */
 public:
    cv::Mat background;
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;
    int map_size_x;
    int map_size_y;
    string w_name;

    GlobalObstacleMap(int s_x, int s_y, int g_x, int g_y, string map_path="../maps/map_1.png", string window_name="Robotics") {
        start_x = s_x;
        start_y = s_y;
        goal_x = g_x;
        goal_y = g_y;
        w_name = window_name;

        cv::namedWindow(w_name, cv::WINDOW_NORMAL);
        background = cv::imread(map_path, cv::IMREAD_COLOR);
        cv::MatSize map_size = background.size;
        map_size_x = map_size[1];
        map_size_y = map_size[0];
        cout << "MAP SIZE: " << map_size_x << ' ' << map_size_y << endl;

        cv::circle(background, cv::Point(s_x, s_y), 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(background, cv::Point(g_x, g_y), 5, cv::Scalar(255, 0, 0), -1);
    }

    bool checkCell(int x, int y) {
        if (x < 0 || y < 0 || x >= map_size_x || y >= map_size_y) return true;
        int b = (int)*(background.data + background.step[0] * y + x * background.step[1]);
        int g = (int)*(background.data + background.step[0] * y + x * background.step[1] + background.elemSize1());
        int r = (int)*(background.data + background.step[0] * y + x * background.step[1] + background.elemSize1() * 2);
        if (DEBUG) cout << b << ' ' << g << ' ' << r << endl;
        return (b == 0) && (g == 0) && (r == 0);
    }

    bool checkPath(cv::Point p1, cv::Point p2) {
        float step_x = p1.x - p2.x, step_y = p1.y - p2.y;
        float step_length = sqrt(pow(step_x, 2) + pow(step_y, 2));
        step_x /= step_length;
        step_y /= step_length;

        float init_x = p1.x, init_y = p1.y;
        while ((init_x - p2.x) * step_x >= 0 && (init_y - p2.y) * step_y >= 0) {
            init_x -= step_x;
            init_y -= step_y;
            // cout << cvRound(init_x) << ' ' << cvRound(init_y) << endl;
            if (checkCell(cvRound(init_x), cvRound(init_y))) return false;
        }

        return true;
    }

    float heuristic(int x, int y) {
        return sqrt(pow(x - goal_x, 2) + pow(y - goal_y, 2));
    }

    void render(int s=0) {
        cv::imshow(w_name, background);
        cv::waitKey(s);
    }
};

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
            cout << current->x << ' ' << current->y << endl;
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
                    cout << points[i].x << ' ' << points[i].y << endl;

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