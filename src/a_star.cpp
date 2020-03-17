#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

#define DEBUG 0

enum CELLTYPE{
    FREE=1,
    ROBOT,
    START,
    GOAL,
    OBSTACLE,
    PATH
};

class GlobalObstacleMap {
    /**
     * Global Obstacle Map class
     * @brief Containing the map information of obstacle, start and goal
     * Visualize the result
    */
 public:
    vector<int> obstacle_x;
    vector<int> obstacle_y;
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;
    int map_size_x;
    int map_size_y;
    cv::Mat background;
    string a_name;

    GlobalObstacleMap(int m_x, int m_y, int s_x, int s_y, int g_x, int g_y,
        vector<int> o_x, vector<int> o_y, string name="Robotics") {
        start_x = s_x;
        start_y = s_y;
        goal_x = g_x;
        goal_y = g_y;
        map_size_x = m_x;
        map_size_y = m_y;
        obstacle_x = o_x;
        obstacle_y = o_y;
        a_name = name;

        background = cv::Mat(m_x, m_y, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        annoteCell(s_x, s_y, START);
        annoteCell(g_x, g_y, GOAL);

        for (int i = 0;i < o_x.size();i++) annoteCell(o_x[i], o_y[i], OBSTACLE);
    }

    void render(int s) {
        cv::imshow(a_name, background);
        cv::waitKey(s);
    }

    CELLTYPE checkCell(int x, int y)
    {
        if (x == start_x && y == start_y) return START;
        if (x == goal_x && y == goal_y) return GOAL;
        for (int i = 0;i < obstacle_x.size();i++) if (obstacle_x[i] == x && obstacle_y[i] == y) return OBSTACLE;
        return FREE; 
    }

    void annoteCell(int c_x, int c_y, CELLTYPE celltype) {
        switch (celltype) {
            case FREE: break;
            case ROBOT: {
                cv::rectangle(background, cv::Rect(c_x, c_y, 1, 1), cv::Scalar(20, 255, 20), -1);
                break;
            }
            case START: {
                cv::rectangle(background, cv::Rect(c_x, c_y, 1, 1), cv::Scalar(255, 60, 0), -1);
                break;
            }
            case GOAL: {
                cv::rectangle(background, cv::Rect(c_x, c_y, 1, 1), cv::Scalar(0, 0, 255), -1);
                break;
            }
            case OBSTACLE: {
                cv::rectangle(background, cv::Rect(c_x, c_y, 1, 1), cv::Scalar(0, 0, 0), -1);
                break;
            }
            case PATH: {
                cv::rectangle(background, cv::Rect(c_x, c_y, 1, 1), cv::Scalar(128, 0, 128), -1);
            }
        }
    }

    float heuristic(int x, int y)
    {
        return sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    }
};

struct Node {
    int x;
    int y;
    float cost;
    Node *pre;
    Node(int x_c, int y_c, float c, Node *p=NULL) : x(x_c), y(y_c), cost(c), pre(p) {}
};

class AstarPlanner {
 private:
    vector<vector<int> > close_list;
    vector<vector<float> > path_cost;
    vector<Node> motions = { Node(1, 0, 1.0), Node(0, 1, 1.0), Node(0, -1, 1.0), Node(-1, 0, 1.0), 
                            Node(1, 1, sqrt(2)), Node(1, -1, sqrt(2)), Node(-1, -1, sqrt(2)), Node(-1, 1, sqrt(2))};

 public:
    void calcPath(Node *end, GlobalObstacleMap om)
    {
        while (end != NULL)
        {
            om.annoteCell(end->x, end->y, PATH);
            end = end->pre;
        }
    }

    void AstarPlanning(GlobalObstacleMap om)
    {
        for (int i = 0;i < om.map_size_x;i++) {
            vector<int> tmp;
            vector<float> tmp_c;
            for (int j = 0;j < om.map_size_y;j++) {
                tmp.push_back(0);
                tmp_c.push_back(2147483647);
            }
            path_cost.push_back(tmp_c);
            close_list.push_back(tmp);
        }

        auto cmp = [](const Node *a, const Node *b) { return a->cost > b->cost; };
        priority_queue<Node*, vector<Node*>, decltype(cmp)> open(cmp);

        Node *start;
        start = new Node(om.start_x, om.start_y, 0.0);
        path_cost[om.start_x][om.start_y] = 0.0;
        open.push(start);

        bool ending_flag = false;
        Node *final;
        while (!open.empty())
        {
            // Smallest element in open list, put it into close list
            Node *current = open.top();
            open.pop();
            if (close_list[current->x][current->y]) continue;
            if (DEBUG) cout << current->cost << endl;
            close_list[current->x][current->y] = 1;

            om.annoteCell(current->x, current->y, ROBOT);
            om.render(0);

            // For every motion, get its next Node
            if (DEBUG) cout << "-----------next----------" << endl;
            for (auto motion : motions)
            {
                int next_x = current->x + motion.x, next_y = current->y + motion.y;
                float current_path_cost = path_cost[current->x][current->y];

                Node *next;
                if (om.checkCell(next_x, next_y) == GOAL) {
                    next = new Node(next_x, next_y, 0.0, current);
                    final = next;
                    ending_flag = true;
                    break;
                }
                if (close_list[next_x][next_y] || om.checkCell(next_x, next_y) == OBSTACLE) continue;

                float next_cost = current_path_cost + om.heuristic(next_x, next_y) + motion.cost;
                next = new Node(next_x, next_y, next_cost, current);
                path_cost[next_x][next_y] = min(path_cost[next_x][next_y], current_path_cost + motion.cost);
                open.push(next);
                if (DEBUG) cout << next->cost << endl;
            }
            if (DEBUG) cout << "-----------next----------" << endl;

            if (ending_flag) break;
        }

        calcPath(final, om);
        om.render(0);
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
    GlobalObstacleMap m(50, 50, 5, 5, 45, 45, o_x, o_y);
    AstarPlanner planner;
    planner.AstarPlanning(m);
    m.render(0);
}