//
// Created by quan
// 2020.3.15
//

#ifndef INCLUDE_ASTAR
#define INCLUDE_ASTAR
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

struct Node {
    /**
     * Node for A* planning
     * @x, y - the coordinate of current point
     * @cost - the f cost of the point computed by A*
    */
    int x;
    int y;
    float cost;
    Node *pre;
    Node(int x_c, int y_c, float c, Node *p=NULL) : x(x_c), y(y_c), cost(c), pre(p) {}
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
    string w_name;

    GlobalObstacleMap(int m_x, int m_y, int s_x, int s_y, int g_x, int g_y,
        vector<int> o_x, vector<int> o_y, string name="A*") {
        /**
         * @brief - Initialization of the Global Obstacle Map class
         * @param m_x, m_y - map size; s_x, s_y - start place; g_x, g_y - goal place
         * @param o_x, o_y - obstacle representation; name - window name of visualization
        */
        start_x = s_x;
        start_y = s_y;
        goal_x = g_x;
        goal_y = g_y;
        map_size_x = m_x;
        map_size_y = m_y;
        obstacle_x = o_x;
        obstacle_y = o_y;
        w_name = name;

        background = cv::Mat(m_x, m_y, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        annoteCell(s_x, s_y, START);
        annoteCell(g_x, g_y, GOAL);

        for (int i = 0;i < o_x.size();i++) annoteCell(o_x[i], o_y[i], OBSTACLE);
    }

    void render(int s) {
        /**
         * @brief - Rendering the visualization of the planner and map
         * @param s - waitKey parameter, second
        */
        cv::imshow(w_name, background);
        cv::waitKey(s);
    }

    CELLTYPE checkCell(int x, int y)
    {
        /**
         * @breif - Collision check base function
         * @param x, y - the checking place in the obstacle map
         * @return - return the cell type of the place inside the map
        */
        if (x < 0 || y < 0 || x >= map_size_x || y >= map_size_y) return OBSTACLE;
        if (x == start_x && y == start_y) return START;
        if (x == goal_x && y == goal_y) return GOAL;
        for (int i = 0;i < obstacle_x.size();i++) if (obstacle_x[i] == x && obstacle_y[i] == y) return OBSTACLE;
        return FREE; 
    }

    void annoteCell(int c_x, int c_y, CELLTYPE celltype) {
        /**
         * @breif - Visualization of the given cell type place in map
         * @param c_x, c_y - exact place in the map
         * @param celltype - given celltype of the place
        */
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
        /**
         * @brief - Heuristic function using in A*
         * @param x, y - exact place in the map
         * @return - heuristic value of the place, i.e. distance to goal
        */
        return sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    }
};

#endif