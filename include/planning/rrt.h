//
// Created by quan
// 2020.3.18
//
#ifndef INCLUDE_RRT
#define INCLUDE_RRT
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
     * Global Obstacle Map for RRT Planning
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

    GlobalObstacleMap(int s_x, int s_y, int g_x, int g_y, string map_path="../maps/map_3.png", string window_name="RRT") {
        /**
         * @brief - Initialization of the Global Obstacle Map class
         * @param m_x, m_y - map size; s_x, s_y - start place; g_x, g_y - goal place
         * @param o_x, o_y - obstacle representation; name - window name of visualization
        */
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
        /**
         * @breif - Collision check base function
         * @param x, y - the checking place in the obstacle map
         * @return - return the cell type of the place inside the map
        */
        if (x < 0 || y < 0 || x >= map_size_x || y >= map_size_y) return true;
        int b = (int)*(background.data + background.step[0] * y + x * background.step[1]);
        int g = (int)*(background.data + background.step[0] * y + x * background.step[1] + background.elemSize1());
        int r = (int)*(background.data + background.step[0] * y + x * background.step[1] + background.elemSize1() * 2);
        if (DEBUG) cout << b << ' ' << g << ' ' << r << endl;
        return (b == 0) && (g == 0) && (r == 0);
    }

    // If not convex shapes or big expand steps, use this as collision check
    bool checkPath(cv::Point p1, cv::Point p2) {
        /**
         * @breif - Collision check function along a path
         * @param p1, p2 - two points defining the checking path
         * @return - return if collapsed
        */
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
        /**
         * @brief - Heuristic function using in A*
         * @param x, y - exact place in the map
         * @return - heuristic value of the place, i.e. distance to goal
        */
        return sqrt(pow(x - goal_x, 2) + pow(y - goal_y, 2));
    }

    void render(int s=0) {
        /**
         * @brief - Rendering the visualization of the planner and map
         * @param s - waitKey parameter, second
        */
        cv::imshow(w_name, background);
        cv::waitKey(s);
    }
};

#endif