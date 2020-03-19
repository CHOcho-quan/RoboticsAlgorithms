#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class GlobalObstacleMap {
 public:
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;
    int map_size_x; 
    int map_size_y;
    vector<int> obstacle_x;
    vector<int> obstacle_y;
    cv::Mat background;
    string w_name;

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
        w_name = name;

        background = cv::Mat(m_x, m_y, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        // TODO - Paint start & goal place & obstacles
    }

    void render(int s) {
        cv::imshow(w_name, background);
        cv::waitKey(s);
    }

    float heuristic(int x, int y)
    {
        return sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    }
};