#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class GlobalObstacleMap {
    public:
        vector<int> obstacle_x;
        vector<int> obstacle_y;
        int start_x;
        int start_y;
        int goal_x;
        int goal_y;
        int map_size_x;
        int map_size_y;
        Mat background;
        string a_name;

        enum CELLTYPE{
            FREE=1,
            ROBOT,
            START,
            GOAL,
            OBSTACLE
        };

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

            background = Mat(m_x, m_y, CV_8UC3, cv::Scalar(255, 255, 255));
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            annoteCell(s_x, s_y, START);
            annoteCell(g_x, g_y, GOAL);

            for (int i = 0;i < o_x.size();i++) annoteCell(o_x[i], o_y[i], OBSTACLE);
        }

        void render() {
            cv::imshow(a_name, background);
            cv::waitKey(0);
        }

        void annoteCell(int c_x, int c_y, CELLTYPE celltype) {
            switch (celltype) {
                case FREE: break;
                case ROBOT: {
                    cv::rectangle(background, cv::Rect(c_x, c_y, 1, 1), cv::Scalar(0, 255, 255), -1);
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
            }
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
    m.render();
}