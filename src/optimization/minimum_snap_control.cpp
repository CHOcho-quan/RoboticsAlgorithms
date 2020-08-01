//
// Created by quan
// 2020.3.28
//

#include "minimum_snap_control.hpp"

int main()
{
    std::vector<int> x = {54, 157, 266, 280, 400, 450, 408, 307}, y = {78, 88, 100, 230, 435, 505, 600, 701};
    MinSnapOptimizer m(6, x, y);
    // m.MinSnapOptimizeTest();
    m.MinSnapOptimizeMultiple(2);
    // cv::resize(m.background, m.background, cv::Size(300, 300));
    cv::imwrite("../results/optimization/minimum_snap4.png", m.background);
}