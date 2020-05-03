//
// Created by quan
// 2020.3.28
//

#include "minimum_snap_control.hpp"

int main()
{
    std::vector<int> x = {54, 250, 588, 210, 754, 477, 909, 910}, y = {78, 78, 176, 555, 435, 200, 177, 178};
    MinSnapOptimizer m(6, x, y);
    // m.MinSnapOptimizeTest();
    m.MinSnapOptimizeMultiple(2);
    cv::resize(m.background, m.background, cv::Size(300, 300));
    cv::imwrite("../results/optimization/minimum_snap4.png", m.background);
}