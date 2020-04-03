//
// Created by quan
// 2020.3.28
//

#include "minimum_snap_control.hpp"

int main()
{
    std::vector<int> x = {54, 587, 696, 750, 710, 687, 598, 372}, y = {78, 124, 245, 431, 682, 762, 858, 788};
    MinSnapOptimizer m(6, x, y);
    // m.MinSnapOptimizeTest();
    m.MinSnapOptimizeMultiple(2);
    cv::resize(m.background, m.background, cv::Size(300, 300));
    cv::imwrite("../results/optimization/minimum_snap2.png", m.background);
}