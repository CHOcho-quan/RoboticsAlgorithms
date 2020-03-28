//
// Created by quan
// 2020.3.28
//
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ALGLIB/optimization.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class MinSnapOptimizer {
 public:
    int degree; // Degree of the fitting polynomial
    float timeT; // Required time of the whole trajectory
    std::string window_name;
    std::vector<int> Px; // Discrete trajectory points coordinate x
    std::vector<int> Py; // Discrete trajectory points coordinate y
    std::vector<float> coord;
    cv::Mat background;

    // Start state definition
    Eigen::Vector2d positionS;
    Eigen::Vector2d velocityS;
    Eigen::Vector2d accelarationS;

    // End state definition
    Eigen::Vector2d positionE;
    Eigen::Vector2d velocityE;
    Eigen::Vector2d accelarationE;

    MinSnapOptimizer() {}
    MinSnapOptimizer(int d, std::vector<int> x, std::vector<int> y, std::string w_name="Minimum Snap") : 
        degree(d), Px(x), Py(y), window_name(w_name) {
        coord = std::vector<float>(degree, 1.0);

        cv::Point pi, lpi;
        for (int i = 0;i < Px.size();i++) {
            if (i == 0) {
                pi = cv::Point(Px[i], Py[i]);
                continue;
            }
            lpi = pi;
            pi = cv::Point(Px[i], Py[i]);
            cv::line(background, pi, pi, cv::Scalar(255, 0, 0), 3);
        }

        cv::namedWindow(w_name, cv::WINDOW_NORMAL);
        cv::imshow(w_name, background);
    }

    void render(int s) {
        cv::imshow(window_name, background);
        cv::waitKey(s);
    }

    void MinSnapOptimize() {
        /**
         * Fitting Polynomial of the discrete path by :
         * sum(coord[i] * p^i)
        */
    }
};

int main()
{
    std::vector<int> x = {0, 1, 4, 7}, y = {0, 2, 4, 10};
    MinSnapOptimizer m(6, x, y);
    m.render(0);
}