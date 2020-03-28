//
// Created by quan
// 2020.3.28
//
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <ALGLIB/optimization.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace alglib;

class MinSnapOptimizer {
 public:
    int degree; // Degree of the fitting polynomial
    float timeT; // Required time of the whole trajectory
    std::string window_name;
    std::vector<int> Px; // Discrete trajectory points coordinate x
    std::vector<int> Py; // Discrete trajectory points coordinate y
    std::vector<float> coord;
    cv::Mat background;
    int map_size_x;
    int map_size_y;

    // Start state definition
    Eigen::Vector2d positionS;
    Eigen::Vector2d velocityS;
    Eigen::Vector2d accelarationS;

    // End state definition
    Eigen::Vector2d positionE;
    Eigen::Vector2d velocityE;
    Eigen::Vector2d accelarationE;

    MinSnapOptimizer() {}
    MinSnapOptimizer(int d, std::vector<int> x, std::vector<int> y, std::string w_name="Minimum Snap",
        int m_x = 500, int m_y = 500, float time_spec = 4) : 
        degree(d), Px(x), Py(y), window_name(w_name), map_size_x(m_x), map_size_y(m_y), timeT(time_spec) {
        coord = std::vector<float>(degree, 1.0);
        background = cv::Mat(m_x, m_y, CV_8UC3, cv::Scalar(255, 255, 255));

        for (int i = 0;i < Px.size();i++) {
            if (i == 0) {
                cv::circle(background, cv::Point(Px[i], Py[i]), 3, cv::Scalar(0, 0, 255), -1);
                continue;
            }

            cv::circle(background, cv::Point(Px[i], Py[i]), 3, cv::Scalar(0, 0, 255), -1);
            cv::line(background, cv::Point(Px[i-1], Py[i-1]), cv::Point(Px[i], Py[i]), cv::Scalar(255, 0, 0), 3);
        }

        cv::namedWindow(w_name, cv::WINDOW_NORMAL);
    }

    void render(int s) {
        cv::imshow(window_name, background);
        cv::waitKey(s);
    }

    void MinSnapOptimizeTest() {
        float time_spec = timeT / Px.size();

        // Test 6 version for x
        real_2d_array qua_term = "[[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 2304, 4608],"
                                  "[0, 0, 0, 0, 4608, 307200]]";
        real_1d_array linear_term = "[0, 0, 0, 0, 0, 0]";
        real_1d_array p0 = "[1, 1, 1, 1, 1, 1]";
        real_1d_array scale = "[1, 1, 1, 1, 1, 1]";
        real_2d_array c;
        double _c[] = {1, 1, 1, 1, 1, 1, 58,
                       1, 2, 4, 8, 16, 32, 47,
                       1, 3, 9, 27, 81, 243, 210,
                       1, 4, 16, 64, 256, 1024, 477};
        c.setcontent(ae_int_t(4), ae_int_t(7), _c);
        integer_1d_array ct = "[0, 0, 0, 0]";
        ae_int_t k = 4;
        real_1d_array x;

        // QP
        minqpstate state;
        minqpreport rep;
        minqpcreate(6, state);
        minqpsetquadraticterm(state, qua_term);
        minqpsetlinearterm(state, linear_term);
        minqpsetstartingpoint(state, p0);
        minqpsetlc(state, c, ct, k);
        minqpsetscale(state, scale);

        minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
        minqpoptimize(state);
        minqpresults(state, x, rep);
        printf("%d\n", int(rep.terminationtype)); 
        printf("%s\n", x.tostring(2).c_str()); 

        // for y
        real_2d_array qua_term2 = "[[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 2304, 4608],"
                                  "[0, 0, 0, 0, 4608, 307200]]";
        real_1d_array linear_term2 = "[0, 0, 0, 0, 0, 0]";
        real_1d_array p02 = "[1, 1, 1, 1, 1, 1]";
        real_1d_array scale2 = "[1, 1, 1, 1, 1, 1]";
        real_2d_array c2;
        double _c2[] = {1, 1, 1, 1, 1, 1, 69,
                       1, 2, 4, 8, 16, 32, 249,
                       1, 3, 9, 27, 81, 243, 405,
                       1, 4, 16, 64, 256, 1024, 109};
        c2.setcontent(ae_int_t(4), ae_int_t(7), _c2);
        integer_1d_array ct2 = "[0, 0, 0, 0]";
        ae_int_t k2 = 4;
        real_1d_array y;

        // QP
        minqpstate state2;
        minqpreport rep2;
        minqpcreate(6, state2);
        minqpsetquadraticterm(state2, qua_term2);
        minqpsetlinearterm(state2, linear_term2);
        minqpsetstartingpoint(state2, p02);
        minqpsetlc(state2, c2, ct2, k2);
        minqpsetscale(state2, scale2);

        minqpsetalgobleic(state2, 0.0, 0.0, 0.0, 0);
        minqpoptimize(state2);
        minqpresults(state2, y, rep2);
        printf("%d\n", int(rep2.terminationtype)); 
        printf("%s\n", y.tostring(2).c_str()); 

        // Now annote the background for the result
        for (float t = 1;t < 4;t+=0.001) {
            int xnum = x[0] + x[1] * t + x[2] * pow(t, 2) + x[3] * pow(t, 3) + x[4] * pow(t, 4) + x[5] * pow(t, 5);
            int ynum = y[0] + y[1] * t + y[2] * pow(t, 2) + y[3] * pow(t, 3) + y[4] * pow(t, 4) + y[5] * pow(t, 5);
            cv::circle(background, cv::Point(xnum, ynum), 1, cv::Scalar(0, 0, 255), -1);
        }
        cv::imwrite("../results/minimum_snapQP.png", background);
    }
};

int main()
{
    std::vector<int> x = {58, 47, 210, 477}, y = {69, 249, 405, 109};
    MinSnapOptimizer m(6, x, y);
    m.MinSnapOptimizeTest();
    m.render(0);
}