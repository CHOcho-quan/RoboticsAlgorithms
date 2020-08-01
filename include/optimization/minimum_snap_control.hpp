//
// Created by quan
// 2020.4.3
//
#ifndef INCLUDE_MINIMUM_SNAP
#define INCLUDE_MINIMUM_SNAP
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <math.h>
#include <ALGLIB/optimization.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace alglib;

#define DEBUG 1

class MinSnapOptimizer {
    /**
     * Robotics Trajectory Optimization - Minimum Snap
     * @MinSnapOptimizeMultiple - optimize the path from start to end by Minimum Snap
    */
 public:
    int degree; // Degree of the fitting polynomial
    double timeT; // Required time of the whole trajectory
    std::string window_name;
    std::vector<int> Px; // Discrete trajectory points coordinate x
    std::vector<int> Py; // Discrete trajectory points coordinate y
    std::vector<double> coord;
    cv::Mat background;
    int map_size_x;
    int map_size_y;

    MinSnapOptimizer() {}
    MinSnapOptimizer(int d, std::vector<int> x, std::vector<int> y, std::string w_name="Minimum Snap",
        int m_x = 1000, int m_y = 1000, double time_spec = 4) : 
        degree(d), Px(x), Py(y), window_name(w_name), map_size_x(m_x), map_size_y(m_y), timeT(time_spec) {
        coord = std::vector<double>(degree, 1.0);
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
        /**
         * @brief - Rendering the visualization of the planner and map
         * @param s - waitKey parameter, second
        */
        cv::imshow(window_name, background);
        cv::waitKey(s);
    }

    bool MinSnapOptimizeMultiple(int part) {
        /**
         * @brief - Optimize composited trajectory planned by discrete method
         * @param part - number of the different trajectory 
        */
        if (part <= 0) {
            std::cout << "What are you doing?"  << std::endl;
            return false;
        }

        std::vector<double> time_stamp;
        double unit = timeT / Px.size();
        for (double stamp = timeT / Px.size();stamp <= timeT;stamp += timeT / Px.size()) time_stamp.push_back(stamp);

        real_1d_array x, y;
        int step = Px.size() / part;
        for (int i = 0;i < part;i++) {
            // Current Points to be generating trajectory
            std::vector<int> c_x, c_y;
            std::vector<double> time_;
            std::vector<std::vector<double>> constraintx, constrainty;
            
            if (i == part - 1) {
                for (int j = std::max(0, i * step - 1);j < Px.size();j++) {
                    // Deal with extra constraints
                    if (j == i * step - 1) {
                        std::vector<double> tmp;
                        for (int k = 0;k < degree;k++) tmp.push_back(k * pow(time_stamp[j], k - 1));
                        double xnum = 0;
                        for (int k = 0;k < degree;k++) xnum += k * x[k] * pow(time_stamp[j], k - 1);
                        tmp.push_back(xnum);
                        constraintx.push_back(tmp);

                        tmp.clear();
                        for (int k = 0;k < degree;k++) tmp.push_back(k * pow(time_stamp[j], k - 1));
                        double ynum = 0;
                        for (int k = 0;k < degree;k++) ynum += k * y[k] * pow(time_stamp[j], k - 1);
                        tmp.push_back(ynum);
                        constrainty.push_back(tmp);
                    }

                    c_x.push_back(Px[j]);
                    c_y.push_back(Py[j]);
                    time_.push_back(time_stamp[j]);
                }
            }
            else {
                for (int j = std::max(0, i * step - 1);j < (i+1) * step;j++) {
                    // Deal with extra constraints
                    if (j == i * step - 1) {
                        std::vector<double> tmp;
                        for (int k = 0;k < degree;k++) tmp.push_back(k * pow(time_stamp[j], k - 1));
                        double xnum = 0;
                        for (int k = 0;k < degree;k++) xnum += k * x[k] * pow(time_stamp[j], k - 1);
                        tmp.push_back(xnum);
                        constraintx.push_back(tmp);

                        tmp.clear();
                        for (int k = 0;k < degree;k++) tmp.push_back(k * pow(time_stamp[j], k - 1));
                        double ynum = 0;
                        for (int k = 0;k < degree;k++) ynum += k * y[k] * pow(time_stamp[j], k - 1);
                        tmp.push_back(ynum);
                        constrainty.push_back(tmp);
                    }

                    c_x.push_back(Px[j]);
                    c_y.push_back(Py[j]);
                    time_.push_back(time_stamp[j]);
                }
            }

            double last_time_spec = time_.at(0), time_spec = time_.at(time_.size()-1);
            if (DEBUG) std::cout << last_time_spec << ' ' << time_spec << std::endl;
            MinSnapOptimizeSingle(last_time_spec, time_spec, unit, constraintx, constrainty, c_x, c_y, x, y);
        }
    }

    void MinSnapOptimizeSingle(double last_time_spec, double time_spec, double unit, std::vector<std::vector<double>> constraintx,
        std::vector<std::vector<double>> constrainty, std::vector<int> Px, std::vector<int> Py, real_1d_array &x, real_1d_array &y) {
        /**
         * @brief - Optimize single trajectory planned by discrete method
         * @param constraintx, constrainty - extra constraint on connecting points
         * @param time_spec, unit - time stamp of the trajectory and unit time stamp
        */
        real_2d_array qua_term, cx, cy;
        real_1d_array linear_term, p0, scale;
        integer_1d_array ct;
        ae_int_t kx, ky;

        // Intializing Quadratic Term
        double *qua_term_;
        qua_term_ = new double[degree * degree];
        for (int i = 0;i < degree;i++) {
            for (int j = 0;j < degree;j++) {
                if (i <= 3 || j <= 3) qua_term_[i * degree + j] = 0;
                else qua_term_[i * degree + j] = ((pow(time_spec, i + j - 7) - pow(last_time_spec, i + j - 7))
                    * std::tgamma(i+1) * std::tgamma(j+1) / std::tgamma(i-3) / std::tgamma(j-3) / (i + j - 7));
            }
        }
        qua_term.setcontent(ae_int_t(degree), ae_int_t(degree), qua_term_);

        // Initializing Linear Term
        double *linear_term_;
        linear_term_ = new double[degree];
        // Hint: DO NOT USE MEMSET 
        for (int i = 0;i < degree;i++) linear_term_[i] = 0;
        linear_term.setcontent(ae_int_t(degree), linear_term_);

        // Initializing start state & sacle
        double *p0_, *scale_;
        p0_ = new double[degree];
        scale_ = new double[degree];
        for (int i = 0;i < degree;i++) p0_[i] = 1;
        for (int i = 0;i < degree;i++) scale_[i] = 1;
        p0.setcontent(ae_int_t(degree), p0_);
        scale.setcontent(ae_int_t(degree), scale_);

        // Initializing Equality Constraint
        ae_int_t *ct_;
        int total = Px.size() + constraintx.size();
        ct_ = new ae_int_t[total];
        for (int i = 0;i < total;i++) ct_[i] = ae_int_t(0);
        ct.setcontent(ae_int_t(total), ct_);

        // Equality Constraint for x
        double *cx_;
        cx_ = new double[total * (degree + 1)];
        for (int i = 0;i < Px.size();i++) {
            for (int j = 0;j < degree + 1;j++) {
                if (j == degree) cx_[i * (degree + 1) + j] = Px[i];
                else cx_[i * (degree + 1) + j] = pow(i * unit + last_time_spec, j);
            }
        }

        // Extra Constraint for x - derivative continuous
        for (int i = 0;i < constraintx.size();i++) {
            for (int j = 0;j < constraintx[i].size();j++) {
                cx_[Px.size() * (degree + 1) + i * (degree + 1) + j] = constraintx[i][j];
            }
        }
        cx.setcontent(ae_int_t(total), ae_int_t(degree+1), cx_);

        // Equality Constraint for y
        double *cy_;
        cy_ = new double[total * (degree + 1)];
        for (int i = 0;i < Py.size();i++) {
            for (int j = 0;j < degree + 1;j++) {
                if (j == degree) cy_[i * (degree + 1) + j] = Py[i];
                else cy_[i * (degree + 1) + j] = pow(i * unit + last_time_spec, j);
            }
        }

        // Extra Constraint for y - derivative continuous
        for (int i = 0;i < constrainty.size();i++) {
            for (int j = 0;j < constrainty[i].size();j++) {
                cy_[Py.size() * (degree + 1) + i * (degree + 1) + j] = constrainty[i][j];
            }
        }
        cy.setcontent(ae_int_t(total), ae_int_t(degree+1), cy_);

        // Initializing kx ky
        kx = ae_int_t(total);
        ky = ae_int_t(total);

        MinSnapQPOptimize(qua_term, linear_term, p0, scale, cx, ct, kx, x);
        MinSnapQPOptimize(qua_term, linear_term, p0, scale, cy, ct, ky, y);

        // Now annote the background for the result
        for (double t = last_time_spec;t < time_spec;t+=0.001) {
            double xnum = 0, ynum = 0;
            for (int j = 0;j < degree;j++) {
                xnum += x[j] * pow(t, j);
                ynum += y[j] * pow(t, j);
            }
            cv::circle(background, cv::Point(int(xnum), int(ynum)), 1, cv::Scalar(0, 0, 255), -1);
        }
        render(0);
    }

    void MinSnapQPOptimize(real_2d_array qua_term, real_1d_array linear_term, real_1d_array p0,
        real_1d_array scale, const real_2d_array c, const integer_1d_array ct, const ae_int_t k, real_1d_array &x) {
        /**
         * @brief - QP Optimizer given by ALGLIB
        */
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
        if (DEBUG) printf("%d\n", int(rep.terminationtype)); 
        if (DEBUG) printf("%s\n", x.tostring(2).c_str()); 
    }

    void MinSnapOptimizeTest() {
        /**
         * @brief - A test version of trajectory generation by minimum snap
        */
        cv::Mat bbk = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
        std::vector<int> xx = {58, 47, 210, 477}, yy = {69, 249, 405, 109};

        for (int i = 0;i < xx.size();i++) {
            if (i == 0) {
                cv::circle(bbk, cv::Point(xx[i], yy[i]), 3, cv::Scalar(0, 0, 255), -1);
                continue;
            }

            cv::circle(bbk, cv::Point(xx[i], yy[i]), 3, cv::Scalar(0, 0, 255), -1);
            cv::line(bbk, cv::Point(xx[i-1], yy[i-1]), cv::Point(xx[i], yy[i]), cv::Scalar(255, 0, 0), 3);
        }

        // Test 6 version for x
        real_2d_array qua_term = "[[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 2304, 23040],"
                                  "[0, 0, 0, 0, 23040, 307200]]";
        real_1d_array linear_term = "[0, 0, 0, 0, 0, 0]";
        real_1d_array p0 = "[1, 1, 1, 1, 1, 1]";
        real_1d_array scale = "[1, 1, 1, 1, 1, 1]";
        real_2d_array c;

        // Note: our time starts from 1.0 instead of 0 to avoid singularity
        double _c[] = {1, 1, 1, 1, 1, 1, 58,
                       1, 2, 4, 8, 16, 32, 47,
                       1, 3, 9, 27, 81, 243, 210,
                       1, 4, 16, 64, 256, 1024, 477};
        c.setcontent(ae_int_t(4), ae_int_t(7), _c);
        integer_1d_array ct = "[0, 0, 0, 0]";
        ae_int_t k = 4;
        real_1d_array x;

        MinSnapQPOptimize(qua_term, linear_term, p0, scale, c, ct, k, x);
       
        // for y
        real_2d_array qua_term2 = "[[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 0, 0],"
                                  "[0, 0, 0, 0, 2304, 23040],"
                                  "[0, 0, 0, 0, 23040, 307200]]";
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

        MinSnapQPOptimize(qua_term2, linear_term2, p02, scale2, c2, ct2, k2, y);

        // Now annote the background for the result
        for (double t = 1;t < 4;t+=0.001) {
            int xnum = x[0] + x[1] * t + x[2] * pow(t, 2) + x[3] * pow(t, 3) + x[4] * pow(t, 4) + x[5] * pow(t, 5);
            int ynum = y[0] + y[1] * t + y[2] * pow(t, 2) + y[3] * pow(t, 3) + y[4] * pow(t, 4) + y[5] * pow(t, 5);
            cv::circle(bbk, cv::Point(xnum, ynum), 1, cv::Scalar(0, 0, 255), -1);
        }

        cv::imshow("Test", bbk);
        cv::waitKey(0);
        cv::resize(bbk, bbk, cv::Size(300, 300));
        cv::imwrite("../results/optimization/minimum_snapQP.png", bbk);
    }
};

#endif
