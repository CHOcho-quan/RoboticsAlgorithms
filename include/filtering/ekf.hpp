//
// Created by quan
// 2020.4.11
//
#include <iostream>
#include <math.h>
#include <random>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define dT 0.1
#define DEBUG 1

class NonlinearDynamicSystem {
    /**
     * Formulation of a linear dynamic system model, simple 2D version
     * Xt = A * Xt-1 + B * u + e1
     * Zt = H * Xt + e2
     * Xt = [x, y, v, theta], u = [v, w]
     * where e1 ~ N(0, sigma1) and e2 ~ N(0, sigma2)
    */
 private:
    Eigen::Matrix4d jacobA(Eigen::Vector2d u) {
        /**
         * @brief get the jacobian matrix of A (since there are nonlinearity only in A)
         * @param state - current state vector
         * @param u - current control input
         * @return jacobian matrix of A
        */
       Eigen::Matrix4d JA = Eigen::Matrix4d::Identity();

       auto v = u(0), w = u(1), theta = cur_state(3);

       JA(0, 2) = dT * cos(theta);
       JA(0, 3) = -dT * sin(theta) * v;
       JA(1, 2) = dT * sin(theta);
       JA(1, 3) = dT * cos(theta) * v;

       return JA;
    }
 public:
    Eigen::Matrix4d A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd H;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;
    Eigen::Vector4d cur_state;

    NonlinearDynamicSystem() {}
    NonlinearDynamicSystem &operator=(NonlinearDynamicSystem &lds) {
        A = lds.A;
        B = lds.B;
        H = lds.H;
        Q = lds.Q;
        R = lds.R;
    }

    NonlinearDynamicSystem(const NonlinearDynamicSystem &lds) {
        A = lds.A;
        B = lds.B;
        H = lds.H;
        Q = lds.Q;
        R = lds.R;
    }

    NonlinearDynamicSystem(double cov1, double cov2) {
        H = Eigen::MatrixXd(2, 4);
        B = Eigen::MatrixXd(4, 2);

        Q(0, 0) = cov1 * cov1;
        Q(1, 1) = cov1 * cov1;
        Q(2, 2) = (1.0/180 * M_PI) * (1.0/180 * M_PI);
        Q(3, 3) = cov1 * cov1;
        R(0, 0) = cov2 * cov2;
        R(1, 1) = cov2 * cov2;
    }

    void setCurState(Eigen::Vector4d state_vec) {
        /**
         * @brief set current state of nonlinear dynamic system
         * @param state_vec - given state vector
        */
        cur_state(0) = state_vec(0);
        cur_state(1) = state_vec(1);
        cur_state(2) = state_vec(2);
        cur_state(3) = state_vec(3);
    }

    Eigen::Matrix4d getJacobA(Eigen::Vector2d u) { return jacobA(u); }

    Eigen::Vector2d getObservation(Eigen::Vector4d input, Eigen::Vector2d error) {
        /**
         * @brief get current observation given input
         * @param input - current state input
         * @param error - the error caused by uncertainty - Gaussian
         * @return observation vector
        */
        Eigen::Vector2d result = H * input + R * error;

        return result;
    }

    Eigen::Vector4d getNextState(Eigen::Vector4d last_state, Eigen::Vector2d control_input, Eigen::Vector4d error) {
        /**
         * @brief get next state given current state and control input
         * @param last_state - current state
         * @return next state vector
        */
        auto theta = cur_state(3);
        B << dT * cos(theta), 0,
             dT * sin(theta), 0,
             1, 0,
             0, dT;

        cur_state = A * last_state + B * control_input + Q * error;

        return cur_state;
    }

    Eigen::Vector4d getReckonedNextState(Eigen::Vector4d last_state, Eigen::Vector2d control_input) {
        /**
         * @brief getting Dead Reckoned state to compare with
         * @return reckoned state vector
        */
         return A * last_state + B * control_input; 
    }
};