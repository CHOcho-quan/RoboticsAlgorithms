//
// Created by quan
// 2020.4.6
//
#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define DEBUG 1

class LinearDynamicSystem
{
public:
    /**
     * Formulation of a linear dynamic system model, simple 2D version
     * Xt = A * Xt-1 + B * u + e1
     * Zt = C * Xt + e2
     * Xt = [x, y, Vx, Vy], u = [ax, ay]
     * where e1 ~ N(0, sigma1) and e2 ~ N(0, sigma2)
    */
    Eigen::Matrix4d A; // State Transfer Matrix
    Eigen::Matrix4d B;
    Eigen::MatrixXd C; // Observation Matrix
    const int state_size = 2, obs_size = 2;
    Eigen::Matrix2d R; // Error Covarience Matrix
    Eigen::Matrix4d Q;

    LinearDynamicSystem() {}
    LinearDynamicSystem &operator=(LinearDynamicSystem &lds) {
        A = lds.A;
        B = lds.B;
        C = lds.C;
        Q = lds.Q;
        R = lds.R;
    }

    LinearDynamicSystem(const LinearDynamicSystem &lds) {
        A = lds.A;
        B = lds.B;
        C = lds.C;
        Q = lds.Q;
        R = lds.R;
    }

    LinearDynamicSystem(double cov1, double cov2) {
        C = Eigen::MatrixXd(2, 4);
        Q(0, 0) = cov1;
        Q(1, 1) = cov1;
        Q(2, 2) = cov1;
        Q(3, 3) = cov1;
        R(0, 0) = cov2;
        R(1, 1) = cov2;
    }

    Eigen::Vector2d getObservation(Eigen::Vector4d input, Eigen::Vector2d error) {
        /**
         * @brief get current observation given input
         * @param input - current state input
         * @param error - the error caused by uncertainty - Gaussian
         * @return observation vector
        */
        Eigen::Vector2d result = C * input + R * error;

        return result;
    }

    Eigen::Vector4d getNextState(Eigen::Vector4d last_state, Eigen::Vector4d control_input, Eigen::Vector4d error) {
        /**
         * @brief get next state given current state and control input
         * @param last_state - current state
         * @return next state vector
        */
        Eigen::Vector4d state = A * last_state + B * control_input + Q * error;

        return state;
    }

    Eigen::Vector4d getReckonedNextState(Eigen::Vector4d last_state, Eigen::Vector4d control_input) {
        /**
         * @brief getting Dead Reckoned state to compare with
         * @return reckoned state vector
        */
         return A * last_state + B * control_input; 
    }
};