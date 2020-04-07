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
     * Xt = A * Xt-1 + B + e1
     * Zt = C * Xt + D + e2
     * where e1 ~ N(0, sigma1) and e2 ~ N(0, sigma2)
    */
    Eigen::Matrix2d A; // State Transfer Matrix
    Eigen::Vector2d B;
    Eigen::Matrix2d C; // Observation Matrix
    Eigen::Vector2d D;
    const int state_size = 2, obs_size = 2;
    Eigen::Matrix2d Q, R; // Error Covarience Matrix

    std::uniform_real_distribution<> n; // Normal Distribution

    LinearDynamicSystem() {}
    LinearDynamicSystem &operator=(LinearDynamicSystem &lds) {
        A = lds.A;
        B = lds.B;
        C = lds.C;
        D = lds.D;
        Q = lds.Q;
        R = lds.R;
    }

    LinearDynamicSystem(const LinearDynamicSystem &lds) {
        A = lds.A;
        B = lds.B;
        C = lds.C;
        D = lds.D;
        Q = lds.Q;
        R = lds.R;
    }

    LinearDynamicSystem(double cov1, double cov2) {
        n = std::uniform_real_distribution<>(0, 1);
        Q(0, 0) = cov1;
        Q(1, 1) = cov1;
        R(0, 0) = cov2;
        R(1, 1) = cov2;
    }

    Eigen::Vector2d getObservation(Eigen::Vector2d input, std::mt19937 gen) {
        Eigen::Vector2d result = C * input + D;
        result(0) += n(gen) * Q(0, 0);
        result(1) += n(gen) * Q(1, 1);

        return result;
    }

    Eigen::Vector2d getNextState(Eigen::Vector2d last_state, std::mt19937 gen) {
        Eigen::Vector2d state = A * last_state + B;
        state(0) += n(gen) * R(0, 0);
        state(1) += n(gen) * R(1, 1);

        return state;
    }

    Eigen::Vector2d getReckonedNextState(Eigen::Vector2d last_state) { return A * last_state + B; }
};