//
// Created by quan
// 2020.4.6
//
#include "kalman_filter.hpp"

class KalmanFilter {
    /**
     * Simple 2D Linear Kalman Filter for dynamic system
     * predict - for the kalman filter predict section
     * update - for the kalman filter update section
    */
 public:
    Eigen::Matrix2d kalman_gain;
    Eigen::Vector2d prediction;
    Eigen::Vector2d filtered;
    Eigen::Matrix2d covarience;
    Eigen::Matrix2d last_covarience;
    LinearDynamicSystem lds;

    KalmanFilter(LinearDynamicSystem s) : lds(s) {
        // Initial Eye matrix for covariences
        covarience(0, 0) = 1.0;
        covarience(1, 1) = 1.0;
        last_covarience(0, 0) = 1.0;
        last_covarience(1, 1) = 1.0;
    }
    void predict() {
        prediction = lds.A * filtered + lds.B;
        covarience = lds.A * last_covarience * lds.A.transpose() + lds.Q;
    }
    void update(Eigen::Vector2d obs) {
        kalman_gain = covarience * lds.C.transpose() * 
            (lds.C.transpose() * covarience * lds.C + lds.R).inverse();
        filtered = prediction + kalman_gain * (obs - lds.C * prediction);
        last_covarience = covarience - kalman_gain * lds.C * covarience;
    }
};

int main()
{
    LinearDynamicSystem lds(1.0, 1.0);
    KalmanFilter kf(lds);
    lds.A << 1.0, 0.0, 
            0.0, 1.0;
    lds.B << 1.0, 1.0;

    lds.C << 1.0, 0.0,
            0.0, 1.0;
    lds.D << 1.0, 1.0;

    // Initial State of position
    Eigen::Vector2d pos(1.0, 1.0);

    // Randomized tools
    std::random_device rd;
    std::mt19937 gen(rd());

    double dT = 0.1, T = 50;
    for (dT;dT < T;dT += 0.1) {
        Eigen::Vector2d obs = lds.getObservation(pos, gen);
        kf.predict();
        kf.update(obs);
    }
}