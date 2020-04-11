//
// Created by quan
// 2020.4.11
//
#include "ekf.hpp"

class ExtendedKalmanFilter {
    /**
     * Extended Linear Kalman Filter for Nonlinear dynamic system
     * predict - for the kalman filter predict section
     * update - for the kalman filter update section
    */
 public:
    Eigen::MatrixXd kalman_gain;
    Eigen::Vector4d prediction;
    Eigen::Vector4d filtered;
    Eigen::Matrix4d covarience;
    Eigen::Matrix4d last_covarience;
    NonlinearDynamicSystem nlds;

    ExtendedKalmanFilter(NonlinearDynamicSystem s) {
        /**
         * @brief Initialization of kalman filter
         * @param s - the given dynamic system
        */
        nlds = s;
        // Initial Eye matrix for covariences
        covarience = Eigen::Matrix4d::Identity();
        last_covarience = Eigen::Matrix4d::Identity();
        kalman_gain = Eigen::MatrixXd(4, 2);
    }
    void predict(Eigen::Vector2d control_input) {
        /**
         * @brief prediction phase of kalman filter
        */
        auto jA = nlds.getJacobA(control_input);
        prediction = nlds.A * filtered + nlds.B * control_input;
        covarience = jA * last_covarience * jA.transpose() + nlds.Q;
    }
    void update(Eigen::Vector2d obs) {
        /**
         * @brief update phase of kalman filter
         * @param obs - given observation of current state
        */
        kalman_gain = covarience * nlds.H.transpose() * 
            (nlds.H * covarience * nlds.H.transpose() + nlds.R).inverse();
        filtered = prediction + kalman_gain * (obs - nlds.H * prediction);
        last_covarience = covarience - kalman_gain * nlds.H * covarience;
    }
};

int main()
{
    NonlinearDynamicSystem nlds(0.5, 0.5);
    nlds.A << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    nlds.H << 1, 0, 0, 0,
              0, 1, 0, 0;
    
    ExtendedKalmanFilter ekf(nlds);

    // Initial State of position
    Eigen::Vector4d pos(1.0, 1.0, 0.5, M_PI / 3), reckon(1.0, 1.0, 0.5, M_PI / 3);
    ekf.filtered(0) = pos(0);
    ekf.filtered(1) = pos(1);
    ekf.filtered(2) = pos(2);
    ekf.filtered(3) = pos(3);
    nlds.setCurState(pos);

    // Control Input
    Eigen::Vector2d u(1.0, M_PI / 720);

    // Randomized tools
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> n{0, 1}; // Normal Distribution

    double T = 50.0;
    for (double t = 0.0;t < T;t+=dT) {
        Eigen::Vector4d reckoned, errorState;
        Eigen::Vector2d errorObs;

        errorObs(0) = n(gen);
        errorObs(1) = n(gen);
        errorState(0) = n(gen);
        errorState(1) = n(gen);
        errorState(2) = n(gen);
        errorState(3) = n(gen);

        pos = nlds.getNextState(pos, u, errorState);
        reckon = nlds.getReckonedNextState(reckon, u);
        Eigen::Vector2d obs = nlds.getObservation(pos, errorObs);

        ekf.predict(u);
        ekf.update(obs);

        if (DEBUG) std::cout << "EKF: " << ekf.filtered(0) << ' ' << ekf.filtered(1) << std::endl;
        if (DEBUG) std::cout << "POS: " << pos(0) << ' ' << pos(1) << std::endl;
        if (DEBUG) std::cout << "OBS: " << obs(0) << ' ' << obs(1) << std::endl;
        if (DEBUG) std::cout << "RECKON: " << reckon(0) << ' ' << reckon(1) << std::endl;
    }
}