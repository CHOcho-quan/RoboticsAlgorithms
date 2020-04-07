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

    KalmanFilter(LinearDynamicSystem s) {
        lds = s;
        // Initial Eye matrix for covariences
        covarience(0, 0) = 1.0;
        covarience(1, 1) = 1.0;
        last_covarience(0, 0) = 1.0;
        last_covarience(1, 1) = 1.0;
    }
    void predict() {
        prediction = lds.A * filtered + lds.B;
        // std::cout << "PREDICT: " << lds.A << std::endl;
        covarience = lds.A * last_covarience * lds.A.transpose() + lds.Q;
    }
    void update(Eigen::Vector2d obs) {
        kalman_gain = covarience * lds.C.transpose() * 
            (lds.C.transpose() * covarience * lds.C + lds.R).inverse();
        filtered = prediction + kalman_gain * (obs - lds.C * prediction - lds.D);
        last_covarience = covarience - kalman_gain * lds.C * covarience;
    }
};

int main()
{
    LinearDynamicSystem lds(1, 1);
    lds.A << 1.0, 0.0, 
            0.0, 1.0;
    lds.B << 1.0, 1.0;

    lds.C << 1.0, 0.0,
            0.0, 1.0;
    lds.D << 1.0, 1.0;
    KalmanFilter kf(lds);
    std::cout << lds.A;

    // Initial State of position
    Eigen::Vector2d pos(1.0, 1.0), reckon(1.0, 1.0);
    kf.filtered(0) = 1.0;
    kf.filtered(1) = 1.0;

    // Randomized tools
    std::random_device rd;
    std::mt19937 gen(rd());

    cv::namedWindow("Kalman Filter");
    cv::Mat background = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    double dT = 0.1, T = 50;
    for (double t = 0.0;t < T;t += dT) {
        pos = lds.getNextState(pos, gen);
        reckon = lds.getReckonedNextState(reckon);
        Eigen::Vector2d obs = lds.getObservation(pos, gen);

        // Visualizing
        cv::circle(background, cv::Point(int(reckon(0)), int(reckon(1))), 5, cv::Scalar(0, 0, 0), -1);
        cv::circle(background, cv::Point(int(pos(0)), int(pos(1))), 5, cv::Scalar(0, 255, 0), -1);
        cv::circle(background, cv::Point(int(kf.filtered(0)), int(kf.filtered(1))), 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(background, cv::Point(int(obs(0)), int(obs(1))), 5, cv::Scalar(0, 0, 255), -1);

        if (DEBUG) std::cout << "KF: " << kf.filtered(0) << ' ' << kf.filtered(1) << std::endl;
        if (DEBUG) std::cout << "POS: " << pos(0) << ' ' << pos(1) << std::endl;
        if (DEBUG) std::cout << "OBS: " << obs(0) << ' ' << obs(1) << std::endl;

        kf.predict();
        kf.update(obs);
    }
    cv::imshow("Kalman Filter", background);
    cv::waitKey(0);
}