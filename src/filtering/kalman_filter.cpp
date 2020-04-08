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
    Eigen::MatrixXd kalman_gain;
    Eigen::Vector4d prediction;
    Eigen::Vector4d filtered;
    Eigen::Matrix4d covarience;
    Eigen::Matrix4d last_covarience;
    LinearDynamicSystem lds;

    KalmanFilter(LinearDynamicSystem s) {
        lds = s;
        // Initial Eye matrix for covariences
        covarience = Eigen::Matrix4d::Identity();
        last_covarience = Eigen::Matrix4d::Identity();
        kalman_gain = Eigen::MatrixXd(4, 2);
    }
    void predict(Eigen::Vector4d control_input) {
        prediction = lds.A * filtered + lds.B * control_input;
        covarience = lds.A * last_covarience * lds.A.transpose() + lds.Q;
    }
    void update(Eigen::Vector2d obs) {
        kalman_gain = covarience * lds.C.transpose() * 
            (lds.C * covarience * lds.C.transpose() + lds.R).inverse();
        filtered = prediction + kalman_gain * (obs - lds.C * prediction);
        last_covarience = covarience - kalman_gain * lds.C * covarience;
    }
};

int main()
{
    double dT = 0.1, T = 50;
    LinearDynamicSystem lds(1.0, 1.0); // Worse Observation
    lds.A << 1.0, 0.0, dT, 0.0, 
            0.0, 1.0, 0.0, dT,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    lds.B << pow(dT, 2) / 2, 0.0, 0.0, 0.0,
            0.0, pow(dT, 2) / 2, 0.0, 0.0,
            0.0, 0.0, dT, 0.0,
            0.0, 0.0, 0.0, dT;

    lds.C << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0;
    KalmanFilter kf(lds);

    // Initial State of position
    Eigen::Vector4d pos(1.0, 1.0, 0.2, 0.2), reckon(1.0, 1.0, 0.2, 0.2);
    kf.filtered(0) = pos(0);
    kf.filtered(1) = pos(1);
    kf.filtered(2) = pos(2);
    kf.filtered(3) = pos(3);

    // Control Input
    Eigen::Vector4d u(0.0, 0.0, 1.0, 0.5);

    // Randomized tools
    std::random_device rd{};
    std::mt19937 gen{rd()};

    cv::namedWindow("Kalman Filter");
    cv::Mat background = cv::Mat(2000, 2000, CV_8UC3, cv::Scalar(255, 255, 255));
    
    for (double t = 0.0;t < T;t += dT) {
        pos = lds.getNextState(pos, u, gen);
        reckon = lds.getReckonedNextState(reckon, u);
        Eigen::Vector2d obs = lds.getObservation(pos, gen);

        kf.predict(u);
        kf.update(obs);

        // if (DEBUG) std::cout << "KF: " << kf.filtered(0) << ' ' << kf.filtered(1) << std::endl;
        // if (DEBUG) std::cout << "POS: " << pos(0) << ' ' << pos(1) << std::endl;
        // if (DEBUG) std::cout << "OBS: " << obs(0) << ' ' << obs(1) << std::endl;

        // Visualizing
        cv::circle(background, cv::Point(int(reckon(0)), int(reckon(1))), 5, cv::Scalar(0, 0, 0), -1);
        cv::circle(background, cv::Point(int(pos(0)), int(pos(1))), 5, cv::Scalar(0, 255, 0), -1);
        cv::circle(background, cv::Point(int(kf.filtered(0)), int(kf.filtered(1))), 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(background, cv::Point(int(obs(0)), int(obs(1))), 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow("Kalman Filter", background);
    cv::waitKey(0);

    // cv::resize(background, background, cv::Size(300, 300));
    cv::imwrite("../results/filtering/kf.png", background);
}