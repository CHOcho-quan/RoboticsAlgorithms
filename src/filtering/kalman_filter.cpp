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
    LinearDynamicSystem lds(10.0, 10.0);
    lds.A << 1.0, 0.0, dT, 0.0, 
            0.0, 1.0, 0.0, dT,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    lds.B << pow(dT, 2) / 20, 0.0, 0.0, 0.0,
            0.0, pow(dT, 2) / 20, 0.0, 0.0,
            0.0, 0.0, dT / 10, 0.0,
            0.0, 0.0, 0.0, dT / 10;

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
    std::uniform_real_distribution<> n; // Normal Distribution

    cv::namedWindow("Kalman Filter");
    cv::Mat background = cv::Mat(200, 200, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<cv::Point> observations, reckoned, position, filtering;
    
    for (double t = 0.0;t < T;t += dT) {
        Eigen::Vector2d errorObs;
        Eigen::Vector4d errorState;
        errorObs(0) = n(gen);
        errorObs(1) = n(gen);
        errorState(0) = n(gen);
        errorState(1) = n(gen);
        errorState(2) = n(gen);
        errorState(3) = n(gen);

        pos = lds.getNextState(pos, u, errorState);
        reckon = lds.getReckonedNextState(reckon, u);
        Eigen::Vector2d obs = lds.getObservation(pos, errorObs);

        kf.predict(u);
        kf.update(obs);

        if (DEBUG) std::cout << "KF: " << kf.filtered(0) << ' ' << kf.filtered(1) << std::endl;
        if (DEBUG) std::cout << "POS: " << pos(0) << ' ' << pos(1) << std::endl;
        if (DEBUG) std::cout << "OBS: " << obs(0) << ' ' << obs(1) << std::endl;

        // Visualizing
        observations.push_back(cv::Point(cvRound(obs(0)), cvRound(obs(1))));
        reckoned.push_back(cv::Point(cvRound(reckon(0)), cvRound(reckon(1))));
        position.push_back(cv::Point(cvRound(pos(0)), cvRound(pos(1))));
        filtering.push_back(cv::Point(cvRound(kf.filtered(0)), cvRound(kf.filtered(1))));
    }

    cv::polylines(background, reckoned, false, cv::Scalar(0, 0, 0), 5);
    cv::polylines(background, position, false, cv::Scalar(255, 0, 0), 10);
    cv::polylines(background, filtering, false, cv::Scalar(0, 255, 0), 5);

    for (int i = 0;i < observations.size();i++) cv::circle(background, observations[i], 3, cv::Scalar(0, 0, 255), -1);

    cv::imshow("Kalman Filter", background);
    cv::waitKey(0);

    cv::resize(background, background, cv::Size(300, 300));
    cv::imwrite("../results/filtering/kf.png", background);
}