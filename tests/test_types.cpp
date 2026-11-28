
#include "ekf_slam_cpp/types.hpp"

#include <iostream>

int main()
{
    ekf_slam_cpp::Pose2D pose{2.0, 3.0, 1.57};

    ekf_slam_cpp::Landmark landmark{1, 5.0, 4.0};

    ekf_slam_cpp::ControlInput control{0.5, 0.2};

    ekf_slam_cpp::RangeBearingMeasurement measurement{
        1, 3.0, 0.5
    };

    ekf_slam_cpp::RobotState state;

    state.mean = ekf_slam_cpp::StateVector::Zero(3);
    state.covariance = ekf_slam_cpp::CovarianceMatrix::Identity(3, 3);

    std::cout << "Robot pose: "
              << pose.x << ", "
              << pose.y << ", "
              << pose.theta << std::endl;

    std::cout << "Landmark ID: "
              << landmark.id << std::endl;

    std::cout << "Linear velocity: "
              << control.linear_velocity << std::endl;

    std::cout << "Measurement range: "
              << measurement.range << std::endl;

    std::cout << "State size: "
              << state.mean.size() << std::endl;

    std::cout << "Covariance dimensions: "
              << state.covariance.rows() << " x "
              << state.covariance.cols() << std::endl;

    return 0;
}
