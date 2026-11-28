#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

namespace ekf_slam_cpp {    

    // Robot pose: x, y and orientation
    using PoseVector = Eigen::Vector3d;

    // Landmark position: x and y
    using LandmarkVector = Eigen::Vector2d;

    // Complete EKF state vector
    using StateVector = Eigen::VectorXd;

    // Complete EKF Covariance matrix
    using CovarianceMatrix = Eigen::MatrixXd;

    // Represents the robot's pose
    struct Pose2D {
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    // Represents a landmark in the environment
    struct Landmark {
        int id{-1}; // unique identifier of each Landmark
        double x{0.0};
        double y{0.0};
    };

    // Robot velocity commands
    struct ControlInput {
        double linear_velocity{0.0};
        double angular_velocity{0.0};
    };

    // Range - How far the landmark is?
    // Bearing - The angle of the landmark relative to the robot's heading
    struct RangeBearingMeasurement {
        int landmark_id{-1};
        double range{0.0};
        double bearing{0.0};
    };

    // Complete EKF-SLAM state
    struct RobotState {
        StateVector mean;
        CovarianceMatrix covariance;

        std::vector<int> landmark_ids;
    };
} // namespace ekf_slam