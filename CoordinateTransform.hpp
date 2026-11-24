
#pragma once

#include <Eigen/Dense>
#include <iostream>

// =====================================================
// CLASS: COORDINATE TRANSFORMATION
// =====================================================

class CoordinateTransform
{
private:

    // Complete transformation from camera frame to robot base.
    Eigen::Matrix4d T_base_camera;

public:

    // Initialize with an identity transformation.
    CoordinateTransform()
    {
        T_base_camera = Eigen::Matrix4d::Identity();
    }

    // Set the complete camera-to-robot transformation.
    void setTransformation(const Eigen::Matrix4d& transform)
    {
        T_base_camera = transform;
    }

    // =================================================
    // CONVERT POSITION
    // =================================================

    Eigen::Vector3d cameraToRobot(
        const Eigen::Vector3d& pointCamera
    ) const
    {
        // Convert XYZ into homogeneous coordinates.
        Eigen::Vector4d point;
        point << pointCamera, 1.0;

        // Apply rotation and translation.
        Eigen::Vector4d pointRobot =
            T_base_camera * point;

        return pointRobot.head<3>();
    }

    // =================================================
    // CONVERT ORIENTATION
    // =================================================

    Eigen::Matrix3d cameraOrientationToRobot(
        const Eigen::Matrix3d& orientationCamera
    ) const
    {
        // Extract the rotation between camera and robot.
        Eigen::Matrix3d R_base_camera =
            T_base_camera.block<3, 3>(0, 0);

        // Express the object's orientation in robot coordinates.
        Eigen::Matrix3d orientationRobot =
            R_base_camera * orientationCamera;

        return orientationRobot;
    }

    // =================================================
    // DISPLAY TRANSFORMATION
    // =================================================

    void printTransformation() const
    {
        std::cout << "\nCamera to Robot Transformation:\n";
        std::cout << T_base_camera << "\n";
    }
};
