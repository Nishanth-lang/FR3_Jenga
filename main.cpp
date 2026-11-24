// =====================================================
// FR3 Jenga — main application
// =====================================================
// Current content: camera-to-robot coordinate transformation demo.
// (Hardcoded example transform — NOT a real camera calibration.)
// Later this becomes the Jenga program: camera -> block pose -> IK ->
// motion.

#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

#include "CoordinateTransform.hpp"

int main()
{
    std::cout << "FR3 Jenga - Coordinate Transformation\n"
              << "=====================================\n";

    // Example transform: camera rotated 90 deg about Z, 0.5 m along X.
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T(0, 3) = 0.5;

    CoordinateTransform transform;
    transform.setTransformation(T);
    transform.printTransformation();

    const Eigen::Vector3d cameraPoint(0.4, 0.2, 0.6);
    const Eigen::Vector3d robotPoint = transform.cameraToRobot(cameraPoint);

    std::cout << "\nBlock in camera frame: " << cameraPoint.transpose()
              << "\nBlock in robot frame:  " << robotPoint.transpose()
              << "\nExpected:              0.3 0.4 0.6\n";

    const bool ok = (robotPoint - Eigen::Vector3d(0.3, 0.4, 0.6)).norm() < 1e-12;
    std::cout << (ok ? "\nPASS" : "\nFAIL") << ": camera point converted correctly\n";
    return ok ? 0 : 1;
}
