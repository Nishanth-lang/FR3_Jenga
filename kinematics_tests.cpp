// =====================================================
// FR3 Jenga — Kinematics validation (all tests in one program)
// =====================================================
// Replaces robot_model_test, ik_test, jacobian_test and
// ik_robustness_test. Prints PASS/FAIL per check and returns 1 if any
// check fails.
//
//   1. Forward kinematics   (regression against the verified reference)
//   2. Inverse kinematics   (one target, verified with FK)
//   3. Jacobian             (finite differences, x_dot = J q_dot,
//                            reference frames, singularity)
//   4. IK robustness        (50 random targets, unreachable targets)

#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/SVD>
#include <pinocchio/spatial/explog.hpp>

#include "InverseKinematics.hpp"
#include "RobotModel.hpp"

namespace
{
bool allPassed = true;

void check(const std::string& name, bool passed)
{
    std::cout << (passed ? "  PASS: " : "  FAIL: ") << name << "\n";
    allPassed = allPassed && passed;
}

// Position error (m) and orientation error (rad) between two poses.
double positionError(const pinocchio::SE3& a, const pinocchio::SE3& b)
{
    return (a.translation() - b.translation()).norm();
}
double rotationError(const pinocchio::SE3& a, const pinocchio::SE3& b)
{
    return pinocchio::log3(a.rotation().transpose() * b.rotation()).norm();
}

// Motion from pose a to pose b as a 6D twist in a's own (LOCAL) frame.
Eigen::Matrix<double, 6, 1> localTwist(const pinocchio::SE3& a, const pinocchio::SE3& b)
{
    return pinocchio::log6(a.actInv(b)).toVector();
}
}  // namespace

int main()
{
    std::cout << std::scientific << std::setprecision(3);
    std::cout << "FR3 Jenga - Kinematics Validation\n"
              << "=================================\n";

    RobotModel robot("models/fr3/fr3.urdf");
    InverseKinematics ik(robot);

    // Reference configuration used throughout the project.
    JointVector q0;
    q0 << 0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5;
    const pinocchio::SE3 T0 = robot.toolPose(q0);

    // =================================================
    std::cout << "\n1. FORWARD KINEMATICS\n";
    // =================================================
    // Reference TCP position, verified earlier against direct Pinocchio FK.
    const Eigen::Vector3d referencePosition(0.287452, 0.0, 0.716366);

    std::cout << std::fixed << std::setprecision(6)
              << "  TCP position [m]: " << T0.translation().transpose() << "\n"
              << std::scientific << std::setprecision(3);
    check("TCP position matches reference (< 1 micrometre)",
          (T0.translation() - referencePosition).norm() < 1e-6);

    // =================================================
    std::cout << "\n2. INVERSE KINEMATICS (one target)\n";
    // =================================================
    JointVector qIk;
    const bool converged = ik.solve(T0, qIk);
    const pinocchio::SE3 Tik = robot.toolPose(qIk);

    std::cout << std::fixed << std::setprecision(6)
              << "  Iterations: " << ik.lastIterations << "\n"
              << "  Solution [rad]: " << qIk.transpose() << "\n"
              << "  Position error: " << positionError(T0, Tik) << " m\n"
              << "  Orientation error: " << rotationError(T0, Tik) << " rad\n"
              << std::scientific << std::setprecision(3);
    check("IK converged", converged);
    check("IK solution reaches target (< 0.1 mm, < 0.1 mrad)",
          positionError(T0, Tik) < 1e-4 && rotationError(T0, Tik) < 1e-4);
    // Different joints than q0 are expected: 7 joints, 6D pose -> redundancy.

    // =================================================
    std::cout << "\n3. JACOBIAN\n";
    // =================================================
    const Eigen::Matrix<double, 6, 7> J = robot.jacobian(q0);

    // 3a. Finite differences: move each joint by +-h, measure the tool
    //     motion with FK, compare with the Jacobian column.
    const double h = 1e-6;
    Eigen::Matrix<double, 6, 7> Jnumeric;
    for (int i = 0; i < 7; ++i)
    {
        JointVector qPlus = q0, qMinus = q0;
        qPlus[i] += h;
        qMinus[i] -= h;
        Jnumeric.col(i) = (localTwist(T0, robot.toolPose(qPlus)) -
                           localTwist(T0, robot.toolPose(qMinus))) / (2.0 * h);
    }
    const double fdError = (J - Jnumeric).cwiseAbs().maxCoeff();
    std::cout << "  Finite-difference max error: " << fdError << "\n";
    check("Jacobian matches finite differences (< 1e-6)", fdError < 1e-6);

    // 3b. Velocity: predict x_dot = J q_dot, compare with FK over dt.
    JointVector qDot;
    qDot << 0.10, -0.20, 0.15, 0.30, -0.10, 0.25, -0.40;   // rad/s
    const double dt = 1e-5;
    const Eigen::Matrix<double, 6, 1> predicted = J * qDot;
    const Eigen::Matrix<double, 6, 1> measured =
        localTwist(T0, robot.toolPose(q0 + qDot * dt)) / dt;
    const double velocityError = (predicted - measured).norm() / predicted.norm();
    std::cout << "  Velocity relative error: " << velocityError << "\n";
    check("x_dot = J * q_dot matches FK motion (< 1e-3)", velocityError < 1e-3);

    // 3c. Frames: base-aligned Jacobian = TCP rotation applied to LOCAL Jacobian.
    const Eigen::Matrix<double, 6, 7> Jaligned =
        robot.jacobian(q0, pinocchio::LOCAL_WORLD_ALIGNED);
    const Eigen::Matrix3d R = T0.rotation();
    Eigen::Matrix<double, 6, 7> Jrotated;
    Jrotated << R * J.topRows<3>(), R * J.bottomRows<3>();
    const double frameError = (Jaligned - Jrotated).cwiseAbs().maxCoeff();
    std::cout << "  Frame consistency error: " << frameError << "\n";
    check("LOCAL and base-aligned Jacobians differ only by rotation", frameError < 1e-10);

    // 3d. Singularity: elbow almost straight (joint 4 near its -0.1169 limit).
    JointVector qElbow = q0;
    qElbow[3] = -0.12;
    const Eigen::Matrix<double, 6, 1> sTest =
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>>(J).singularValues();
    const Eigen::Matrix<double, 6, 1> sElbow =
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>>(robot.jacobian(qElbow)).singularValues();

    std::cout << std::fixed << std::setprecision(5)
              << "                        test pose   straight elbow\n"
              << "  Smallest sing. value: " << std::setw(9) << sTest(5)
              << "   " << std::setw(9) << sElbow(5) << "\n"
              << "  Manipulability:       " << std::setw(9) << sTest.prod()
              << "   " << std::setw(9) << sElbow.prod() << "\n"
              << std::setprecision(1)
              << "  Condition number:     " << std::setw(9) << sTest(0) / sTest(5)
              << "   " << std::setw(9) << sElbow(0) / sElbow(5) << "\n"
              << std::scientific << std::setprecision(3);
    check("smallest singular value >= 2x lower near straight elbow",
          sElbow(5) < 0.5 * sTest(5));

    // =================================================
    std::cout << "\n4. IK ROBUSTNESS\n";
    // =================================================
    // 50 random reachable targets: FK of random joints in the middle 80 %
    // of each range. Fixed seed -> the same targets every run.
    std::mt19937 rng(42);
    const JointVector lower = robot.lowerLimits();
    const JointVector upper = robot.upperLimits();

    int singleStart = 0, withRestarts = 0, wrongAnswers = 0;
    const int targets = 50;

    for (int t = 0; t < targets; ++t)
    {
        JointVector qRandom;
        for (int i = 0; i < 7; ++i)
        {
            const double margin = 0.1 * (upper[i] - lower[i]);
            qRandom[i] = std::uniform_real_distribution<double>(
                lower[i] + margin, upper[i] - margin)(rng);
        }
        const pinocchio::SE3 target = robot.toolPose(qRandom);

        JointVector q;
        if (ik.solve(target, q)) ++singleStart;

        if (ik.solveWithRestarts(target, q))
        {
            const pinocchio::SE3 reached = robot.toolPose(q);
            if (positionError(target, reached) < 1e-4 && rotationError(target, reached) < 1e-4)
                ++withRestarts;
            else
                ++wrongAnswers;
        }
    }

    std::cout << "  Single start:  " << singleStart << " / " << targets << "\n"
              << "  With restarts: " << withRestarts << " / " << targets << "\n";
    check("no solution reported as success but wrong", wrongAnswers == 0);
    check("with restarts >= 98 % solved", withRestarts >= 0.98 * targets);

    // Unreachable targets (FR3 reach is under 1 m): must report failure.
    const std::vector<Eigen::Vector3d> unreachable = {
        {2.0, 0.0, 0.5}, {0.0, 1.5, 0.5}, {0.3, 0.0, 2.0}};
    Eigen::Matrix3d down;
    down << 1, 0, 0, 0, -1, 0, 0, 0, -1;

    int rejected = 0;
    for (const Eigen::Vector3d& p : unreachable)
    {
        JointVector q;
        if (!ik.solveWithRestarts(pinocchio::SE3(down, p), q)) ++rejected;
    }
    std::cout << "  Unreachable targets rejected: " << rejected << " / 3\n";
    check("all unreachable targets rejected", rejected == 3);

    // =================================================
    std::cout << "\n=================================\n"
              << (allPassed ? "ALL KINEMATICS TESTS PASSED\n" : "SOME TESTS FAILED\n");
    return allPassed ? 0 : 1;
}
