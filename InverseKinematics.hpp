#pragma once
// =====================================================
// InverseKinematics — desired tool pose -> 7 joint angles
// =====================================================
// Method: damped least squares (DLS), iterated until the pose error
// is below the tolerance:
//
//   e  = log6( T_current^-1 * T_target )        6D pose error
//   J' = Jlog6(...) * J                          Jacobian of that error
//   dq = J'^T (J' J'^T + lambda I)^-1 e          DLS step (solved with LDLT)
//   q  = clamp(q + stepSize * dq, joint limits)
//
// Uses RobotModel for FK and the Jacobian (no own model loading).

#include <algorithm>
#include <random>

#include <Eigen/Dense>
#include <pinocchio/spatial/explog.hpp>

#include "RobotModel.hpp"

class InverseKinematics
{
public:
    // Solver settings (numerical choices, not universal optimal values).
    struct Settings
    {
        int maxIterations = 2000;
        double tolerance = 1e-5;   // pose-error norm for convergence
        double stepSize = 0.1;     // fraction of the DLS step applied per iteration
        double damping = 1e-4;     // lambda: keeps the solve stable near singularities

        // Cost of moving each joint (redundancy resolution). All 1 = standard IK.
        // A larger weight makes the solver move that joint less, e.g. weight 10
        // on joints 1-3 lets the wrist do most of the work.
        JointVector weights = JointVector::Ones();
    };

    Settings settings;
    int lastIterations = 0;        // iterations used by the last successful solve

    explicit InverseKinematics(RobotModel& robot) : robot_(robot) {}

    // Start from the middle of every joint range (avoids starting on a limit).
    bool solve(const pinocchio::SE3& target, JointVector& solution)
    {
        const JointVector midpoint = 0.5 * (robot_.lowerLimits() + robot_.upperLimits());
        return solveFrom(target, midpoint, solution);
    }

    // Start from a given configuration, e.g. the robot's current joints or
    // the previous waypoint (keeps consecutive poses in the same arm shape).
    bool solveFrom(const pinocchio::SE3& target, const JointVector& start,
                   JointVector& solution)
    {
        const JointVector lower = robot_.lowerLimits();
        const JointVector upper = robot_.upperLimits();
        JointVector q = start;

        for (int iteration = 0; iteration < settings.maxIterations; ++iteration)
        {
            const pinocchio::SE3 difference = robot_.toolPose(q).actInv(target);
            const Eigen::Matrix<double, 6, 1> error = pinocchio::log6(difference).toVector();

            if (error.norm() < settings.tolerance)
            {
                solution = q;
                lastIterations = iteration;
                return true;
            }

            const Eigen::Matrix<double, 6, 7> J =
                pinocchio::Jlog6(difference.inverse()) * robot_.jacobian(q);

            // Weighted damped least squares: dq = D J^T (J D J^T + lambda I)^-1 e
            // with D = diag(1 / weight). With all weights 1 this is the standard
            // formula dq = J^T (J J^T + lambda I)^-1 e.
            const JointVector D = settings.weights.cwiseInverse();
            const Eigen::Matrix<double, 6, 7> JD = J * D.asDiagonal();

            Eigen::Matrix<double, 6, 6> JJt = JD * J.transpose();
            JJt.diagonal().array() += settings.damping;

            const JointVector dq = JD.transpose() * JJt.ldlt().solve(error);
            q = (q + settings.stepSize * dq).cwiseMax(lower).cwiseMin(upper);
        }
        return false;
    }

    // If the midpoint start fails, retry from random configurations.
    // (A single start solved 43/50 random targets; with restarts 50/50.)
    bool solveWithRestarts(const pinocchio::SE3& target, JointVector& solution,
                           int restarts = 10, unsigned int seed = 1)
    {
        if (solve(target, solution)) return true;

        std::mt19937 rng(seed);
        const JointVector lower = robot_.lowerLimits();
        const JointVector upper = robot_.upperLimits();

        for (int attempt = 0; attempt < restarts; ++attempt)
        {
            JointVector start;
            for (int i = 0; i < 7; ++i)
                start[i] = std::uniform_real_distribution<double>(lower[i], upper[i])(rng);

            if (solveFrom(target, start, solution)) return true;
        }
        return false;
    }

private:
    RobotModel& robot_;
};
