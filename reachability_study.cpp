// =====================================================
// FR3 Jenga — Reachability study (concept feasibility)
// =====================================================
// Question: where should the tower stand so the FR3 can push out and
// pull out as many blocks as possible?
//
// For each tower placement (distance, rotation, pedestal) and each of the
// 54 blocks, the extraction sequence is checked:
//   PUSH: touch one block end, push it 35 mm into the tower
//   PULL: grip the protruding other end, pull it 100 mm out
// A move is feasible if IK finds both end poses, the second pose follows
// from the first without a large joint jump, no joint is within 0.05 rad
// of a limit, and the arm/hand stays out of the tower and above the table.
//
// Usage:
//   ./build/fr3_reachability_study                 standard Franka fingers
//   ./build/fr3_reachability_study side 0.10       L-shaped fingers, tip 10 cm sideways
//   ./build/fr3_reachability_study straight 0.10   fingers 10 cm longer, straight down
//
// LIMITATIONS: approximate collision check (sampled points + hand box),
// kinematics only (no forces, friction or tower stability).

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "InverseKinematics.hpp"
#include "RobotModel.hpp"

namespace
{
// ---- Jenga tower (standard block, metres) ----
const double BLOCK_L = 0.075, BLOCK_W = 0.025, BLOCK_H = 0.015;
const int LAYERS = 18;

// ---- Extraction motion ----
const double PUSH_DEPTH = 0.035, GRIP_OFFSET = 0.020, PULL_LENGTH = 0.100;

// ---- Placements to test ----
const std::vector<double> PEDESTALS = {0.0, 0.10};
const std::vector<double> YAWS_DEG = {0.0, 45.0};
const std::vector<double> DISTANCES = {0.40, 0.45, 0.50, 0.55, 0.60, 0.65};

// ---- Safety margins ----
const double MIN_JOINT_MARGIN = 0.05;   // rad
const double MAX_JOINT_JUMP = 0.5;      // rad between the two poses of one move

struct Tower
{
    pinocchio::SE3 placement;   // tower base centre in the robot base frame
    double pedestal = 0.0;

    // Is a point inside the tower (or its pedestal), with a margin?
    bool contains(const Eigen::Vector3d& world, double margin) const
    {
        const Eigen::Vector3d p = placement.actInv(world);
        const double half = BLOCK_L / 2 + margin;
        return std::abs(p.x()) < half && std::abs(p.y()) < half &&
               p.z() > -pedestal - margin && p.z() < LAYERS * BLOCK_H + margin;
    }
};

struct Result
{
    int layer = 0, block = 0;
    bool extractable = false;
    double margin = 0.0, manipulability = 0.0;
};

class Study
{
public:
    explicit Study(RobotModel& robot) : robot_(robot), ik_(robot)
    {
        for (const char* name : {"fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
                                 "fr3_link5", "fr3_link6", "fr3_link7", "fr3_hand"})
            armFrames_.push_back(robot.frameId(name));
        handId_ = robot.frameId("fr3_hand");
    }

    std::vector<Result> evaluate(double distance, double yawDeg, double pedestal)
    {
        const Eigen::Matrix3d R =
            Eigen::AngleAxisd(yawDeg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Tower tower{pinocchio::SE3(R, Eigen::Vector3d(distance, 0.0, pedestal)), pedestal};

        std::vector<Result> results;
        for (int layer = 0; layer < LAYERS; ++layer)
        {
            // Layers alternate: even along tower X, odd along tower Y.
            const bool even = (layer % 2 == 0);
            const Eigen::Vector3d axis = R * (even ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY());
            const Eigen::Vector3d across = even ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX();

            for (int j = 0; j < 3; ++j)
            {
                const Eigen::Vector3d local =
                    across * ((j - 1) * BLOCK_W) + Eigen::Vector3d(0, 0, (layer + 0.5) * BLOCK_H);
                const Eigen::Vector3d center = tower.placement.act(local);

                Result r;
                r.layer = layer + 1;
                r.block = j + 1;

                // Push from either end; pull at the opposite end.
                for (const double sign : {1.0, -1.0})
                {
                    const Eigen::Vector3d inward = sign * axis;
                    const Eigen::Vector3d pushEnd = center - inward * (BLOCK_L / 2);
                    const Eigen::Vector3d pullEnd = center + inward * (BLOCK_L / 2);
                    const Eigen::Vector3d grip = pullEnd + inward * (PUSH_DEPTH - GRIP_OFFSET);

                    double m1, m2, w1, w2;
                    if (phase(pushEnd, pushEnd + inward * PUSH_DEPTH, inward, tower, m1, w1) &&
                        phase(grip, grip + inward * PULL_LENGTH, -inward, tower, m2, w2))
                    {
                        r.extractable = true;
                        r.margin = std::min(m1, m2);
                        r.manipulability = std::min(w1, w2);
                        break;
                    }
                }
                results.push_back(r);
            }
        }
        return results;
    }

private:
    // Tool Z = approach direction; 'roll' picks where the finger-closing axis Y points.
    static Eigen::Matrix3d toolRotation(const Eigen::Vector3d& approach, int roll)
    {
        const Eigen::Vector3d z = approach.normalized();
        const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d side = up.cross(z).normalized();
        const Eigen::Vector3d y = (roll == 0) ? side : (roll == 1) ? up : (roll == 2) ? -side : -up;

        Eigen::Matrix3d R;
        R << y.cross(z), y, z;
        return R;
    }

    // One push or pull phase: try all four finger orientations.
    bool phase(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& approach,
               const Tower& tower, double& margin, double& manipulability)
    {
        for (int roll = 0; roll < 4; ++roll)
            if (move(p1, p2, toolRotation(approach, roll), tower, margin, manipulability))
                return true;
        return false;
    }

    // Straight move p1 -> p2 with fixed orientation R.
    bool move(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Matrix3d& R,
              const Tower& tower, double& margin, double& manipulability)
    {
        JointVector q1, q2;
        if (!ik_.solveWithRestarts(pinocchio::SE3(R, p1), q1, 4, seed_++)) return false;
        if (!collisionFree(q1, tower)) return false;
        if (!ik_.solveFrom(pinocchio::SE3(R, p2), q1, q2)) return false;   // continue from q1
        if (!collisionFree(q2, tower)) return false;
        if ((q2 - q1).cwiseAbs().maxCoeff() > MAX_JOINT_JUMP) return false;

        margin = std::min(robot_.jointLimitMargin(q1), robot_.jointLimitMargin(q2));
        if (margin < MIN_JOINT_MARGIN) return false;

        manipulability = std::min(manip(q1), manip(q2));
        return true;
    }

    double manip(const JointVector& q)
    {
        const Eigen::Matrix<double, 6, 7> J = robot_.jacobian(q);
        return std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
    }

    // APPROXIMATE collision check: points along the arm + corners of a hand box.
    bool collisionFree(const JointVector& q, const Tower& tower)
    {
        robot_.update(q);

        for (std::size_t k = 0; k + 1 < armFrames_.size(); ++k)
        {
            const Eigen::Vector3d a = robot_.pose(armFrames_[k]).translation();
            const Eigen::Vector3d b = robot_.pose(armFrames_[k + 1]).translation();
            for (int s = 0; s <= 4; ++s)
            {
                const Eigen::Vector3d p = a + (b - a) * (s / 4.0);
                if (tower.contains(p, 0.02)) return false;     // 2 cm margin
                if (k >= 2 && p.z() < 0.03) return false;      // above the table
            }
        }

        // Franka Hand body: about 20 cm wide, 6 cm thick, 6.5 cm tall.
        const pinocchio::SE3& hand = robot_.pose(handId_);
        for (double x : {-0.03, 0.03})
            for (double y : {-0.10, 0.10})
                for (double z : {0.0, 0.065})
                {
                    const Eigen::Vector3d p = hand.act(Eigen::Vector3d(x, y, z));
                    if (tower.contains(p, 0.0) || p.z() < 0.005) return false;
                }
        return true;
    }

    RobotModel& robot_;
    InverseKinematics ik_;
    std::vector<pinocchio::FrameIndex> armFrames_;
    pinocchio::FrameIndex handId_;
    unsigned int seed_ = 1;
};
}  // namespace

int main(int argc, char** argv)
{
    RobotModel robot("models/fr3/fr3.urdf");
    std::string toolName = "standard Franka fingers";

    // ---- Optional custom finger tip ----
    if (argc >= 3)
    {
        const std::string mode = argv[1];
        const double length = std::stod(argv[2]);

        if (mode == "straight")        // tip further along the hand TCP Z axis
            robot.setToolOffset(pinocchio::SE3(Eigen::Matrix3d::Identity(),
                                               Eigen::Vector3d(0, 0, length)));
        else if (mode == "side")       // L-shape: tip along hand TCP X, approach along X
        {
            Eigen::Matrix3d R;
            R << 0, 0, 1,
                 0, 1, 0,
                -1, 0, 0;
            robot.setToolOffset(pinocchio::SE3(R, Eigen::Vector3d(length, 0, 0)));
        }
        else
        {
            std::cerr << "Usage: " << argv[0] << " [side|straight <length_m>]\n";
            return 1;
        }
        toolName = mode + " fingers, " + argv[2] + " m";
    }

    std::cout << "FR3 Jenga - Reachability Study (" << toolName << ")\n"
              << "Tower: " << LAYERS << " layers x 3 blocks\n\n"
              << " Pedestal  Distance  Yaw  Extractable  Layers 2-16\n";

    Study study(robot);
    int bestScore = -1;
    std::string bestLabel;
    std::vector<Result> best;

    for (double ped : PEDESTALS)
        for (double yaw : YAWS_DEG)
            for (double d : DISTANCES)
            {
                const std::vector<Result> r = study.evaluate(d, yaw, ped);
                int total = 0, playable = 0;
                for (const Result& b : r)
                {
                    total += b.extractable;
                    if (b.layer >= 2 && b.layer <= 16) playable += b.extractable;
                }

                std::cout << std::fixed << std::setprecision(2) << "  " << ped << " m    " << d
                          << " m   " << std::setw(2) << std::setprecision(0) << yaw << "   "
                          << std::setw(2) << total << " / 54     " << std::setw(2) << playable
                          << " / 45\n" << std::flush;

                if (playable > bestScore)
                {
                    bestScore = playable;
                    best = r;
                    std::ostringstream label;
                    label << std::fixed << std::setprecision(2) << d << " m, rotated " << std::setprecision(0)
                          << yaw << " deg, pedestal " << std::setprecision(2) << ped << " m";
                    bestLabel = label.str();
                }
            }

    // ---- Best placement in detail ----
    std::cout << "\nBEST PLACEMENT: " << bestLabel << "\n(o = extractable, x = not)\n";
    double worstMargin = 1e9, worstManip = 1e9;
    for (int layer = LAYERS; layer >= 1; --layer)
    {
        std::cout << "  Layer " << std::setw(2) << layer << ":  ";
        for (const Result& b : best)
            if (b.layer == layer)
            {
                std::cout << (b.extractable ? "o " : "x ");
                if (b.extractable)
                {
                    worstMargin = std::min(worstMargin, b.margin);
                    worstManip = std::min(worstManip, b.manipulability);
                }
            }
        std::cout << "\n";
    }
    std::cout << std::setprecision(3) << "  Smallest joint-limit margin: " << worstMargin
              << " rad\n  Smallest manipulability:     " << worstManip << "\n";

    std::ofstream csv("reachability_results.csv");
    csv << "layer,block,extractable,min_joint_margin_rad,min_manipulability\n";
    for (const Result& b : best)
        csv << b.layer << "," << b.block << "," << b.extractable << "," << b.margin << ","
            << b.manipulability << "\n";
    std::cout << "\nSaved: reachability_results.csv\n"
              << "NOTE: kinematic feasibility, approximate collision check only.\n";
    return 0;
}
