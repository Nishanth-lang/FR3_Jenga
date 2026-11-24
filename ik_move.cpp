// =====================================================
// FR3 Jenga — IK demos on the real robot
// =====================================================
// MOVES THE ROBOT. Supervisor present, stop button in hand, area clear.
//
// Modes:
//   ./build/fr3_ik_move <ip> goto <x> <y> <z>
//       Straight line to (x, y, z) [m, robot base frame], same hand orientation.
//
//   ./build/fr3_ik_move <ip> elbow <amount>
//       Hand pose FIXED, elbow swings (redundancy). <amount> max 1.5.
//
//   ./build/fr3_ik_move <ip> orbit <radius> <angle_deg> <height> [weighted]
//       Virtual tower: a vertical axis <radius> m in front of the hand
//       (outward from the robot). The hand moves around it along an arc
//       (+angle, -angle, back to the middle), then up by <height> m and back
//       down, keeping its orientation relative to the tower.
//       "weighted": shoulder joints 1-3 cost 10x more -> the IK uses the
//       redundancy so the wrist does most of the work.
//
//   ./build/fr3_ik_move <ip> square <half_size> <down> [turn] [weighted]
//       Path AROUND a virtual tower: a square with rounded corners
//       (side 2 x half_size), the hand's current position in the middle of
//       the near side, tower centre half_size in front. Visits the middle of
//       all 4 sides (counter-clockwise); at each side: down <down> m and up.
//       Gripper keeps pointing down. "turn": gripper turns to stay parallel
//       to each side (+90 / -90 deg alternately, using the 180-deg symmetry
//       of the parallel gripper; not valid for asymmetric fingers).
//
//   ./build/fr3_ik_move <ip> home
//       Slow joint move to the project HOME pose (RobotModel::homePose()):
//       gripper pointing down at about (0.30, 0, 0.55) m, clear of the tower,
//       far from joint limits and singularities. Start every session here.
//
//   Use "offline" instead of <ip> to only print the plan (no robot needed;
//   offline plans start from HOME).

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "InverseKinematics.hpp"
#include "RobotModel.hpp"

namespace
{
// ---- Safety limits ----
const double MAX_DISTANCE = 0.20;      // m, goto: longest straight move
const double MIN_HEIGHT = 0.15;        // m, hand never below this (robot base frame)
const double MAX_ELBOW = 1.5;          // elbow: largest joint-3 nudge
const double MAX_RADIUS = 0.30;        // m, orbit radius
const double MAX_ANGLE = 60.0;         // deg, orbit arc to each side
const double MAX_UPDOWN = 0.15;        // m, orbit vertical travel
const double MAX_HALF_SIZE = 0.20;     // m, square: half the side length
const double CORNER_RADIUS = 0.05;     // m, square: rounded corners
const double MAX_JOINT_CHANGE = 2.2;   // rad, largest change of any joint in one segment
const double MIN_MARGIN = 0.1;         // rad, distance to joint limits
const double MAX_STEP = 0.05;          // rad, joint change between waypoints
const double MAX_JOINT_SPEED = 0.2;    // rad/s (peak)
const double MAX_HAND_SPEED = 0.05;    // m/s (peak)

// One smooth motion: starts and ends at rest.
struct Segment
{
    std::string name;
    std::vector<pinocchio::SE3> poses;   // target hand poses (empty for elbow mode)
    std::vector<JointVector> path;       // joint configurations from IK
    std::vector<double> progress;        // 0..1 for each waypoint (time share)
    double handLength = 0.0;             // m
    double duration = 0.0;               // s
};

double smoothStep(double x)
{
    x = std::max(0.0, std::min(1.0, x));
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

// Joint angles at progress s in [0, 1]. Each waypoint has its own progress
// value, so parts of the path where the joints move a lot get more time.
JointVector sample(const Segment& seg, double s)
{
    const std::vector<double>& p = seg.progress;
    s = std::max(0.0, std::min(1.0, s));
    const std::size_t i = std::min<std::size_t>(
        std::upper_bound(p.begin(), p.end(), s) - p.begin(), p.size() - 1);
    if (i == 0) return seg.path.front();
    const double span = p[i] - p[i - 1];
    const double a = span > 0.0 ? (s - p[i - 1]) / span : 1.0;

    // Smooth curve through the waypoints (Catmull-Rom spline): no sudden
    // velocity changes at waypoints, unlike straight lines between them.
    const std::size_t n = seg.path.size();
    const JointVector& P0 = seg.path[i >= 2 ? i - 2 : 0];
    const JointVector& P1 = seg.path[i - 1];
    const JointVector& P2 = seg.path[i];
    const JointVector& P3 = seg.path[std::min(i + 1, n - 1)];
    return 0.5 * (2.0 * P1 + (P2 - P0) * a + (2.0 * P0 - 5.0 * P1 + 4.0 * P2 - P3) * a * a +
                  (3.0 * P1 - P0 - 3.0 * P2 + P3) * a * a * a);
}

pinocchio::SE3 toSE3(const std::array<double, 16>& a)
{
    const Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(a.data());
    return pinocchio::SE3(T.block<3, 3>(0, 0), T.block<3, 1>(0, 3));
}

// IK for every pose of a segment, each starting from the previous solution.
void solveSegment(InverseKinematics& ik, Segment& seg, const JointVector& start)
{
    seg.path = {start};
    for (std::size_t k = 0; k < seg.poses.size(); ++k)
    {
        JointVector q;
        if (!ik.solveFrom(seg.poses[k], seg.path.back(), q))
            throw std::runtime_error("IK failed in segment '" + seg.name + "' at point " +
                                     std::to_string(k + 1));
        seg.path.push_back(q);
    }
}

// Safety checks + duration. Returns total absolute motion of each joint.
JointVector checkSegment(RobotModel& model, Segment& seg)
{
    JointVector travel = JointVector::Zero();
    for (std::size_t k = 1; k < seg.path.size(); ++k)
    {
        const JointVector step = (seg.path[k] - seg.path[k - 1]).cwiseAbs();
        travel += step;
        if (step.maxCoeff() > MAX_STEP)
            throw std::runtime_error("joint jump too large in segment '" + seg.name + "'");
        // Near a limit is only allowed while moving AWAY from it.
        const double margin = model.jointLimitMargin(seg.path[k]);
        if (margin < MIN_MARGIN && margin <= model.jointLimitMargin(seg.path[k - 1]))
            throw std::runtime_error("segment '" + seg.name + "' moves towards a joint limit (< 0.1 rad)");
        if (model.toolPose(seg.path[k]).translation().z() < MIN_HEIGHT)
            throw std::runtime_error("segment '" + seg.name + "' goes below the minimum height");
    }
    if (seg.name != "to HOME" &&
        (seg.path.back() - seg.path.front()).cwiseAbs().maxCoeff() > MAX_JOINT_CHANGE)
        throw std::runtime_error("a joint changes by more than 2.2 rad in segment '" + seg.name + "'");

    // Timing: each step costs the time it needs at the speed limits
    // (joints and hand). Progress = accumulated cost -> slow where the joints
    // move a lot (e.g. corners), faster on easy stretches.
    const std::size_t n = seg.path.size();
    std::vector<double> stepCost(n, 0.0);
    for (std::size_t k = 1; k < n; ++k)
    {
        const double jointTime = (seg.path[k] - seg.path[k - 1]).cwiseAbs().maxCoeff() / MAX_JOINT_SPEED;
        const double handTime = (model.toolPose(seg.path[k]).translation() -
                                 model.toolPose(seg.path[k - 1]).translation()).norm() / MAX_HAND_SPEED;
        stepCost[k] = std::max({jointTime, handTime, 1e-6});
    }
    // Smooth the timing (sliding maximum, then average over neighbours) so the
    // speed along the path changes gradually. This changes WHEN the robot is
    // where, never WHERE it goes.
    const int w = 20;
    std::vector<double> smoothCost(n, 0.0);
    for (int pass = 0; pass < 2; ++pass)
    {
        for (std::size_t k = 1; k < n; ++k)
        {
            const std::size_t lo = (k > static_cast<std::size_t>(w)) ? k - w : 1;
            const std::size_t hi = std::min(n - 1, k + w);
            double v = 0.0;
            for (std::size_t j = lo; j <= hi; ++j)
                v = (pass == 0) ? std::max(v, stepCost[j]) : v + stepCost[j] / (hi - lo + 1);
            smoothCost[k] = v;
        }
        stepCost = smoothCost;
    }
    seg.progress.assign(n, 0.0);
    double cost = 0.0;
    for (std::size_t k = 1; k < n; ++k)
    {
        cost += stepCost[k];
        seg.progress[k] = cost;
    }
    for (double& p : seg.progress) p /= cost;
    // Smooth start/stop profile has a peak speed factor of 1.875.
    seg.duration = std::max(3.0, 1.875 * cost);
    return travel;
}

// Execute one segment on the robot (smooth start and stop).
void play(franka::Robot& robot, const Segment& seg)
{
    std::array<double, 7> initial{};
    double time = 0.0;
    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::JointPositions {
        time += period.toSec();
        if (time == 0.0) initial = state.q_d;

        const JointVector delta = sample(seg, smoothStep(time / seg.duration)) - seg.path.front();
        std::array<double, 7> command;
        for (int i = 0; i < 7; ++i) command[i] = initial[i] + delta[i];

        franka::JointPositions output(command);
        if (time >= seg.duration) return franka::MotionFinished(output);
        return output;
    });
}

void printVector(const std::string& label, const JointVector& v)
{
    std::cout << label << v.transpose() << "\n";
}
}  // namespace

int main(int argc, char** argv)
{
    std::cout << std::fixed << std::setprecision(4);

    if (argc < 3)
    {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " <ip|offline> goto <x> <y> <z>\n"
                  << "  " << argv[0] << " <ip|offline> elbow <amount>\n"
                  << "  " << argv[0] << " <ip|offline> orbit <radius> <angle_deg> <height> [weighted]\n"
                  << "  " << argv[0] << " <ip|offline> square <half_size> <down> [turn] [weighted]\n"
                  << "  " << argv[0] << " <ip|offline> home\n";
        return 2;
    }

    const std::string ip = argv[1];
    const std::string mode = argv[2];
    const bool offline = (ip == "offline");

    try
    {
        // ---- 1. Current joint angles ----
        JointVector q0;
        std::unique_ptr<franka::Robot> robot;
        const JointVector home = RobotModel::homePose();

        if (offline)
            q0 = home;
        else
        {
            robot = std::make_unique<franka::Robot>(ip, franka::RealtimeConfig::kIgnore);
            const franka::RobotState state = robot->readOnce();
            for (int i = 0; i < 7; ++i) q0[i] = state.q[i];
        }

        RobotModel model("models/fr3/fr3.urdf");
        InverseKinematics ik(model);
        const pinocchio::SE3 start = model.toolPose(q0);
        const int N = 100;   // waypoints per segment

        // ---- 2. Build the segments (target hand poses) ----
        std::vector<Segment> segments;

        if (mode == "goto" && argc == 6)
        {
            const Eigen::Vector3d goal(std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]));
            Segment seg{"straight line"};
            seg.handLength = (goal - start.translation()).norm();
            if (seg.handLength > MAX_DISTANCE)
                throw std::runtime_error("target is more than 0.20 m away");
            if (goal.z() < MIN_HEIGHT)
                throw std::runtime_error("target below the minimum height");
            for (int k = 1; k <= N; ++k)
            {
                const double s = static_cast<double>(k) / N;
                seg.poses.emplace_back(start.rotation(), (1 - s) * start.translation() + s * goal);
            }
            segments.push_back(seg);
        }
        else if (mode == "orbit" && (argc == 6 || argc == 7))
        {
            const double radius = std::stod(argv[3]);
            const double angle = std::stod(argv[4]) * M_PI / 180.0;
            const double height = std::stod(argv[5]);
            const bool weighted = (argc == 7 && std::string(argv[6]) == "weighted");

            if (radius <= 0.0 || radius > MAX_RADIUS) throw std::runtime_error("radius must be 0-0.30 m");
            if (std::abs(angle) > MAX_ANGLE * M_PI / 180.0) throw std::runtime_error("angle max 60 deg");
            if (std::abs(height) > MAX_UPDOWN) throw std::runtime_error("height max 0.15 m");

            // Virtual tower axis: 'radius' in front of the hand, outward from the robot base.
            Eigen::Vector3d outward(start.translation().x(), start.translation().y(), 0.0);
            outward.normalize();
            const Eigen::Vector3d center = start.translation() + radius * outward;

            // Hand pose rotated by phi about the vertical tower axis, then raised by dz.
            auto orbitPose = [&](double phi, double dz) {
                const Eigen::Matrix3d Rz =
                    Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                return pinocchio::SE3(Rz * start.rotation(),
                                      center + Rz * (start.translation() - center) +
                                          Eigen::Vector3d(0, 0, dz));
            };
            auto arc = [&](const std::string& name, double from, double to) {
                Segment seg{name};
                seg.handLength = radius * std::abs(to - from);
                for (int k = 1; k <= N; ++k)
                    seg.poses.push_back(orbitPose(from + (to - from) * k / N, 0.0));
                return seg;
            };
            auto vertical = [&](const std::string& name, double from, double to) {
                Segment seg{name};
                seg.handLength = std::abs(to - from);
                for (int k = 1; k <= N; ++k)
                    seg.poses.push_back(orbitPose(0.0, from + (to - from) * k / N));
                return seg;
            };

            segments = {arc("arc to +side", 0.0, angle), arc("arc to -side", angle, -angle),
                        arc("arc to middle", -angle, 0.0), vertical("up", 0.0, height),
                        vertical("down", height, 0.0)};

            std::cout << "Virtual tower axis at x = " << center.x() << ", y = " << center.y()
                      << " (radius " << radius << " m)\n";

            // Redundancy comparison: plan with standard and with weighted IK.
            auto totalTravel = [&](std::vector<Segment> segs) {
                JointVector total = JointVector::Zero();
                JointVector q = q0;
                for (Segment& s : segs)
                {
                    solveSegment(ik, s, q);
                    total += checkSegment(model, s);
                    q = s.path.back();
                }
                return total;
            };

            JointVector shoulderHeavy;
            shoulderHeavy << 10, 10, 10, 1, 1, 1, 1;

            ik.settings.weights = JointVector::Ones();
            const JointVector standard = totalTravel(segments);
            ik.settings.weights = shoulderHeavy;
            const JointVector weightedTravel = totalTravel(segments);

            std::cout << "\nREDUNDANCY COMPARISON: total motion of each joint [rad]\n"
                      << "                 J1      J2      J3      J4      J5      J6      J7\n";
            printVector("  standard IK: ", standard);
            printVector("  weighted IK: ", weightedTravel);
            std::cout << std::setprecision(1) << "  Shoulder (J1-J3) motion: standard "
                      << standard.head<3>().sum() << " rad, weighted " << weightedTravel.head<3>().sum()
                      << " rad (" << 100.0 * (1.0 - weightedTravel.head<3>().sum() / standard.head<3>().sum())
                      << " % less)\n" << std::setprecision(4);

            ik.settings.weights = weighted ? shoulderHeavy : JointVector::Ones();
            std::cout << "  -> executing with " << (weighted ? "WEIGHTED" : "STANDARD") << " IK\n";
        }
        else if (mode == "square" && argc >= 5)
        {
            const double half = std::stod(argv[3]);
            const double down = std::stod(argv[4]);
            bool turn = false, weighted = false;
            for (int a = 5; a < argc; ++a)
            {
                if (std::string(argv[a]) == "turn") turn = true;
                if (std::string(argv[a]) == "weighted") weighted = true;
            }
            if (half <= 0.06 || half > MAX_HALF_SIZE) throw std::runtime_error("half_size must be 0.06-0.20 m");
            if (down < 0.0 || down > MAX_UPDOWN) throw std::runtime_error("down must be 0-0.15 m");

            // Square axes: u = outward from the robot, v = to the left.
            Eigen::Vector3d u(start.translation().x(), start.translation().y(), 0.0);
            u.normalize();
            const Eigen::Vector3d v = Eigen::Vector3d::UnitZ().cross(u);
            const Eigen::Vector3d center = start.translation() + half * u;   // virtual tower axis
            const double r = std::min(CORNER_RADIUS, 0.5 * half);

            // Walk around the square counter-clockwise (seen from above), starting
            // in the middle of the near side heading right (-v). The path is cut
            // into 4 travel pieces: middle of one side -> corner -> middle of the
            // next side. At the middle of every side: down and back up.
            //
            // "turn": the gripper turns to stay parallel to each side. Because the
            // parallel gripper looks the same after 180 deg, it turns +90 deg at
            // one corner and -90 deg at the next (wrist only swings +-90 deg).
            double px = -half, py = 0.0, heading = -M_PI / 2, yaw = 0.0;
            const double ds = 0.002;   // 2 mm between waypoints
            double length = 0.0;

            auto pose = [&]() {
                const Eigen::Vector3d p = center + px * u + py * v;
                const Eigen::Matrix3d R = turn
                    ? Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * start.rotation())
                    : start.rotation();
                return pinocchio::SE3(R, p);
            };
            auto straight = [&](Segment& seg, double len) {
                const int n = std::max(1, static_cast<int>(std::ceil(len / ds)));
                for (int k = 0; k < n; ++k)
                {
                    px += std::cos(heading) * len / n;
                    py += std::sin(heading) * len / n;
                    seg.poses.push_back(pose());
                }
                seg.handLength += len;
            };
            auto corner = [&](Segment& seg, double turnSign) {   // quarter circle to the left
                const double cx = px - std::sin(heading) * r, cy = py + std::cos(heading) * r;
                const int n = std::max(turn ? 120 : 20, static_cast<int>(std::ceil(r * M_PI / 2 / ds)));
                const double a0 = std::atan2(py - cy, px - cx), yaw0 = yaw;
                for (int k = 1; k <= n; ++k)
                {
                    px = cx + r * std::cos(a0 + (M_PI / 2) * k / n);
                    py = cy + r * std::sin(a0 + (M_PI / 2) * k / n);
                    heading += (M_PI / 2) / n;
                    // Gripper turn eases in and out (no sudden wrist start/stop).
                    yaw = yaw0 + turnSign * (M_PI / 2) * smoothStep(static_cast<double>(k) / n);
                    seg.poses.push_back(pose());
                }
                seg.handLength += r * M_PI / 2;
            };
            auto upDown = [&](const std::string& side) {
                if (down <= 0.0) return;
                const pinocchio::SE3 top = pose();
                Segment dn{"down (" + side + ")"}, upSeg{"up (" + side + ")"};
                dn.handLength = upSeg.handLength = down;
                for (int k = 1; k <= N; ++k)
                {
                    dn.poses.emplace_back(top.rotation(), top.translation() - Eigen::Vector3d(0, 0, down * k / N));
                    upSeg.poses.emplace_back(top.rotation(),
                                             top.translation() - Eigen::Vector3d(0, 0, down * (N - k) / N));
                }
                segments.push_back(dn);
                segments.push_back(upSeg);
            };

            const char* sideName[4] = {"near side", "right side", "far side", "left side"};
            upDown(sideName[0]);
            for (int side = 0; side < 4; ++side)
            {
                Segment travel{std::string("to ") + sideName[(side + 1) % 4]};
                straight(travel, half - r);
                corner(travel, (side % 2 == 0) ? 1.0 : -1.0);
                straight(travel, half - r);
                length += travel.handLength;
                segments.push_back(travel);
                if (side < 3) upDown(sideName[side + 1]);
            }

            std::cout << "Virtual tower axis at x = " << center.x() << ", y = " << center.y()
                      << "; square " << 2 * half << " x " << 2 * half << " m, corner radius " << r
                      << " m, loop length " << length << " m"
                      << (turn ? ", gripper turns with each side" : ", gripper orientation fixed")
                      << ", down/up " << down << " m at each side\n";

            if (weighted)
            {
                JointVector shoulderHeavy;
                shoulderHeavy << 10, 10, 10, 1, 1, 1, 1;
                ik.settings.weights = shoulderHeavy;
            }
        }
        else if (mode == "home" && argc == 3)
        {
            // Joint-space move: straight line in joint angles (no IK needed).
            Segment seg{"to HOME"};
            const int M = std::max(N, static_cast<int>(
                std::ceil((home - q0).cwiseAbs().maxCoeff() / (0.5 * MAX_STEP))));
            seg.path = {q0};
            for (int k = 1; k <= M; ++k)
                seg.path.push_back(q0 + (home - q0) * (static_cast<double>(k) / M));
            segments.push_back(seg);
        }
        else if (mode == "elbow" && argc == 4)
        {
            const double amount = std::stod(argv[3]);
            if (std::abs(amount) > MAX_ELBOW) throw std::runtime_error("elbow amount max 1.5");
            Segment seg{"elbow swing"};
            seg.path = {q0};
            for (int k = 1; k <= N; ++k)
            {
                JointVector guess = seg.path.back();
                guess[2] += amount / N;
                JointVector q;
                if (!ik.solveFrom(start, guess, q))
                    throw std::runtime_error("IK failed at point " + std::to_string(k));
                seg.path.push_back(q);
            }
            segments.push_back(seg);
        }
        else
        {
            std::cerr << "Unknown mode or wrong number of arguments.\n";
            return 2;
        }

        // ---- 3. Final plan with the chosen IK, plus checks ----
        JointVector q = q0;
        double totalTime = 0.0;
        std::cout << "\nPLAN (" << mode << ")\n";
        for (Segment& seg : segments)
        {
            if (!seg.poses.empty())
            {
                solveSegment(ik, seg, q);
            }
            checkSegment(model, seg);
            const pinocchio::SE3 end = model.toolPose(seg.path.back());
            std::cout << "  " << std::left << std::setw(15) << seg.name << std::right
                      << " hand -> " << end.translation().transpose()
                      << "   largest joint change "
                      << (seg.path.back() - seg.path.front()).cwiseAbs().maxCoeff() << " rad, "
                      << std::setprecision(1) << seg.duration << " s\n" << std::setprecision(4);
            q = seg.path.back();
            totalTime += seg.duration;
        }
        std::cout << "  Total time: " << std::setprecision(1) << totalTime << " s\n" << std::setprecision(4);

        if (offline)
        {
            std::cout << "OFFLINE: plan only, nothing sent to a robot.\n";
            return 0;
        }

        std::cout << "\nStop button in hand, area clear? Type 'yes' to move: ";
        std::string answer;
        std::getline(std::cin, answer);
        if (answer != "yes")
        {
            std::cout << "Cancelled. Nothing moved.\n";
            return 0;
        }

        // ---- 4. Execute, segment by segment ----
        robot->setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                                    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                                    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                                    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        for (const Segment& seg : segments)
        {
            std::cout << "  moving: " << seg.name << " ..." << std::flush;
            play(*robot, seg);

            const franka::RobotState now = robot->readOnce();
            const pinocchio::SE3 reached = toSE3(now.O_T_EE);
            const pinocchio::SE3 planned = model.toolPose(seg.path.back());
            std::cout << " done, hand error "
                      << 1000.0 * (reached.translation() - planned.translation()).norm() << " mm\n";
        }

        // ---- 5. Final check with the robot's own measurement ----
        const franka::RobotState after = robot->readOnce();
        const pinocchio::SE3 reached = toSE3(after.O_T_EE);
        const pinocchio::SE3 target = model.toolPose(segments.back().path.back());
        JointVector q1;
        for (int i = 0; i < 7; ++i) q1[i] = after.q[i];

        std::cout << "\nRESULT (measured by the robot)\n";
        printVector("  Joints before [rad]: ", q0);
        printVector("  Joints after  [rad]: ", q1);
        std::cout << "  Hand reached [m]: " << reached.translation().transpose() << "\n"
                  << "  Hand target  [m]: " << target.translation().transpose() << "\n"
                  << "  Position error: " << 1000.0 * (reached.translation() - target.translation()).norm()
                  << " mm\n";
    }
    catch (const franka::Exception& e)
    {
        std::cerr << "Robot stopped: " << e.what() << "\n";
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ABORT (nothing moved): " << e.what() << "\n";
        return 1;
    }
    return 0;
}
