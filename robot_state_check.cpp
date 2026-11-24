// =====================================================
// FR3 Jenga — Robot state check (READ-ONLY, NO MOTION)
// =====================================================
// Connects to the FR3 through FCI (libfranka, no ROS), reads the state
// ONCE and disconnects. Never sends a motion command.
//
// Check: the robot reports its own flange and end-effector poses. We
// compute the same poses with OUR model from the robot's measured joint
// angles. Agreement within ~1 mm means our FK matches the real robot.
//
// Usage (lab PC, project root):  ./build/fr3_robot_state_check <robot-ip>
// Each run appends one line to robot_state_log.csv (one run per pose).

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "RobotModel.hpp"

namespace
{
// libfranka stores 4x4 matrices as 16 numbers, column-major (same as Eigen).
pinocchio::SE3 toSE3(const std::array<double, 16>& a)
{
    const Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(a.data());
    return pinocchio::SE3(T.block<3, 3>(0, 0), T.block<3, 1>(0, 3));
}

// Prints the comparison and returns {position error mm, rotation error deg}.
std::pair<double, double> compare(const std::string& name, const pinocchio::SE3& robot,
                                  const pinocchio::SE3& ours)
{
    const double mm = 1000.0 * (robot.translation() - ours.translation()).norm();
    const double deg = pinocchio::log3(robot.rotation().transpose() * ours.rotation()).norm()
                       * 180.0 / M_PI;
    std::cout << "\n" << name << "\n"
              << "  Robot reports [m]: " << robot.translation().transpose() << "\n"
              << "  Our model     [m]: " << ours.translation().transpose() << "\n"
              << "  Difference: " << mm << " mm, " << deg << " deg\n";
    return {mm, deg};
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-ip>\n";
        return 2;
    }
    std::cout << std::fixed << std::setprecision(5)
              << "FR3 Jenga - Robot State Check (READ-ONLY, no motion)\n";

    // ---- 1. Read the robot state once ----
    franka::RobotState state;
    try
    {
        // kIgnore: allows a non-real-time kernel; fine because we only READ.
        franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
        state = robot.readOnce();
    }
    catch (const franka::Exception& e)
    {
        std::cerr << "Could not read the robot state:\n  " << e.what() << "\n"
                  << "Check: IP address, network cable, FCI activated in Desk,\n"
                  << "       libfranka version matches the robot system version.\n";
        return 1;
    }

    JointVector q;
    for (int i = 0; i < 7; ++i) q[i] = state.q[i];
    std::cout << "Measured joints [rad]: " << q.transpose() << "\n";

    // ---- 2. Robot's reported poses ----
    const pinocchio::SE3 O_T_EE = toSE3(state.O_T_EE);   // base -> end effector
    const pinocchio::SE3 F_T_EE = toSE3(state.F_T_EE);   // flange -> end effector (Desk setting)
    const pinocchio::SE3 O_T_F = O_T_EE * F_T_EE.inverse();

    std::cout << "Desk end-effector offset [m]: " << F_T_EE.translation().transpose()
              << ", mass " << state.m_ee << " kg\n"
              << "(standard Franka Hand: about 0 0 0.1034, about 0.73 kg)\n";

    // ---- 3. Our model for the same joints, and the comparison ----
    RobotModel model("models/fr3/fr3.urdf");
    model.update(q);

    const auto [flangeMm, flangeDeg] =
        compare("FLANGE (independent of gripper settings):", O_T_F,
                model.pose(model.frameId("fr3_link8")));
    const auto [eeMm, eeDeg] =
        compare("END EFFECTOR (meaningful only if Desk is set to the Franka Hand):", O_T_EE,
                model.pose(model.frameId("fr3_hand_tcp")));

    const bool ok = flangeMm < 1.0 && flangeDeg < 0.5;
    std::cout << "\n" << (ok ? "PASS" : "FAIL")
              << ": flange pose matches our model (< 1 mm, < 0.5 deg)\n";

    // ---- 4. Log for the report ----
    const bool newFile = !std::ifstream("robot_state_log.csv").good();
    std::ofstream log("robot_state_log.csv", std::ios::app);
    if (newFile) log << "q1,q2,q3,q4,q5,q6,q7,flange_mm,flange_deg,ee_mm,ee_deg\n";
    for (int i = 0; i < 7; ++i) log << q[i] << ",";
    log << flangeMm << "," << flangeDeg << "," << eeMm << "," << eeDeg << "\n";
    std::cout << "Logged to robot_state_log.csv\n";

    return ok ? 0 : 1;
}
