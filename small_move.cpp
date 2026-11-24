// =====================================================
// FR3 Jenga — First motion test: tiny wrist rotation
// =====================================================
// MOVES THE ROBOT. Only use with a supervisor present, the
// enable/stop button in your hand, and a clear workspace.
//
// What it does:
//   Rotates ONLY joint 7 (the wrist) by +0.1 rad (about 6 degrees)
//   over 6 seconds, then back to the start over 6 seconds.
//   The hand position does not change, it only turns in place.
//   Peak speed about 0.03 rad/s. Sensitive collision limits.
//
// Smooth motion profile (quintic): s(t) = 10t^3 - 15t^4 + 6t^5
//   -> zero speed AND zero acceleration at start and end (no jerk).
//
// Usage (lab PC, project root):  ./build/fr3_small_move <robot-ip>

#include <array>
#include <cmath>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

#include "RobotModel.hpp"

namespace
{
const int JOINT = 6;              // index 6 = joint 7 (wrist rotation)
const double DELTA = 0.1;         // rad (about 6 degrees)
const double DURATION = 6.0;      // seconds for each direction

// Smooth 0 -> 1 progress with zero velocity and acceleration at both ends.
double smoothStep(double x)
{
    x = std::max(0.0, std::min(1.0, x));
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-ip>\n";
        return 2;
    }

    try
    {
        // kIgnore: no real-time kernel required. If the robot stops with
        // "communication_constraints_violation", the PC is not keeping up.
        franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);

        // ---- 1. Read the start position and check joint limits ----
        const franka::RobotState start = robot.readOnce();
        JointVector q;
        for (int i = 0; i < 7; ++i) q[i] = start.q[i];

        RobotModel model("models/fr3/fr3.urdf");
        JointVector target = q;
        target[JOINT] += DELTA;

        if (model.jointLimitMargin(target) < 0.1)
        {
            std::cerr << "ABORT: joint 7 would come within 0.1 rad of its limit.\n"
                      << "Hand-guide the wrist away from its limit and try again.\n";
            return 1;
        }

        std::cout << "PLAN: rotate joint 7 from " << q[JOINT] << " to " << target[JOINT]
                  << " rad over " << DURATION << " s, then back.\n"
                  << "Only the wrist turns. Keep the enable/stop button in your hand.\n"
                  << "Type 'yes' and press Enter to move: ";
        std::string answer;
        std::getline(std::cin, answer);
        if (answer != "yes")
        {
            std::cout << "Cancelled. Nothing moved.\n";
            return 0;
        }

        // ---- 2. Safety settings ----
        // Collision thresholds (Nm per joint, N/Nm for the end effector):
        // values from Franka's own examples. Contact above them stops the robot.
        robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                                   {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                                   {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                                   {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        // ---- 3. Motion: there (6 s) and back (6 s) ----
        std::array<double, 7> initial{};
        double time = 0.0;

        robot.control([&](const franka::RobotState& state,
                          franka::Duration period) -> franka::JointPositions {
            time += period.toSec();
            if (time == 0.0) initial = state.q_d;   // exact commanded start

            // 0 -> 1 during the first half, 1 -> 0 during the second half.
            const double s = (time <= DURATION) ? smoothStep(time / DURATION)
                                                : 1.0 - smoothStep((time - DURATION) / DURATION);

            std::array<double, 7> command = initial;
            command[JOINT] = initial[JOINT] + DELTA * s;

            franka::JointPositions output(command);
            if (time >= 2.0 * DURATION) return franka::MotionFinished(output);
            return output;
        });

        std::cout << "Motion finished. Robot back at its start position.\n";
    }
    catch (const franka::Exception& e)
    {
        std::cerr << "Robot stopped: " << e.what() << "\n"
                  << "(The robot stops safely on any error. If this says\n"
                  << " communication_constraints_violation, the PC is too slow\n"
                  << " without a real-time kernel.)\n";
        return 1;
    }
    return 0;
}
