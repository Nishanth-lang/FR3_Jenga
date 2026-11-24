# FR3 Jenga

Standalone C++17 application (no ROS) for a Franka Research 3 robot that
extracts Jenga blocks. Libraries: Eigen (maths), Pinocchio (robot model),
libfranka (robot connection, lab PC only).

## Structure

```text
FR3_Jenga/
├── CMakeLists.txt               build (optimized Release by default)
├── models/fr3/fr3.urdf          robot description (links, joints, limits)
├── include/
│   ├── RobotModel.hpp           THE robot: loads the URDF once; FK, Jacobian,
│   │                            joint limits, custom tool tip
│   ├── InverseKinematics.hpp    tool pose -> joint angles (damped least squares,
│   │                            restarts); uses RobotModel
│   └── CoordinateTransform.hpp  camera coordinates -> robot coordinates
└── apps/
    ├── main.cpp                 coordinate-transform demo (future Jenga program)
    ├── kinematics_tests.cpp     all validation tests: FK, IK, Jacobian, robustness
    ├── reachability_study.cpp   which blocks the robot can extract, per tower placement
    ├── robot_state_check.cpp    READ-ONLY: real robot pose vs our model (lab PC)
    └── small_move.cpp           FIRST MOTION: joint 7 +0.1 rad and back (lab PC, supervised)
```

```text
fr3.urdf ──► RobotModel ──► InverseKinematics
                 │                 │
                 ├─────────────────┼──► kinematics_tests
                 ├─────────────────┴──► reachability_study
                 └────────────────────► robot_state_check ──► FR3 (libfranka, read-only)

CoordinateTransform ──► main
```

## Build and run (from this folder)

```bash
cmake -S . -B build
cmake --build build -j$(nproc)

./build/fr3_jenga                              # coordinate transform demo
./build/fr3_kinematics_tests                   # all kinematics checks, < 1 s
./build/fr3_reachability_study side 0.10       # L-shaped fingers, about 1 min
./build/fr3_robot_state_check <robot-ip>       # lab PC only, READ-ONLY
./build/fr3_small_move <robot-ip>              # lab PC only, MOVES the wrist (supervised)
```

## Verified results

| Check | Result |
|---|---|
| FK, reference pose q = [0, -0.5, 0, -1.5, 0, 1, 0.5] | TCP at (0.287452, 0, 0.716366) m |
| IK, same target | converged in 111 iterations; error 4 µm / 9 µrad |
| Jacobian vs finite differences | max error 5.7e-11 |
| x_dot = J q_dot | relative error 9.9e-7 |
| Singularity (elbow nearly straight) | smallest singular value 0.140 -> 0.049 |
| IK on 50 random targets | 43/50 single start, 50/50 with restarts |
| Unreachable targets | 3/3 rejected |
| Reachability, L-shaped fingers (10 cm), tower 0.45 m, rotated 45°, 10 cm pedestal | 54/54 blocks |

## Limitations

- Camera transform uses a hardcoded example, not a real calibration.
- Reachability: approximate collision check, kinematics only (no forces,
  friction, tower stability). Finger tip height assumed equal to the
  standard Franka Hand TCP.
- Only fr3_small_move sends motion commands: joint 7 only, 0.1 rad, about 0.03 rad/s,
  asks for confirmation, sensitive collision limits. Supervised use only.
