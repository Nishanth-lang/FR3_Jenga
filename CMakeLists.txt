cmake_minimum_required(VERSION 3.16)
project(FR3_Jenga CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimized build by default (about 10x faster than an unoptimized build).
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Franka QUIET)   # only on the lab PC

# Helper: one program = one .cpp file in apps/, using our headers.
function(fr3_program name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(${name} PRIVATE Eigen3::Eigen pinocchio::pinocchio ${ARGN})
endfunction()

fr3_program(fr3_jenga              apps/main.cpp)
fr3_program(fr3_kinematics_tests   apps/kinematics_tests.cpp)
fr3_program(fr3_reachability_study apps/reachability_study.cpp)

# Programs that talk to the real FR3 (need libfranka, lab PC only).
#   fr3_robot_state_check : READ-ONLY, never moves the robot
#   fr3_small_move        : MOVES joint 7 by 0.1 rad and back (supervised only)
if(Franka_FOUND)
    message(STATUS "libfranka ${Franka_VERSION} found: building robot programs")
    fr3_program(fr3_robot_state_check apps/robot_state_check.cpp Franka::Franka)
    fr3_program(fr3_small_move        apps/small_move.cpp        Franka::Franka)
    fr3_program(fr3_ik_move           apps/ik_move.cpp           Franka::Franka)
else()
    message(STATUS "libfranka not found: skipping robot programs")
endif()
