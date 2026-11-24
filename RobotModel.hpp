#pragma once
// =====================================================
// RobotModel — the ONE place that loads the FR3 model.
// =====================================================
// Everything that needs the robot (IK, tests, planners) uses this
// class instead of loading the URDF itself.
//
// Provides:
//   - forward kinematics: pose of any frame for 7 arm joint angles
//   - the 6x7 arm Jacobian
//   - joint limits
//   - the tool frame (standard hand TCP, or a custom finger tip)

#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

// Seven arm joint angles (fixed size: faster and size-checked by the compiler).
using JointVector = Eigen::Matrix<double, 7, 1>;

class RobotModel
{
public:
    explicit RobotModel(const std::string& urdfPath)
    {
        pinocchio::urdf::buildModel(urdfPath, model_);

        // FR3 URDF: 7 arm joints + 2 finger joints.
        if (model_.nq != 9 || model_.nv != 9)
            throw std::runtime_error("Unexpected FR3 model: expected nq = nv = 9");

        data_ = pinocchio::Data(model_);
        toolId_ = frameId("fr3_hand_tcp");
    }

    // ---- Frames -------------------------------------

    // Frame index by name; throws if the name does not exist.
    pinocchio::FrameIndex frameId(const std::string& name) const
    {
        if (!model_.existFrame(name))
            throw std::runtime_error("Frame not found: " + name);
        return model_.getFrameId(name);
    }

    pinocchio::FrameIndex toolId() const { return toolId_; }

    // Use a tool tip offset from fr3_hand_tcp (e.g. extended fingers).
    // The offset is expressed in the fr3_hand_tcp frame.
    void setToolOffset(const pinocchio::SE3& handTcpToTool)
    {
        const pinocchio::FrameIndex handTcp = frameId("fr3_hand_tcp");
        const pinocchio::Frame& hand = model_.frames[handTcp];

#if PINOCCHIO_VERSION_AT_LEAST(3, 0, 0)
        const pinocchio::JointIndex parent = hand.parentJoint;
#else
        const pinocchio::JointIndex parent = hand.parent;   // Pinocchio 2.x name
#endif
        const pinocchio::Frame tool("jenga_tool_tip", parent, handTcp,
                                    hand.placement * handTcpToTool,
                                    pinocchio::OP_FRAME);

        if (model_.existFrame("jenga_tool_tip"))
            model_.frames[model_.getFrameId("jenga_tool_tip")] = tool;
        else
            model_.addFrame(tool);

        data_ = pinocchio::Data(model_);   // data must match the model
        toolId_ = frameId("jenga_tool_tip");
    }

    // ---- Kinematics ---------------------------------

    // Forward kinematics for ALL frames at once; read them with pose().
    void update(const JointVector& arm)
    {
        pinocchio::forwardKinematics(model_, data_, fullConfiguration(arm));
        pinocchio::updateFramePlacements(model_, data_);
    }

    // Pose of a frame (in the robot base frame) from the last update().
    const pinocchio::SE3& pose(pinocchio::FrameIndex frame) const
    {
        return data_.oMf[frame];
    }

    // Forward kinematics for one frame.
    pinocchio::SE3 framePose(const JointVector& arm, pinocchio::FrameIndex frame)
    {
        update(arm);
        return pose(frame);
    }

    pinocchio::SE3 toolPose(const JointVector& arm)
    {
        return framePose(arm, toolId_);
    }

    // 6x7 arm Jacobian of the tool frame.
    // Rows: 3 linear + 3 angular velocity. Columns: 7 arm joints.
    // LOCAL = expressed in the tool's own axes (used by IK).
    // LOCAL_WORLD_ALIGNED = expressed in the robot base axes.
    Eigen::Matrix<double, 6, 7> jacobian(
        const JointVector& arm,
        pinocchio::ReferenceFrame reference = pinocchio::LOCAL)
    {
        Eigen::Matrix<double, 6, Eigen::Dynamic> full(6, model_.nv);
        full.setZero();
        pinocchio::computeFrameJacobian(model_, data_, fullConfiguration(arm),
                                        toolId_, reference, full);
        return full.leftCols<7>();   // drop the two finger columns
    }

    // ---- Project HOME pose ---------------------------
    // Starting pose for every run. Gripper pointing straight down at about
    // (0.30, 0, 0.55) m: 18 cm above and 10 cm behind the planned tower
    // (0.45 m in front, on a 10 cm pedestal, 27 cm tall). Chosen for good
    // conditioning (smallest singular value 0.21) and large joint-limit
    // margins (>= 0.91 rad).
    static JointVector homePose()
    {
        JointVector q;
        q << 0.0, -0.750, 0.0, -2.168, 0.0, 1.418, 0.785;
        return q;
    }

    // ---- Joint limits -------------------------------

    JointVector lowerLimits() const { return model_.lowerPositionLimit.head<7>(); }
    JointVector upperLimits() const { return model_.upperPositionLimit.head<7>(); }

    // Smallest distance (rad) from any joint to its nearest limit.
    double jointLimitMargin(const JointVector& arm) const
    {
        return std::min((arm - lowerLimits()).minCoeff(),
                        (upperLimits() - arm).minCoeff());
    }

private:
    // 9-value Pinocchio configuration: 7 arm joints + 2 fingers (kept at 0).
    Eigen::VectorXd fullConfiguration(const JointVector& arm) const
    {
        Eigen::VectorXd q = pinocchio::neutral(model_);
        q.head<7>() = arm;
        return q;
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex toolId_ = 0;
};
