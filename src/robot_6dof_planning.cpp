/**
 * OMPL Example: 6-DOF Robot Arm Motion Planning
 *
 * Problem:
 *   - Start: Known joint angles (q1, q2, q3, q4, q5, q6)
 *   - Goal: Desired end-effector pose (position + orientation)
 *   - Output: Trajectory of joint states from start to goal
 *
 * This example demonstrates:
 *   1. RealVectorStateSpace for 6 joint angles
 *   2. Forward kinematics using DH parameters
 *   3. Custom GoalRegion for task-space goals
 *   4. Simple self-collision checking
 *   5. Path interpolation for smooth trajectory
 *
 * Robot: Generic 6R manipulator (similar to UR5/PUMA style)
 */

 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
 #include <ompl/geometric/planners/prm/PRM.h>
 #include <ompl/geometric/SimpleSetup.h>
 #include <ompl/geometric/PathGeometric.h>

 #include <Eigen/Dense>
 #include <Eigen/Geometry>

 #include <iostream>
 #include <cmath>
 #include <vector>
 #include <iomanip>

 namespace ob = ompl::base;
 namespace og = ompl::geometric;

 //==============================================================================
 // ROBOT KINEMATICS
 //==============================================================================

/**
 * FANUC M710iC/50H Robot with OPW Kinematics
 *
 * OPW (Ortho-Parallel Weld) parameters from m710ic50H_opw_params.json
 * Joint limits from arm.xacro
 */
class Robot6DOF
{
public:
    // OPW Kinematic Parameters
    struct OPWParams
    {
        double a1, a2;      // Offsets in XY plane
        double b;           // Offset (usually 0)
        double c1, c2, c3, c4;  // Link lengths along kinematic chain
        std::array<double, 6> offsets;          // Joint angle offsets
        std::array<int, 6> sign_corrections;    // Direction corrections
    };

    Robot6DOF()
    {
        // Base height from base.xacro (pedestal/mount)
        // m710ic50_fixed_base_link_joint: z=0.525 + arm_mount_joint: z=0.525 = 1.05m
        base_height_ = 1.05;

        // OPW parameters from m710ic50H_opw_params.json
        opw_.a1 = 0.15;     // Offset from J1 to J2 axis
        opw_.a2 = -0.16;    // Offset
        opw_.b = 0.0;       // Usually 0 for this robot type
        opw_.c1 = 0.565;    // J1 axis height relative to arm mount
        opw_.c2 = 0.87;     // Lower arm length (J2 to J3)
        opw_.c3 = 0.970;    // Upper arm length (J3 to wrist)
        opw_.c4 = 0.175;    // Wrist to tool flange
        opw_.offsets = {0.0, 0.0, -M_PI/2, 0.0, 0.0, 0.0};
        opw_.sign_corrections = {1, 1, -1, 1, -1, -1};

        // Joint limits from arm.xacro (M710iC/50H)
        const double DEG2RAD = M_PI / 180.0;
        joint_limits_lower_ = {
            -180.0 * DEG2RAD,   // J1 s_joint
            -90.0 * DEG2RAD,    // J2 l_joint
            -80.0 * DEG2RAD,    // J3 u_joint
            -360.0 * DEG2RAD,   // J4 r_joint
            -125.0 * DEG2RAD,   // J5 b_joint
            -360.0 * DEG2RAD    // J6 t_joint
        };
        joint_limits_upper_ = {
            180.0 * DEG2RAD,    // J1
            135.0 * DEG2RAD,    // J2
            206.0 * DEG2RAD,    // J3
            360.0 * DEG2RAD,    // J4
            125.0 * DEG2RAD,    // J5
            360.0 * DEG2RAD     // J6
        };

        std::cout << "Loaded FANUC M710iC/50H robot\n";
        std::cout << "  Base height: " << base_height_ << " m (from base.xacro)\n";
        std::cout << "  OPW params: c1=" << opw_.c1 << " c2=" << opw_.c2
                  << " c3=" << opw_.c3 << " c4=" << opw_.c4 << "\n";
    }

    double getBaseHeight() const { return base_height_; }

    /**
     * Forward Kinematics using OPW parameters
     * Standard OPW model for 6-axis industrial robots
     */
    Eigen::Matrix4d forwardKinematics(const std::vector<double>& joints) const
    {
        // Apply offsets and sign corrections
        std::array<double, 6> q;
        for (int i = 0; i < 6; ++i)
            q[i] = opw_.sign_corrections[i] * joints[i] + opw_.offsets[i];

        double c1 = cos(q[0]), s1 = sin(q[0]);
        double c2 = cos(q[1]), s2 = sin(q[1]);
        double c3 = cos(q[2]), s3 = sin(q[2]);
        double c4 = cos(q[3]), s4 = sin(q[3]);
        double c5 = cos(q[4]), s5 = sin(q[4]);
        double c6 = cos(q[5]), s6 = sin(q[5]);
        double c23 = cos(q[1] + q[2]), s23 = sin(q[1] + q[2]);

        // Wrist center position
        double px = (opw_.a1 + opw_.c2 * c2 + opw_.c3 * c23) * c1 - opw_.a2 * s1;
        double py = (opw_.a1 + opw_.c2 * c2 + opw_.c3 * c23) * s1 + opw_.a2 * c1;
        double pz = opw_.c1 + opw_.c2 * s2 + opw_.c3 * s23;

        // Tool frame offset (simplified)
        px += opw_.c4 * c1 * c23;
        py += opw_.c4 * s1 * c23;
        pz += opw_.c4 * s23;

        // Build rotation matrix using ZYZ Euler angles approximation
        Eigen::Matrix3d Rz1 = Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ()).matrix();
        Eigen::Matrix3d Ry23 = Eigen::AngleAxisd(q[1] + q[2], Eigen::Vector3d::UnitY()).matrix();
        Eigen::Matrix3d Rz4 = Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitZ()).matrix();
        Eigen::Matrix3d Ry5 = Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY()).matrix();
        Eigen::Matrix3d Rz6 = Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitZ()).matrix();

        Eigen::Matrix3d R = Rz1 * Ry23 * Rz4 * Ry5 * Rz6;

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T(0,3) = px;
        T(1,3) = py;
        T(2,3) = pz + base_height_;  // Add pedestal height

        return T;
    }

    /**
     * Get all joint positions (for collision checking)
     * Returns positions of each link frame in base frame
     */
    std::vector<Eigen::Vector3d> getJointPositions(const std::vector<double>& joints) const
    {
        std::vector<Eigen::Vector3d> positions;

        std::array<double, 6> q;
        for (int i = 0; i < 6; ++i)
            q[i] = opw_.sign_corrections[i] * joints[i] + opw_.offsets[i];

        double c1 = cos(q[0]), s1 = sin(q[0]);
        double c2 = cos(q[1]), s2 = sin(q[1]);
        double c23 = cos(q[1] + q[2]), s23 = sin(q[1] + q[2]);

        // f_link - base at pedestal top
        positions.push_back(Eigen::Vector3d(0, 0, base_height_));

        // s_link - J1 axis (at base height c1 + pedestal)
        positions.push_back(Eigen::Vector3d(0, 0, base_height_ + opw_.c1));

        // l_link - J2 axis
        double j2x = opw_.a1 * c1 - opw_.a2 * s1;
        double j2y = opw_.a1 * s1 + opw_.a2 * c1;
        positions.push_back(Eigen::Vector3d(j2x, j2y, base_height_ + opw_.c1));

        // u_link - J3 axis (end of lower arm)
        double j3x = (opw_.a1 + opw_.c2 * c2) * c1 - opw_.a2 * s1;
        double j3y = (opw_.a1 + opw_.c2 * c2) * s1 + opw_.a2 * c1;
        double j3z = base_height_ + opw_.c1 + opw_.c2 * s2;
        positions.push_back(Eigen::Vector3d(j3x, j3y, j3z));

        // b_link - wrist center (J4/J5)
        double wx = (opw_.a1 + opw_.c2 * c2 + opw_.c3 * c23) * c1 - opw_.a2 * s1;
        double wy = (opw_.a1 + opw_.c2 * c2 + opw_.c3 * c23) * s1 + opw_.a2 * c1;
        double wz = base_height_ + opw_.c1 + opw_.c2 * s2 + opw_.c3 * s23;
        positions.push_back(Eigen::Vector3d(wx, wy, wz));

        // t_link - J6 axis
        positions.push_back(Eigen::Vector3d(wx, wy, wz));

        // ee_mount - end effector
        Eigen::Matrix4d T = forwardKinematics(joints);
        positions.push_back(T.block<3,1>(0,3));

        return positions;
    }

     /**
      * Extract position from transformation matrix
      */
     Eigen::Vector3d getPosition(const Eigen::Matrix4d& T) const
     {
         return T.block<3,1>(0,3);
     }

     /**
      * Extract rotation matrix from transformation matrix
      */
     Eigen::Matrix3d getRotation(const Eigen::Matrix4d& T) const
     {
         return T.block<3,3>(0,0);
     }

     /**
      * Convert rotation matrix to quaternion
      */
     Eigen::Quaterniond getQuaternion(const Eigen::Matrix4d& T) const
     {
         return Eigen::Quaterniond(getRotation(T));
     }

    // Accessors
    const std::vector<double>& getJointLimitsLower() const { return joint_limits_lower_; }
    const std::vector<double>& getJointLimitsUpper() const { return joint_limits_upper_; }

    /**
     * Compute numerical Jacobian (6x6: position + orientation)
     * Top 3 rows: position, Bottom 3 rows: orientation (angle-axis)
     */
    Eigen::Matrix<double, 6, 6> computeJacobian6D(const std::vector<double>& joints) const
    {
        const double delta = 1e-6;
        Eigen::Matrix<double, 6, 6> J;

        Eigen::Matrix4d T0 = forwardKinematics(joints);
        Eigen::Vector3d pos0 = getPosition(T0);
        Eigen::Matrix3d R0 = getRotation(T0);

        for (int i = 0; i < 6; ++i)
        {
            std::vector<double> joints_delta = joints;
            joints_delta[i] += delta;
            Eigen::Matrix4d T1 = forwardKinematics(joints_delta);
            Eigen::Vector3d pos1 = getPosition(T1);
            Eigen::Matrix3d R1 = getRotation(T1);

            // Position derivative
            J.block<3,1>(0, i) = (pos1 - pos0) / delta;

            // Orientation derivative (angular velocity approximation)
            Eigen::Matrix3d dR = (R1 - R0) / delta;
            Eigen::Matrix3d skew = dR * R0.transpose();
            // Extract angular velocity from skew-symmetric matrix
            J(3, i) = skew(2, 1);  // wx
            J(4, i) = skew(0, 2);  // wy
            J(5, i) = skew(1, 0);  // wz
        }
        return J;
    }

    /**
     * Compute orientation error as angle-axis vector
     */
    static Eigen::Vector3d orientationError(const Eigen::Quaterniond& current,
                                            const Eigen::Quaterniond& target)
    {
        // Error quaternion: q_err = q_target * q_current^-1
        Eigen::Quaterniond q_err = target * current.inverse();
        q_err.normalize();

        // Ensure shortest path (q and -q represent same rotation)
        if (q_err.w() < 0)
        {
            q_err.coeffs() = -q_err.coeffs();
        }

        // Convert to angle-axis (scaled axis = angle * axis)
        double angle = 2.0 * acos(std::min(1.0, std::abs(q_err.w())));
        if (angle < 1e-6)
            return Eigen::Vector3d::Zero();

        Eigen::Vector3d axis(q_err.x(), q_err.y(), q_err.z());
        double sin_half = axis.norm();
        if (sin_half < 1e-6)
            return Eigen::Vector3d::Zero();

        axis /= sin_half;
        return angle * axis;
    }

    /**
     * Full 6-DOF IK using damped least squares (position + orientation)
     * Returns true if solution found within tolerances
     */
    bool solveIK(const Eigen::Vector3d& target_pos,
                 const Eigen::Quaterniond& target_orient,
                 std::vector<double>& joints,
                 double pos_tolerance = 0.01,
                 double orient_tolerance = 0.1,
                 int max_iter = 200) const
    {
        const double lambda = 0.1;  // Damping factor (lower = faster convergence near goal)

        for (int iter = 0; iter < max_iter; ++iter)
        {
            Eigen::Matrix4d T = forwardKinematics(joints);
            Eigen::Vector3d current_pos = getPosition(T);
            Eigen::Quaterniond current_orient = getQuaternion(T);

            // Compute errors
            Eigen::Vector3d pos_error = target_pos - current_pos;
            Eigen::Vector3d orient_error = orientationError(current_orient, target_orient);

            // Check convergence
            if (pos_error.norm() < pos_tolerance && orient_error.norm() < orient_tolerance)
                return true;

            // Build 6D error vector
            Eigen::Matrix<double, 6, 1> error;
            error.head<3>() = pos_error;
            error.tail<3>() = orient_error;

            // Damped least squares
            Eigen::Matrix<double, 6, 6> J = computeJacobian6D(joints);
            Eigen::Matrix<double, 6, 6> JJT = J * J.transpose();
            JJT += lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::Matrix<double, 6, 1> v = JJT.ldlt().solve(error);
            Eigen::Matrix<double, 6, 1> dq = J.transpose() * v;

            // Update joints with clamping
            for (int i = 0; i < 6; ++i)
            {
                joints[i] += dq(i);
                joints[i] = std::max(joint_limits_lower_[i],
                            std::min(joint_limits_upper_[i], joints[i]));
            }
        }
        return false;
    }

    /**
     * Position-only IK (backward compatible)
     */
    bool solveIK(const Eigen::Vector3d& target_pos,
                 std::vector<double>& joints,
                 double tolerance = 0.01,
                 int max_iter = 100) const
    {
        // Use 6D IK with relaxed orientation constraint
        Eigen::Quaterniond dummy_orient = getQuaternion(forwardKinematics(joints));
        return solveIK(target_pos, dummy_orient, joints, tolerance, 10.0, max_iter);
    }

private:
    OPWParams opw_;
    double base_height_;  // Pedestal height from base.xacro
    std::vector<double> joint_limits_lower_;
    std::vector<double> joint_limits_upper_;
};


 //==============================================================================
 // COLLISION CHECKING
 //==============================================================================

/**
 * Collision checker with realistic industrial environment
 * Includes: table, walls, boxes, cylinders, and safety zones
 * With verbose collision reporting
 */
class CollisionChecker
{
public:
    // Axis-Aligned Bounding Box
    struct AABB {
        Eigen::Vector3d min_pt;
        Eigen::Vector3d max_pt;
        std::string name;
    };

    // Sphere obstacle
    struct Sphere {
        Eigen::Vector3d center;
        double radius;
        std::string name;
    };

    // Cylinder obstacle (vertical)
    struct Cylinder {
        Eigen::Vector3d base_center;
        double radius;
        double height;
        std::string name;
    };

    CollisionChecker(const Robot6DOF& robot) : robot_(robot)
    {
        setupLinkNames();
        setupEnvironment();
        self_collision_threshold_ = 0.05;  // 5cm between non-adjacent links
        safety_margin_ = 0.015;  // 1.5cm safety margin around obstacles
        verbose_ = false;  // Set to true to see collision details
        collision_count_ = 0;
    }

    void setupLinkNames()
    {
        // M710iC/50H link names from arm.xacro
        link_names_ = {
            "f_link (foundation)",
            "s_link (swing/J1)",
            "l_link (lower arm/J2)",
            "u_link (upper arm/J3)",
            "b_link (wrist bend/J5)",
            "t_link (tool flange/J6)",
            "ee_mount (end-effector)"
        };
    }

    void setupEnvironment()
    {
        // ==================== FROM static_obstacles.xacro ====================

        // Floor - below ground level (collision if EE goes below z=0)
        boxes_.push_back({
            Eigen::Vector3d(-5.0, -5.0, -1.0),
            Eigen::Vector3d(5.0, 5.0, -0.02),
            "floor"
        });

        // ==================== FROM workcell_only_obstacles.xacro ====================

        // UPS Conveyor Frame - box 0.2 x 1.0 x 2.5 at (2.02, 0.0, 1.25)
        // Box is centered at origin, so we need to compute min/max corners
        boxes_.push_back({
            Eigen::Vector3d(2.02 - 0.1, 0.0 - 0.5, 1.25 - 1.25),
            Eigen::Vector3d(2.02 + 0.1, 0.0 + 0.5, 1.25 + 1.25),
            "ups_conveyor_frame"
        });

        // UPS Conveyor Items - box 1.9 x 0.74 x 0.5 at (2.8, 0.0, 1.35)
        boxes_.push_back({
            Eigen::Vector3d(2.8 - 0.95, 0.0 - 0.37, 1.35 - 0.25),
            Eigen::Vector3d(2.8 + 0.95, 0.0 + 0.37, 1.35 + 0.25),
            "ups_conveyor_items"
        });

        // ==================== RACK POSITIONS (simplified as boxes) ====================
        // From static_obstacles.xacro - approximated as bounding boxes

        // Rack 01 at (1.85, -0.381, 0) - single rack (compact bbox to avoid start collision)
        addRotatedBox(1.95, -0.45, 1.0, 0.4, 0.25, 2.0, -18.0, "rack_01");

        // Rack 02/03 at (1.148, -1.35, 0) - big rack pair (smaller bbox)
        addRotatedBox(1.148, -1.35, 1.1, 1.0, 0.4, 2.2, -54.0, "rack_02_03");

        // Rack 04/05 at (0.0, -1.73, 0) - big rack pair
        addRotatedBox(0.0, -1.73, 1.1, 1.0, 0.4, 2.2, -90.0, "rack_04_05");

        // Rack 06/07 at (-1.148, -1.35, 0) - big rack pair
        addRotatedBox(-1.148, -1.35, 1.1, 1.0, 0.4, 2.2, -126.0, "rack_06_07");

        // Rack 08 at (-1.68, -0.912, 0) - single rack
        addRotatedBox(-1.68, -0.912, 1.0, 0.5, 0.3, 2.0, 198.0, "rack_08");

        // ==================== SAFETY ZONES ====================

        // Robot pedestal from base.xacro: cylinder length=1.05m, radius=0.425m
        // Centered at z=0.525 (half of 1.05)
        cylinders_.push_back({
            Eigen::Vector3d(0, 0, 0),
            0.425,   // radius from base.xacro
            1.05,    // height from base.xacro
            "m710ic50_pedestal"
        });

        printEnvironment();
    }

    // Helper to add an axis-aligned approximation of a rotated box
    void addRotatedBox(double cx, double cy, double cz,
                       double sx, double sy, double sz,
                       double angle_deg, const std::string& name)
    {
        // For simplicity, use bounding box that contains the rotated box
        double angle_rad = angle_deg * M_PI / 180.0;
        double ca = std::abs(cos(angle_rad));
        double sa = std::abs(sin(angle_rad));
        double bx = sx * ca + sy * sa;  // Bounding box size
        double by = sx * sa + sy * ca;

        boxes_.push_back({
            Eigen::Vector3d(cx - bx/2, cy - by/2, cz - sz/2),
            Eigen::Vector3d(cx + bx/2, cy + by/2, cz + sz/2),
            name
        });
    }

    void setVerbose(bool v) { verbose_ = v; }
    size_t getCollisionCount() const { return collision_count_; }
    void resetCollisionCount() { collision_count_ = 0; }

    void printEnvironment() const
    {
        std::cout << "\nCollision Environment:\n";
        std::cout << "  Boxes: " << boxes_.size() << "\n";
        for (const auto& b : boxes_)
            std::cout << "    - " << b.name << "\n";
        std::cout << "  Cylinders: " << cylinders_.size() << "\n";
        for (const auto& c : cylinders_)
            std::cout << "    - " << c.name << "\n";
        std::cout << "  Spheres: " << spheres_.size() << "\n";
        for (const auto& s : spheres_)
            std::cout << "    - " << s.name << "\n";
        std::cout << "\nRobot Links:\n";
        for (size_t i = 0; i < link_names_.size(); ++i)
            std::cout << "  [" << i << "] " << link_names_[i] << "\n";
    }

    /**
     * Check if configuration is collision-free
     */
    bool isValid(const std::vector<double>& joints) const
    {
        std::vector<Eigen::Vector3d> positions = robot_.getJointPositions(joints);

        // Self-collision check (non-adjacent links)
        for (size_t i = 0; i < positions.size(); ++i)
        {
            for (size_t j = i + 2; j < positions.size(); ++j)
            {
                double dist = (positions[i] - positions[j]).norm();
                if (dist < self_collision_threshold_)
                {
                    ++collision_count_;
                    if (verbose_)
                    {
                        std::cout << "\033[31m[SELF-COLLISION]\033[0m "
                                  << link_names_[i] << " <-> " << link_names_[j]
                                  << " (dist: " << dist*1000 << "mm, threshold: "
                                  << self_collision_threshold_*1000 << "mm)\n";
                    }
                    return false;
                }
            }
        }

        // Check each joint position against obstacles (skip fixed base frames)
        for (size_t i = 2; i < positions.size(); ++i)
        {
            const Eigen::Vector3d& pt = positions[i];
            const std::string& link = link_names_[i];

            // Check boxes (AABB)
            for (const auto& box : boxes_)
            {
                if (pointInAABB(pt, box, safety_margin_))
                {
                    ++collision_count_;
                    if (verbose_)
                    {
                        std::cout << "\033[31m[COLLISION]\033[0m "
                                  << link << " hit BOX '" << box.name << "'"
                                  << " at pos(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")\n";
                    }
                    return false;
                }
            }

            // Check spheres
            for (const auto& sphere : spheres_)
            {
                double dist = (pt - sphere.center).norm();
                if (dist < sphere.radius + safety_margin_)
                {
                    ++collision_count_;
                    if (verbose_)
                    {
                        std::cout << "\033[31m[COLLISION]\033[0m "
                                  << link << " hit SPHERE '" << sphere.name << "'"
                                  << " (dist: " << dist*1000 << "mm, radius: "
                                  << sphere.radius*1000 << "mm)\n";
                    }
                    return false;
                }
            }

            // Check cylinders
            for (const auto& cyl : cylinders_)
            {
                if (pointInCylinder(pt, cyl, safety_margin_))
                {
                    ++collision_count_;
                    if (verbose_)
                    {
                        std::cout << "\033[31m[COLLISION]\033[0m "
                                  << link << " hit CYLINDER '" << cyl.name << "'"
                                  << " at pos(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")\n";
                    }
                    return false;
                }
            }
        }

        // Check link segments (not just joint positions, skip base)
        for (size_t i = 2; i + 1 < positions.size(); ++i)
        {
            std::string collision_obj;
            if (!segmentCollisionFree(positions[i], positions[i+1], collision_obj))
            {
                ++collision_count_;
                if (verbose_)
                {
                    std::cout << "\033[31m[COLLISION]\033[0m link segment "
                              << link_names_[i] << " -> " << link_names_[i+1]
                              << " hit '" << collision_obj << "'\n";
                }
                return false;
            }
        }

        // Workspace bounds for M710iC/50H on 1.05m pedestal
        Eigen::Vector3d ee = positions.back();
        if (ee.z() < 0.05)
        {
            ++collision_count_;
            if (verbose_)
                std::cout << "\033[31m[BOUNDS]\033[0m end-effector below ground (z=" << ee.z() << ")\n";
            return false;
        }
        // Distance from pedestal top to check reach (pedestal at z=1.05)
        Eigen::Vector3d ee_from_base(ee.x(), ee.y(), ee.z() - 1.05);
        if (ee_from_base.norm() > 2.5)
        {
            ++collision_count_;
            if (verbose_)
                std::cout << "\033[31m[BOUNDS]\033[0m end-effector beyond reach (dist=" << ee_from_base.norm() << ")\n";
            return false;
        }

        return true;
    }

    void addBox(const Eigen::Vector3d& min_pt, const Eigen::Vector3d& max_pt,
                const std::string& name = "custom_box")
    {
        boxes_.push_back({min_pt, max_pt, name});
    }

    void addSphere(const Eigen::Vector3d& center, double radius,
                   const std::string& name = "custom_sphere")
    {
        spheres_.push_back({center, radius, name});
    }

    void addCylinder(const Eigen::Vector3d& base, double radius, double height,
                     const std::string& name = "custom_cylinder")
    {
        cylinders_.push_back({base, radius, height, name});
    }

    // Getters for visualization
    const std::vector<AABB>& getBoxes() const { return boxes_; }
    const std::vector<Cylinder>& getCylinders() const { return cylinders_; }
    const std::vector<Sphere>& getSpheres() const { return spheres_; }

private:
    bool pointInAABB(const Eigen::Vector3d& pt, const AABB& box, double margin) const
    {
        return pt.x() >= box.min_pt.x() - margin && pt.x() <= box.max_pt.x() + margin &&
               pt.y() >= box.min_pt.y() - margin && pt.y() <= box.max_pt.y() + margin &&
               pt.z() >= box.min_pt.z() - margin && pt.z() <= box.max_pt.z() + margin;
    }

    bool pointInCylinder(const Eigen::Vector3d& pt, const Cylinder& cyl, double margin) const
    {
        if (pt.z() < cyl.base_center.z() - margin ||
            pt.z() > cyl.base_center.z() + cyl.height + margin)
            return false;

        double dx = pt.x() - cyl.base_center.x();
        double dy = pt.y() - cyl.base_center.y();
        return std::sqrt(dx*dx + dy*dy) < cyl.radius + margin;
    }

    bool segmentCollisionFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                               std::string& collision_obj) const
    {
        const int num_samples = 5;
        for (int i = 1; i < num_samples; ++i)
        {
            double t = static_cast<double>(i) / num_samples;
            Eigen::Vector3d pt = p1 + t * (p2 - p1);

            for (const auto& box : boxes_)
            {
                if (pointInAABB(pt, box, safety_margin_))
                {
                    collision_obj = box.name;
                    return false;
                }
            }

            for (const auto& sphere : spheres_)
            {
                if ((pt - sphere.center).norm() < sphere.radius + safety_margin_)
                {
                    collision_obj = sphere.name;
                    return false;
                }
            }

            for (const auto& cyl : cylinders_)
            {
                if (pointInCylinder(pt, cyl, safety_margin_))
                {
                    collision_obj = cyl.name;
                    return false;
                }
            }
        }
        return true;
    }

    const Robot6DOF& robot_;
    std::vector<std::string> link_names_;
    std::vector<AABB> boxes_;
    std::vector<Sphere> spheres_;
    std::vector<Cylinder> cylinders_;
    double self_collision_threshold_;
    double safety_margin_;
    bool verbose_;
    mutable size_t collision_count_;
};


 //==============================================================================
 // OMPL GOAL DEFINITION
 //==============================================================================

/**
 * Custom Goal: End-effector reaches target pose with IK-based sampling
 *
 * The goal is defined in task space (position + orientation)
 * but we plan in joint space. This class bridges the gap.
 * Implements GoalSampleableRegion to enable RRTConnect.
 */
class EndEffectorGoal : public ob::GoalSampleableRegion
{
public:
    EndEffectorGoal(const ob::SpaceInformationPtr& si,
                    const Robot6DOF& robot,
                    const Eigen::Vector3d& target_position,
                    const Eigen::Quaterniond& target_orientation,
                    double position_tolerance = 0.02,      // 2cm
                    double orientation_tolerance = 0.1)    // ~6 degrees
        : ob::GoalSampleableRegion(si),
          robot_(robot),
          target_position_(target_position),
          target_orientation_(target_orientation),
          position_tolerance_(position_tolerance),
          orientation_tolerance_(orientation_tolerance),
          sampler_(si->getStateSpace()->allocStateSampler())
    {
        // Set threshold for isSatisfied() method
        setThreshold(position_tolerance);
    }

    /**
     * Compute distance from current state to goal
     */
    double distanceGoal(const ob::State* state) const override
    {
        const auto* rvstate = state->as<ob::RealVectorStateSpace::StateType>();
        std::vector<double> joints(6);
        for (int i = 0; i < 6; ++i)
            joints[i] = rvstate->values[i];

        Eigen::Matrix4d T = robot_.forwardKinematics(joints);
        Eigen::Vector3d current_pos = robot_.getPosition(T);
        Eigen::Quaterniond current_quat = robot_.getQuaternion(T);

        double pos_error = (current_pos - target_position_).norm();
        double dot = std::abs(current_quat.dot(target_orientation_));
        dot = std::min(1.0, dot);
        double orient_error = 2.0 * acos(dot);

        return pos_error + 0.1 * orient_error;
    }

    /**
     * Sample a goal state using 6-DOF IK from random seed configurations
     */
    void sampleGoal(ob::State* state) const override
    {
        auto* rvstate = state->as<ob::RealVectorStateSpace::StateType>();

        // Try IK from multiple random seeds
        for (int attempt = 0; attempt < 30; ++attempt)
        {
            sampler_->sampleUniform(state);
            std::vector<double> joints(6);
            for (int i = 0; i < 6; ++i)
                joints[i] = rvstate->values[i];

            // Use full 6-DOF IK with position and orientation
            if (robot_.solveIK(target_position_, target_orientation_, joints,
                               position_tolerance_, orientation_tolerance_, 200))
            {
                for (int i = 0; i < 6; ++i)
                    rvstate->values[i] = joints[i];
                return;
            }
        }
        // Fallback: random sample (planner will check validity)
        sampler_->sampleUniform(state);
    }

    unsigned int maxSampleCount() const override { return 1000; }

    /**
     * Check if state satisfies the goal
     */
    bool isSatisfied(const ob::State* state) const override
    {
        const auto* rvstate = state->as<ob::RealVectorStateSpace::StateType>();
        std::vector<double> joints(6);
        for (int i = 0; i < 6; ++i)
            joints[i] = rvstate->values[i];

        Eigen::Matrix4d T = robot_.forwardKinematics(joints);
        Eigen::Vector3d current_pos = robot_.getPosition(T);
        Eigen::Quaterniond current_quat = robot_.getQuaternion(T);

        double pos_error = (current_pos - target_position_).norm();
        if (pos_error > position_tolerance_)
            return false;

        double dot = std::abs(current_quat.dot(target_orientation_));
        dot = std::min(1.0, dot);
        double orient_error = 2.0 * acos(dot);
        if (orient_error > orientation_tolerance_)
            return false;

        return true;
    }

private:
    const Robot6DOF& robot_;
    Eigen::Vector3d target_position_;
    Eigen::Quaterniond target_orientation_;
    double position_tolerance_;
    double orientation_tolerance_;
    ob::StateSamplerPtr sampler_;
};


 //==============================================================================
 // STATE VALIDITY CHECKER
 //==============================================================================

 class RobotStateValidityChecker : public ob::StateValidityChecker
 {
 public:
     RobotStateValidityChecker(const ob::SpaceInformationPtr& si,
                                const Robot6DOF& robot,
                                const CollisionChecker& collision_checker)
         : ob::StateValidityChecker(si),
           robot_(robot),
           collision_checker_(collision_checker)
     {
        double base_height = robot.getBaseHeight();
        std::cout << "Base height: " << base_height << " m\n";
     }

     bool isValid(const ob::State* state) const override
     {
         // Check bounds first (fast)
         if (!si_->satisfiesBounds(state))
             return false;

         // Extract joint values
         const auto* rvstate = state->as<ob::RealVectorStateSpace::StateType>();
         std::vector<double> joints(6);
         for (int i = 0; i < 6; ++i)
             joints[i] = rvstate->values[i];

         // Check collisions
         return collision_checker_.isValid(joints);
     }

     Robot6DOF getRobot() const { return robot_; }

 private:
     const Robot6DOF& robot_;
     const CollisionChecker& collision_checker_;
 };


 //==============================================================================
 // MAIN PLANNER
 //==============================================================================

 class Robot6DOFPlanner
 {
 public:
     Robot6DOFPlanner()
         : robot_(),
           collision_checker_(robot_)
     {
         // Create 6D state space for joint angles
         space_ = std::make_shared<ob::RealVectorStateSpace>(6);

         // Set joint limits
         ob::RealVectorBounds bounds(6);
         const auto& lower = robot_.getJointLimitsLower();
         const auto& upper = robot_.getJointLimitsUpper();
         for (int i = 0; i < 6; ++i)
         {
             bounds.setLow(i, lower[i]);
             bounds.setHigh(i, upper[i]);
         }
         space_->setBounds(bounds);

         // Create space information
         si_ = std::make_shared<ob::SpaceInformation>(space_);

         // Set validity checker
         si_->setStateValidityChecker(
             std::make_shared<RobotStateValidityChecker>(si_, robot_, collision_checker_));

         // Set motion validity resolution (how finely to check edges)
         si_->setStateValidityCheckingResolution(0.01);

         si_->setup();

        std::cout << "Robot6DOFPlanner initialized\n";
        std::cout << "  Joint space dimension: 6\n";
        std::cout << "  Joint limits: [" << lower[0] << ", " << upper[0] << "] rad\n";
    }

    void setVerboseCollisions(bool v) { collision_checker_.setVerbose(v); }

    void printCollisionStats() const
    {
        std::cout << "\n========== COLLISION STATISTICS ==========\n";
        std::cout << "Total collision checks that failed: " << collision_checker_.getCollisionCount() << "\n";
        std::cout << "===========================================\n";
    }

    void resetCollisionStats() { collision_checker_.resetCollisionCount(); }

    /**
     * Plan from start joint configuration to goal joint configuration
     *
     * @param start_joints: Starting joint angles (6 values, radians)
     * @param goal_joints: Target joint angles (6 values, radians)
     * @param planning_time: Maximum planning time in seconds
     * @return Vector of joint configurations forming the path
     */
    std::vector<std::vector<double>> plan(
        const std::vector<double>& start_joints,
        const std::vector<double>& goal_joints,
        double planning_time = 5.0)
    {
        std::vector<std::vector<double>> trajectory;

        // Validate start configuration
        if (!collision_checker_.isValid(start_joints))
        {
            std::cerr << "ERROR: Start configuration is in collision!\n";
            return trajectory;
        }

        // Validate goal configuration
        if (!collision_checker_.isValid(goal_joints))
        {
            std::cerr << "ERROR: Goal configuration is in collision!\n";
            return trajectory;
        }

        // Print start pose
        Eigen::Matrix4d T_start = robot_.forwardKinematics(start_joints);
        std::cout << "\nStart configuration:\n";
        std::cout << "  Joints: [";
        for (size_t i = 0; i < start_joints.size(); ++i)
            std::cout << std::fixed << std::setprecision(3) << start_joints[i] << (i<5 ? ", " : "");
        std::cout << "] rad\n";
        std::cout << "  End-effector position: " << robot_.getPosition(T_start).transpose() << " m\n";

        Eigen::Matrix4d T_goal = robot_.forwardKinematics(goal_joints);
        std::cout << "\nGoal configuration:\n";
        std::cout << "  Joints: [";
        for (size_t i = 0; i < goal_joints.size(); ++i)
            std::cout << std::fixed << std::setprecision(3) << goal_joints[i] << (i<5 ? ", " : "");
        std::cout << "] rad\n";
        std::cout << "  End-effector position: " << robot_.getPosition(T_goal).transpose() << " m\n";

        // Create problem definition
        auto pdef = std::make_shared<ob::ProblemDefinition>(si_);

        // Set start state
        ob::ScopedState<ob::RealVectorStateSpace> start(space_);
        for (size_t i = 0; i < start_joints.size(); ++i)
            start[i] = start_joints[i];
        pdef->addStartState(start);

        // Set goal state (joint-space goal)
        ob::ScopedState<ob::RealVectorStateSpace> goal(space_);
        for (size_t i = 0; i < goal_joints.size(); ++i)
            goal[i] = goal_joints[i];
        pdef->setGoalState(goal);

        // Create planner (RRTConnect is fast - grows two trees)
        auto planner = std::make_shared<og::RRTConnect>(si_);
        planner->setRange(0.5);  // Max step size in joint space
        planner->setProblemDefinition(pdef);
        planner->setup();

        std::cout << "\nPlanning with RRTConnect (timeout: " << planning_time << "s)...\n";

        // Solve
        ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(planning_time));

        if (solved)
        {
            std::cout << "Solution found!\n";

            // Get path
            auto path = pdef->getSolutionPath()->as<og::PathGeometric>();

            std::cout << "Raw path has " << path->getStateCount() << " waypoints\n";

            // Simplify
            og::PathSimplifier simplifier(si_);
            simplifier.simplifyMax(*path);
            std::cout << "Simplified path has " << path->getStateCount() << " waypoints\n";

            // Interpolate for smooth trajectory
            path->interpolate(50);  // 50 waypoints total
            std::cout << "Interpolated path has " << path->getStateCount() << " waypoints\n";

            // Extract trajectory
            for (size_t i = 0; i < path->getStateCount(); ++i)
            {
                const auto* state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
                std::vector<double> joints(start_joints.size());
                for (size_t j = 0; j < start_joints.size(); ++j)
                    joints[j] = state->values[j];
                trajectory.push_back(joints);
            }

            // Print final pose comparison
            Eigen::Matrix4d T_final = robot_.forwardKinematics(trajectory.back());
            Eigen::Vector3d final_pos = robot_.getPosition(T_final);
            Eigen::Vector3d goal_pos = robot_.getPosition(T_goal);

            std::cout << "\nFinal end-effector pose:\n";
            std::cout << "  Position: " << final_pos.transpose() << " m\n";
            std::cout << "  Position error: " << (final_pos - goal_pos).norm() * 1000 << " mm\n";
        }
        else
        {
            std::cout << "No solution found within time limit.\n";
            std::cout << "Try:\n";
            std::cout << "  - Increasing planning time\n";
            std::cout << "  - Checking if path between configs is feasible\n";
        }

        return trajectory;
    }

    /**
     * Plan from start joint configuration to goal end-effector pose
     *
     * @param start_joints: Starting joint angles (6 values, radians)
     * @param goal_position: Target end-effector position (x, y, z) in meters
     * @param goal_orientation: Target orientation as quaternion (w, x, y, z)
     * @param planning_time: Maximum planning time in seconds
     * @return Vector of joint configurations forming the path
     */
    std::vector<std::vector<double>> plan(
        const std::vector<double>& start_joints,
        const Eigen::Vector3d& goal_position,
        const Eigen::Quaterniond& goal_orientation,
        double planning_time = 5.0)
     {
         std::vector<std::vector<double>> trajectory;

         // Validate start configuration
         if (!collision_checker_.isValid(start_joints))
         {
             std::cerr << "ERROR: Start configuration is in collision!\n";
             return trajectory;
         }

         // Print start pose
         Eigen::Matrix4d T_start = robot_.forwardKinematics(start_joints);
         std::cout << "\nStart configuration:\n";
         std::cout << "  Joints: [";
         for (size_t i = 0; i < start_joints.size(); ++i)
             std::cout << std::fixed << std::setprecision(3) << start_joints[i] << (i<5 ? ", " : "");
         std::cout << "] rad\n";
         std::cout << "  End-effector position: " << robot_.getPosition(T_start).transpose() << " m\n";

         std::cout << "\nGoal pose:\n";
         std::cout << "  Position: " << goal_position.transpose() << " m\n";
         std::cout << "  Orientation (quat): [" << goal_orientation.w() << ", "
                   << goal_orientation.x() << ", " << goal_orientation.y() << ", "
                   << goal_orientation.z() << "]\n";

         // Create problem definition
         auto pdef = std::make_shared<ob::ProblemDefinition>(si_);

         // Set start state
         ob::ScopedState<ob::RealVectorStateSpace> start(space_);
         for (size_t i = 0; i < start_joints.size(); ++i)
             start[i] = start_joints[i];
         pdef->addStartState(start);

         // Set goal (task-space goal)
         auto goal = std::make_shared<EndEffectorGoal>(
             si_, robot_, goal_position, goal_orientation,
            0.001,  // 5mm position tolerance
            0.01);  // ~3 degree orientation tolerance
         pdef->setGoal(goal);

        // Create planner (RRTConnect is fast - grows two trees)
        auto planner = std::make_shared<og::RRTConnect>(si_);
        planner->setRange(0.5);  // Max step size in joint space
        planner->setProblemDefinition(pdef);
        planner->setup();

        std::cout << "\nPlanning with RRTConnect (timeout: " << planning_time << "s)...\n";

         // Solve
         ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(planning_time));

         if (solved)
         {
             std::cout << "Solution found!\n";

             // Get path
             auto path = pdef->getSolutionPath()->as<og::PathGeometric>();

             std::cout << "Raw path has " << path->getStateCount() << " waypoints\n";

             // Simplify
             og::PathSimplifier simplifier(si_);
             simplifier.simplifyMax(*path);
             std::cout << "Simplified path has " << path->getStateCount() << " waypoints\n";

             // Interpolate for smooth trajectory
             path->interpolate(50);  // 50 waypoints total
             std::cout << "Interpolated path has " << path->getStateCount() << " waypoints\n";

             // Extract trajectory
             for (size_t i = 0; i < path->getStateCount(); ++i)
             {
                 const auto* state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
                 std::vector<double> joints(start_joints.size());
                 for (size_t j = 0; j < start_joints.size(); ++j)
                     joints[j] = state->values[j];
                 trajectory.push_back(joints);
             }

             // Print final pose
             Eigen::Matrix4d T_final = robot_.forwardKinematics(trajectory.back());
             Eigen::Vector3d final_pos = robot_.getPosition(T_final);
             Eigen::Quaterniond final_quat = robot_.getQuaternion(T_final);

             std::cout << "\nFinal end-effector pose:\n";
             std::cout << "  Position: " << final_pos.transpose() << " m\n";
             std::cout << "  Position error: " << (final_pos - goal_position).norm() * 1000 << " mm\n";

             double dot = std::abs(final_quat.dot(goal_orientation));
             double orient_error = 2.0 * acos(std::min(1.0, dot)) * 180.0 / M_PI;
             std::cout << "  Orientation error: " << orient_error << " degrees\n";
         }
         else
         {
             std::cout << "No solution found within time limit.\n";
             std::cout << "Try:\n";
             std::cout << "  - Increasing planning time\n";
             std::cout << "  - Checking if goal is reachable\n";
             std::cout << "  - Relaxing goal tolerances\n";
         }

         return trajectory;
     }

     /**
      * Print the trajectory
      */
     void printTrajectory(const std::vector<std::vector<double>>& trajectory, const size_t joint_size) const
     {
         std::cout << "\n========== TRAJECTORY ==========\n";
         std::cout << std::fixed << std::setprecision(4);

         std::cout << "waypoint,q1,q2,q3,q4,q5,q6,x,y,z\n";

         for (size_t i = 0; i < trajectory.size(); ++i)
         {
             const auto& joints = trajectory[i];
             Eigen::Matrix4d T = robot_.forwardKinematics(joints);
             Eigen::Vector3d pos = robot_.getPosition(T);

             std::cout << i << ",";
             for (size_t j = 0; j < joint_size; ++j)
                 std::cout << joints[j] << ",";
             std::cout << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
         }

         std::cout << "================================\n";
     }

     /**
     * Compute forward kinematics for given joints
     */
    Eigen::Matrix4d computeFK(const std::vector<double>& joints) const
    {
        return robot_.forwardKinematics(joints);
    }

    /**
     * ASCII visualization of the workspace
     * Shows top-down (XY) and side (XZ) views
     */
    void visualize(const std::vector<std::vector<double>>& trajectory,
                   const Eigen::Vector3d& start_pos,
                   const Eigen::Vector3d& goal_pos) const
    {
        const int WIDTH = 80;
        const int HEIGHT = 40;

        // World bounds for visualization
        const double x_min = -1.0, x_max = 4.0;
        const double y_min = -2.5, y_max = 2.5;
        const double z_min = 0.0, z_max = 4.0;

        auto toGridXY = [&](double x, double y, int& gx, int& gy) {
            gx = static_cast<int>((x - x_min) / (x_max - x_min) * (WIDTH - 1));
            gy = static_cast<int>((y_max - y) / (y_max - y_min) * (HEIGHT - 1));
            gx = std::max(0, std::min(WIDTH - 1, gx));
            gy = std::max(0, std::min(HEIGHT - 1, gy));
        };

        auto toGridXZ = [&](double x, double z, int& gx, int& gz) {
            gx = static_cast<int>((x - x_min) / (x_max - x_min) * (WIDTH - 1));
            gz = static_cast<int>((z_max - z) / (z_max - z_min) * (HEIGHT - 1));
            gx = std::max(0, std::min(WIDTH - 1, gx));
            gz = std::max(0, std::min(HEIGHT - 1, gz));
        };

        // Initialize grids
        std::vector<std::string> gridXY(HEIGHT, std::string(WIDTH, ' '));
        std::vector<std::string> gridXZ(HEIGHT, std::string(WIDTH, ' '));

        // Draw collision boxes
        for (const auto& box : collision_checker_.getBoxes())
        {
            // Skip floor in XY view (it covers everything)
            bool is_floor = (box.name == "floor");

            // XY view (top-down) - skip floor, draw outline for others
            if (!is_floor)
            {
                // Draw box outline
                for (double x = box.min_pt.x(); x <= box.max_pt.x(); x += 0.03)
                {
                    int gx, gy;
                    toGridXY(x, box.min_pt.y(), gx, gy);
                    if (gridXY[gy][gx] == ' ') gridXY[gy][gx] = '#';
                    toGridXY(x, box.max_pt.y(), gx, gy);
                    if (gridXY[gy][gx] == ' ') gridXY[gy][gx] = '#';
                }
                for (double y = box.min_pt.y(); y <= box.max_pt.y(); y += 0.03)
                {
                    int gx, gy;
                    toGridXY(box.min_pt.x(), y, gx, gy);
                    if (gridXY[gy][gx] == ' ') gridXY[gy][gx] = '#';
                    toGridXY(box.max_pt.x(), y, gx, gy);
                    if (gridXY[gy][gx] == ' ') gridXY[gy][gx] = '#';
                }
            }
            // XZ view (side)
            for (double x = box.min_pt.x(); x <= box.max_pt.x(); x += 0.05)
            {
                for (double z = box.min_pt.z(); z <= box.max_pt.z(); z += 0.05)
                {
                    int gx, gz;
                    toGridXZ(x, z, gx, gz);
                    if (gridXZ[gz][gx] == ' ') gridXZ[gz][gx] = '#';
                }
            }
        }

        // Draw cylinders (pedestal)
        for (const auto& cyl : collision_checker_.getCylinders())
        {
            double cx = cyl.base_center.x();
            double cy = cyl.base_center.y();
            double cz = cyl.base_center.z();
            // XY view - circle
            for (double a = 0; a < 2 * M_PI; a += 0.1)
            {
                double x = cx + cyl.radius * cos(a);
                double y = cy + cyl.radius * sin(a);
                int gx, gy;
                toGridXY(x, y, gx, gy);
                if (gridXY[gy][gx] == ' ') gridXY[gy][gx] = 'O';
            }
            // XZ view - rectangle
            for (double z = cz; z <= cz + cyl.height; z += 0.05)
            {
                int gx1, gz1, gx2, gz2;
                toGridXZ(cx - cyl.radius, z, gx1, gz1);
                toGridXZ(cx + cyl.radius, z, gx2, gz2);
                if (gridXZ[gz1][gx1] == ' ') gridXZ[gz1][gx1] = '|';
                if (gridXZ[gz2][gx2] == ' ') gridXZ[gz2][gx2] = '|';
            }
        }

        // Draw trajectory path
        for (const auto& joints : trajectory)
        {
            Eigen::Matrix4d T = robot_.forwardKinematics(joints);
            Eigen::Vector3d pos = robot_.getPosition(T);
            int gx, gy, gz;
            toGridXY(pos.x(), pos.y(), gx, gy);
            if (gridXY[gy][gx] == ' ') gridXY[gy][gx] = '.';
            toGridXZ(pos.x(), pos.z(), gx, gz);
            if (gridXZ[gz][gx] == ' ') gridXZ[gz][gx] = '.';
        }

        // Draw robot at start configuration
        if (!trajectory.empty())
        {
            auto link_pos = robot_.getJointPositions(trajectory.front());
            for (size_t i = 1; i < link_pos.size(); ++i)
            {
                // Draw line between links
                Eigen::Vector3d p1 = link_pos[i-1];
                Eigen::Vector3d p2 = link_pos[i];
                for (double t = 0; t <= 1.0; t += 0.1)
                {
                    Eigen::Vector3d p = p1 + t * (p2 - p1);
                    int gx, gy, gz;
                    toGridXY(p.x(), p.y(), gx, gy);
                    if (gridXY[gy][gx] == ' ' || gridXY[gy][gx] == '.') gridXY[gy][gx] = '*';
                    toGridXZ(p.x(), p.z(), gx, gz);
                    if (gridXZ[gz][gx] == ' ' || gridXZ[gz][gx] == '.') gridXZ[gz][gx] = '*';
                }
            }
        }

        // Draw start pose (S)
        {
            int gx, gy, gz;
            toGridXY(start_pos.x(), start_pos.y(), gx, gy);
            gridXY[gy][gx] = 'S';
            toGridXZ(start_pos.x(), start_pos.z(), gx, gz);
            gridXZ[gz][gx] = 'S';
        }

        // Draw goal pose (G)
        {
            int gx, gy, gz;
            toGridXY(goal_pos.x(), goal_pos.y(), gx, gy);
            gridXY[gy][gx] = 'G';
            toGridXZ(goal_pos.x(), goal_pos.z(), gx, gz);
            gridXZ[gz][gx] = 'G';
        }

        // Print XY view
        std::cout << "\n========== TOP-DOWN VIEW (XY) ==========\n";
        std::cout << "Legend: S=start, G=goal, *=robot, .=path, #=box, O=cylinder\n";
        std::cout << "        Y+" << std::string(WIDTH - 10, ' ') << "X: [" << x_min << ", " << x_max << "]\n";
        std::cout << "        ^" << std::string(WIDTH - 10, ' ') << "Y: [" << y_min << ", " << y_max << "]\n";
        std::cout << "        |" << "\n";
        for (int row = 0; row < HEIGHT; ++row)
        {
            if (row == HEIGHT / 2)
                std::cout << "  ----> X  ";
            else
                std::cout << "           ";
            std::cout << gridXY[row] << "\n";
        }

        // Print XZ view
        std::cout << "\n========== SIDE VIEW (XZ) ==========\n";
        std::cout << "Legend: S=start, G=goal, *=robot, .=path, #=box, |=cylinder\n";
        std::cout << "        Z+" << std::string(WIDTH - 10, ' ') << "X: [" << x_min << ", " << x_max << "]\n";
        std::cout << "        ^" << std::string(WIDTH - 10, ' ') << "Z: [" << z_min << ", " << z_max << "]\n";
        std::cout << "        |" << "\n";
        for (int row = 0; row < HEIGHT; ++row)
        {
            if (row == HEIGHT / 2)
                std::cout << "  ----> X  ";
            else
                std::cout << "           ";
            std::cout << gridXZ[row] << "\n";
        }
        std::cout << "=====================================\n";
    }

private:
     Robot6DOF robot_;
     CollisionChecker collision_checker_;
     std::shared_ptr<ob::RealVectorStateSpace> space_;
     ob::SpaceInformationPtr si_;
 };


 //==============================================================================
 // MAIN
 //==============================================================================

 int main()
 {
    std::cout << "==============================================\n";
    std::cout << "  FANUC M710iC/50H Motion Planning with OMPL\n";
    std::cout << "  Using OPW Kinematics + Workcell Obstacles\n";
    std::cout << "==============================================\n\n";

    // Create planner
    Robot6DOFPlanner planner;

    // Enable verbose collision reporting to see what's happening during search
    planner.setVerboseCollisions(true);

    // Define start configuration from gazebo_plugins.xacro
    std::vector<double> start_joints = {
        0.0,            // J1 s_joint
        0.927747217,    // J2 l_joint
        -0.642,         // J3 u_joint
        0.0,            // J4 r_joint (not in xacro, default 0)
        0.0,            // J5 b_joint
        0.0             // J6 t_joint
    };

    // Define goal pose (end-effector target)
    // Reaching toward the UPS conveyor area (robot base is at z=1.05)
    Eigen::Vector3d goal_position(1.86, 0.0, 1.0);  // meters - near conveyor height

    // Goal orientation: identity (w,x,y,z) = (1,0,0,0)
    Eigen::Quaterniond goal_orientation(1.0, 0.0, 0.0, 0.0);

    // Option 1: Plan to end-effector pose (position + orientation)
    auto trajectory = planner.plan(
        start_joints,
        goal_position,
        goal_orientation,
        5.0  // 5 second timeout
    );

    // Option 2: Plan to specific joint configuration (uncomment to use)
    std::vector<double> goal_joints = {
        -2.77074,   // J1
        0.326677,   // J2
        0.796674,  // J3
        0.0,   // J4
        -2.0407, // J5
        0.0566896   // J6
    };
    // auto trajectory = planner.plan(start_joints, goal_joints, 5.0);

    // Print collision statistics
    planner.printCollisionStats();

    if (!trajectory.empty())
    {
        // Get start position for visualization
        Eigen::Matrix4d T_start = planner.computeFK(start_joints);
        Eigen::Vector3d start_pos(T_start(0,3), T_start(1,3), T_start(2,3));

        // Visualize the workspace
        // planner.visualize(trajectory, start_pos, goal_position);

        // Print trajectory (can be used to execute on real robot)
        planner.printTrajectory(trajectory, start_joints.size());

        std::cout << "\nTrajectory has " << trajectory.size() << " waypoints.\n";
        std::cout << "Each waypoint is a set of 6 joint angles (radians).\n";
        std::cout << "Send these to your robot controller sequentially.\n";
    }

    return 0;
}