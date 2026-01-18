# FANUC M710iC/50H Motion Planning with OMPL
## Comprehensive Code Documentation

---

## Table of Contents
1. [Overview](#overview)
2. [Architecture Diagram](#architecture-diagram)
3. [Class: Robot6DOF](#class-robot6dof)
4. [Class: CollisionChecker](#class-collisionchecker)
5. [Class: EndEffectorGoal](#class-endeffectorgoal)
6. [Class: RobotStateValidityChecker](#class-robotstatevaliditychecker)
7. [Class: Robot6DOFPlanner](#class-robot6dofplanner)
8. [Planning Flow](#planning-flow)
9. [Key Algorithms](#key-algorithms)

---

## Overview

This code implements motion planning for a **FANUC M710iC/50H** industrial robot arm using the **Open Motion Planning Library (OMPL)**. 

### The Problem
```
INPUT:
  ├── Start: 6 joint angles (radians)
  └── Goal: Either
        ├── 6 joint angles (joint-space goal), OR
        └── End-effector pose (position + orientation in task-space)

OUTPUT:
  └── Trajectory: Sequence of 50 joint configurations from start to goal
                  that avoids all obstacles
```

### Robot Specifications
```
Robot: FANUC M710iC/50H
├── 6 revolute joints (6-DOF)
├── Mounted on 1.05m pedestal
├── Reach: ~2.05m from base
├── OPW Kinematics (standard industrial robot model)
└── Joint limits from URDF/xacro files
```

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              USER CODE (main)                               │
│                                                                             │
│   start_joints = [0, 0.927, -0.642, 0, 0, 0]                                │
│   goal_position = (1.86, 0.0, 1.0)                                          │
│   trajectory = planner.plan(start_joints, goal_position, orientation)       │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Robot6DOFPlanner                                   │
│                                                                             │
│   ┌─────────────────┐  ┌──────────────────┐  ┌─────────────────────────┐    │
│   │   Robot6DOF     │  │ CollisionChecker │  │ OMPL Components         │    │
│   │                 │  │                  │  │                         │    │
│   │ • OPW params    │  │ • Boxes (racks)  │  │ • RealVectorStateSpace  │    │
│   │ • FK/IK         │  │ • Cylinders      │  │ • SpaceInformation      │    │
│   │ • Jacobian      │  │ • Self-collision │  │ • ProblemDefinition     │    │
│   │ • Joint limits  │  │ • isValid()      │  │ • RRTConnect planner    │    │
│   └────────┬────────┘  └────────┬─────────┘  └────────────┬────────────┘    │
│            │                    │                         │                 │
│            └────────────────────┼─────────────────────────┘                 │
│                                 │                                           │
│                                 ▼                                           │
│                    ┌────────────────────────┐                               │
│                    │ RobotStateValidity     │                               │
│                    │ Checker                │                               │
│                    │                        │                               │
│                    │ isValid(joints) →      │                               │
│                    │   collision_checker    │                               │
│                    │   .isValid(joints)     │                               │
│                    └────────────────────────┘                               │
│                                 │                                           │
│                                 ▼                                           │
│                    ┌────────────────────────┐                               │
│                    │  EndEffectorGoal       │                               │
│                    │  (GoalSampleableRegion)│                               │
│                    │                        │                               │
│                    │ • distanceGoal()       │                               │
│                    │ • sampleGoal() via IK  │                               │
│                    │ • isSatisfied()        │                               │
│                    └────────────────────────┘                               │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
                            ┌──────────────────┐
                            │   TRAJECTORY     │
                            │                  │
                            │ [q1,q2,q3,q4,q5,q6] × 50 waypoints
                            └──────────────────┘
```

---

## Class: Robot6DOF

### Purpose
Encapsulates the robot's kinematic model using **OPW (Ortho-Parallel Weld) parameters** - a standard parameterization for 6-axis industrial robots.

### OPW Parameters Explained

```
                    ┌─────┐
                    │ J6  │ ← c4 = 0.175m (wrist to flange)
                    └──┬──┘
                       │
                    ┌──┴──┐
                    │ J5  │
                    └──┬──┘
                       │
                    ┌──┴──┐
                    │ J4  │
                    └──┬──┘
                       │
         c3 = 0.970m   │   (upper arm)
                       │
                    ┌──┴──┐
                    │ J3  │
                    └──┬──┘
                       │
         c2 = 0.870m   │   (lower arm)
                       │
                    ┌──┴──┐
     a1 = 0.15m ──► │ J2  │ ◄── offset from J1 axis
                    └──┬──┘
                       │
         c1 = 0.565m   │   (shoulder height)
                       │
                    ┌──┴──┐
                    │ J1  │ ← Base rotation
                    └──┬──┘
                       │
    ════════════════════════════ Pedestal top (z = 1.05m)
                       │
         1.05m         │   (pedestal)
                       │
    ════════════════════════════ Floor (z = 0)
```

### Key Methods

#### `forwardKinematics(joints) → Matrix4d`

Computes the 4×4 transformation matrix from base to end-effector.

```cpp
Eigen::Matrix4d forwardKinematics(const std::vector<double>& joints) const
{
    // 1. Apply sign corrections and offsets to joint angles
    std::array<double, 6> q;
    for (int i = 0; i < 6; ++i)
        q[i] = opw_.sign_corrections[i] * joints[i] + opw_.offsets[i];
    
    // 2. Compute trig values
    double c1 = cos(q[0]), s1 = sin(q[0]);  // J1 angle
    double c2 = cos(q[1]), s2 = sin(q[1]);  // J2 angle
    // ... etc
    
    // 3. Wrist center position using OPW equations
    double px = (a1 + c2*c2 + c3*c23) * c1 - a2*s1;
    double py = (a1 + c2*c2 + c3*c23) * s1 + a2*c1;
    double pz = c1 + c2*s2 + c3*s23;
    
    // 4. Add tool offset and pedestal height
    // 5. Build rotation matrix from joint angles
    // 6. Return 4x4 homogeneous transform
}
```

**Visual explanation:**
```
joints = [θ1, θ2, θ3, θ4, θ5, θ6]   (what you command)
              │
              ▼
        ┌───────────┐
        │ Forward   │
        │ Kinematics│
        └───────────┘
              │
              ▼
T = │ R₁₁ R₁₂ R₁₃ Px │   (where the hand is)
    │ R₂₁ R₂₂ R₂₃ Py │
    │ R₃₁ R₃₂ R₃₃ Pz │
    │  0   0   0   1 │
    
    └─── Rot ───┘└Pos┘
```

#### `getJointPositions(joints) → vector<Vector3d>`

Returns the 3D position of each link frame (for collision checking).

```
Returns 7 positions:
  [0] f_link      - Foundation (pedestal base)
  [1] s_link      - Swing/J1 (top of pedestal)
  [2] l_link      - Lower arm/J2
  [3] u_link      - Upper arm/J3
  [4] b_link      - Wrist bend/J5
  [5] t_link      - Tool flange/J6
  [6] ee_mount    - End-effector tip
```

#### `computeJacobian6D(joints) → Matrix<6,6>`

Computes the 6×6 Jacobian matrix numerically (for IK).

```
Jacobian relates joint velocities to end-effector velocities:

┌    ┐   ┌                              ┐ ┌    ┐
│ vx │   │ J₁₁ J₁₂ J₁₃ J₁₄ J₁₅ J₁₆ │ │ q̇₁ │
│ vy │   │ J₂₁ J₂₂ J₂₃ J₂₄ J₂₅ J₂₆ │ │ q̇₂ │
│ vz │ = │ J₃₁ J₃₂ J₃₃ J₃₄ J₃₅ J₃₆ │ │ q̇₃ │
│ ωx │   │ J₄₁ J₄₂ J₄₃ J₄₄ J₄₅ J₄₆ │ │ q̇₄ │
│ ωy │   │ J₅₁ J₅₂ J₅₃ J₅₄ J₅₅ J₅₆ │ │ q̇₅ │
│ ωz │   │ J₆₁ J₆₂ J₆₃ J₆₄ J₆₅ J₆₆ │ │ q̇₆ │
└    ┘   └                              ┘ └    ┘

Linear      Angular    Joint
velocity    velocity   velocities
```

Computed numerically using finite differences:
```cpp
for (int i = 0; i < 6; ++i) {
    joints_delta[i] += delta;  // Perturb joint i
    T1 = forwardKinematics(joints_delta);
    J.col(i) = (T1_pose - T0_pose) / delta;  // Derivative
}
```

#### `solveIK(target_pos, target_orient, joints) → bool`

Solves inverse kinematics using **Damped Least Squares** method.

```
Goal: Find joint angles that place end-effector at target pose

Algorithm:
1. Compute current EE pose via FK
2. Compute error: e = [position_error; orientation_error]
3. Compute Jacobian J
4. Solve: Δq = J^T (J J^T + λ²I)^(-1) e   ← Damped least squares
5. Update: q = q + Δq
6. Repeat until converged or max iterations
```

**Why damped least squares?**
```
Regular least squares:  Δq = J⁻¹ e
  Problem: J can be singular (robot at certain poses)
  
Damped least squares:   Δq = J^T (J J^T + λ²I)⁻¹ e
  Solution: λ² term prevents division by zero
  Trade-off: Larger λ = more stable but slower convergence
```

---

## Class: CollisionChecker

### Purpose
Checks if a robot configuration collides with:
1. **Environment obstacles** (racks, conveyors, floor)
2. **Itself** (self-collision between non-adjacent links)

### Obstacle Types

```cpp
struct AABB {      // Axis-Aligned Bounding Box
    Vector3d min_pt;  // Corner with smallest x,y,z
    Vector3d max_pt;  // Corner with largest x,y,z
    string name;
};

struct Sphere {
    Vector3d center;
    double radius;
    string name;
};

struct Cylinder {
    Vector3d base_center;
    double radius;
    double height;
    string name;
};
```

### Environment Setup (from XACRO files)

```
WORKCELL LAYOUT (Top View):
                    
        rack_06_07          rack_04_05          rack_02_03
            ■                   ■                   ■
             \                  |                  /
              \                 |                 /
               \                |                /
      rack_08   \               |               /   rack_01
          ■      \              |              /        ■
                  \             |             /
                   \     ┌─────────────┐    /
                    \    │   ROBOT     │   /
                     \   │  (pedestal) │  /
                      \  │      ○      │ /
                       \ └─────────────┘/
                        \              /
                         \            /
                          \          /
        ─────────────────────────────────────────────
                                            UPS Conveyor
                                            (to the right)
```

### Key Method: `isValid(joints) → bool`

```cpp
bool isValid(const std::vector<double>& joints) const
{
    // 1. Get positions of all robot links
    std::vector<Eigen::Vector3d> positions = robot_.getJointPositions(joints);
    
    // 2. SELF-COLLISION CHECK
    //    Check distance between non-adjacent links
    for (i = 0; i < positions.size(); ++i) {
        for (j = i + 2; j < positions.size(); ++j) {  // Skip adjacent (i+1)
            if (distance(positions[i], positions[j]) < 0.05m)
                return false;  // Self-collision!
        }
    }
    
    // 3. OBSTACLE COLLISION CHECK
    //    For each link position (skip fixed base links)
    for (i = 2; i < positions.size(); ++i) {
        
        // Check against boxes (AABB)
        for (box : boxes_) {
            if (point_in_box(positions[i], box))
                return false;
        }
        
        // Check against spheres
        for (sphere : spheres_) {
            if (distance(positions[i], sphere.center) < sphere.radius)
                return false;
        }
        
        // Check against cylinders
        for (cylinder : cylinders_) {
            if (point_in_cylinder(positions[i], cylinder))
                return false;
        }
    }
    
    // 4. LINK SEGMENT CHECK
    //    Check line segments between links (not just points)
    for (i = 2; i < positions.size() - 1; ++i) {
        if (!segmentCollisionFree(positions[i], positions[i+1]))
            return false;
    }
    
    // 5. WORKSPACE BOUNDS
    if (end_effector.z < 0.05)  return false;  // Below ground
    if (distance_from_base > 2.5) return false; // Beyond reach
    
    return true;  // Configuration is valid!
}
```

---

## Class: EndEffectorGoal

### Purpose
Defines a **task-space goal** (position + orientation) that OMPL can use for planning in **joint space**.

This class inherits from `ob::GoalSampleableRegion`, which allows RRTConnect to:
1. Check if a state satisfies the goal
2. **Sample** goal states directly (using IK)

### Why GoalSampleableRegion?

```
Regular GoalRegion:
  - Planner grows tree randomly
  - Hopes to stumble into goal region
  - Can be slow for small goal regions
  
GoalSampleableRegion:
  - Planner can SAMPLE valid goal states
  - Grows tree FROM goal too (bidirectional)
  - Much faster for task-space goals

RRTConnect with sampleable goal:
  
  START ─────●                          ● ←── sampled via IK
              \                        /
               ●──●                ●──●
                   \              /
                    ●────────────●
                    
  Tree from start    Tree from goal (sampled)
```

### Key Methods

#### `distanceGoal(state) → double`

Computes how far a joint configuration is from the goal pose.

```cpp
double distanceGoal(const ob::State* state) const
{
    // Extract joints from OMPL state
    std::vector<double> joints = extract_joints(state);
    
    // Compute current EE pose via FK
    Matrix4d T = robot_.forwardKinematics(joints);
    Vector3d current_pos = robot_.getPosition(T);
    Quaterniond current_quat = robot_.getQuaternion(T);
    
    // Position error (Euclidean distance)
    double pos_error = (current_pos - target_position_).norm();
    
    // Orientation error (angle between quaternions)
    double dot = abs(current_quat.dot(target_orientation_));
    double orient_error = 2.0 * acos(min(1.0, dot));  // Radians
    
    // Weighted sum (position more important)
    return pos_error + 0.1 * orient_error;
}
```

#### `sampleGoal(state)`

Generates a valid goal joint configuration using IK.

```cpp
void sampleGoal(ob::State* state) const
{
    for (int attempt = 0; attempt < 30; ++attempt)
    {
        // Start from random configuration (seed for IK)
        sampler_->sampleUniform(state);
        std::vector<double> joints = extract_joints(state);
        
        // Run IK to find joints that reach goal pose
        if (robot_.solveIK(target_position_, target_orientation_, joints,
                           position_tolerance_, orientation_tolerance_))
        {
            // IK succeeded! Return this configuration
            set_joints(state, joints);
            return;
        }
    }
    // IK failed 30 times, return random (planner will handle)
}
```

#### `isSatisfied(state) → bool`

Checks if configuration reaches the goal within tolerances.

```cpp
bool isSatisfied(const ob::State* state) const
{
    std::vector<double> joints = extract_joints(state);
    
    Matrix4d T = robot_.forwardKinematics(joints);
    Vector3d current_pos = robot_.getPosition(T);
    Quaterniond current_quat = robot_.getQuaternion(T);
    
    // Check position tolerance (default: 1mm)
    if ((current_pos - target_position_).norm() > position_tolerance_)
        return false;
    
    // Check orientation tolerance (default: ~0.6 degrees)
    double dot = abs(current_quat.dot(target_orientation_));
    double orient_error = 2.0 * acos(min(1.0, dot));
    if (orient_error > orientation_tolerance_)
        return false;
    
    return true;
}
```

---

## Class: RobotStateValidityChecker

### Purpose
Bridges OMPL's abstract validity checking to our collision checker.

```cpp
class RobotStateValidityChecker : public ob::StateValidityChecker
{
    bool isValid(const ob::State* state) const override
    {
        // 1. Check OMPL bounds (joint limits)
        if (!si_->satisfiesBounds(state))
            return false;
        
        // 2. Extract joint values from OMPL state
        const auto* rvstate = state->as<ob::RealVectorStateSpace::StateType>();
        std::vector<double> joints(6);
        for (int i = 0; i < 6; ++i)
            joints[i] = rvstate->values[i];
        
        // 3. Delegate to our collision checker
        return collision_checker_.isValid(joints);
    }
};
```

---

## Class: Robot6DOFPlanner

### Purpose
High-level interface that orchestrates all components.

### Constructor

```cpp
Robot6DOFPlanner()
{
    // 1. Create 6D state space for joint angles
    space_ = std::make_shared<ob::RealVectorStateSpace>(6);
    
    // 2. Set joint limits as bounds
    ob::RealVectorBounds bounds(6);
    bounds.setLow(i, joint_limits_lower_[i]);
    bounds.setHigh(i, joint_limits_upper_[i]);
    space_->setBounds(bounds);
    
    // 3. Create SpaceInformation (OMPL's central hub)
    si_ = std::make_shared<ob::SpaceInformation>(space_);
    
    // 4. Attach our validity checker
    si_->setStateValidityChecker(
        std::make_shared<RobotStateValidityChecker>(si_, robot_, collision_checker_));
    
    // 5. Set motion resolution (1% of space = ~6° per check)
    si_->setStateValidityCheckingResolution(0.01);
    
    // 6. Finalize setup
    si_->setup();
}
```

### Method: `plan(start_joints, goal_joints)` — Joint-to-Joint

```cpp
std::vector<std::vector<double>> plan(
    const std::vector<double>& start_joints,
    const std::vector<double>& goal_joints,
    double planning_time)
{
    // 1. Validate both configurations
    if (!collision_checker_.isValid(start_joints)) return {};
    if (!collision_checker_.isValid(goal_joints)) return {};
    
    // 2. Create problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si_);
    
    // 3. Set start and goal states directly
    ob::ScopedState<> start(space_), goal(space_);
    for (int i = 0; i < 6; ++i) {
        start[i] = start_joints[i];
        goal[i] = goal_joints[i];
    }
    pdef->addStartState(start);
    pdef->setGoalState(goal);  // Simple goal state
    
    // 4. Plan and return trajectory
    // ...
}
```

### Method: `plan(start_joints, goal_position, goal_orientation)` — Joint-to-Pose

```cpp
std::vector<std::vector<double>> plan(
    const std::vector<double>& start_joints,
    const Eigen::Vector3d& goal_position,
    const Eigen::Quaterniond& goal_orientation,
    double planning_time)
{
    // 1. Validate start configuration
    if (!collision_checker_.isValid(start_joints)) return {};
    
    // 2. Create problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si_);
    
    // 3. Set start state
    ob::ScopedState<> start(space_);
    for (int i = 0; i < 6; ++i)
        start[i] = start_joints[i];
    pdef->addStartState(start);
    
    // 4. Set goal (task-space with IK sampling)
    auto goal = std::make_shared<EndEffectorGoal>(
        si_, robot_, goal_position, goal_orientation,
        0.001,  // 1mm position tolerance
        0.01);  // ~0.6° orientation tolerance
    pdef->setGoal(goal);
    
    // 5. Create RRTConnect planner
    auto planner = std::make_shared<og::RRTConnect>(si_);
    planner->setRange(0.5);  // Max joint step per iteration
    planner->setProblemDefinition(pdef);
    planner->setup();
    
    // 6. SOLVE!
    ob::PlannerStatus solved = planner->solve(planning_time);
    
    if (solved)
    {
        // 7. Get and process path
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        
        // 8. Simplify (remove unnecessary waypoints)
        og::PathSimplifier simplifier(si_);
        simplifier.simplifyMax(*path);
        
        // 9. Interpolate (add intermediate waypoints for smoothness)
        path->interpolate(50);
        
        // 10. Extract trajectory
        std::vector<std::vector<double>> trajectory;
        for (size_t i = 0; i < path->getStateCount(); ++i)
            trajectory.push_back(extract_joints(path->getState(i)));
        return trajectory;
    }
    
    return {};  // No solution found
}
```

---

## Planning Flow

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          PLANNING FLOW                                   │
└──────────────────────────────────────────────────────────────────────────┘

1. INITIALIZATION
   ┌─────────────────────────────────────────────────────────────────────┐
   │ • Create 6D joint space with limits                                 │
   │ • Setup collision checker with workcell obstacles                   │
   │ • Configure OMPL SpaceInformation                                   │
   └─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
2. PROBLEM DEFINITION
   ┌─────────────────────────────────────────────────────────────────────┐
   │ • Validate start configuration (not in collision)                   │
   │ • Create EndEffectorGoal with target pose                           │
   │ • Goal uses IK to sample valid goal configurations                  │
   └─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
3. RRTConnect PLANNING
   ┌─────────────────────────────────────────────────────────────────────┐
   │                                                                     │
   │    START TREE                              GOAL TREE                │
   │        ●                                       ●  ← sampled via IK  │
   │       /|\                                     /|\                   │
   │      ● ● ●      ──── tries to connect ────  ● ● ●                   │
   │     /|   |\                                /|   |\                  │
   │                                                                     │
   │  Each extension:                                                    │
   │   1. Sample random joint config                                     │
   │   2. Find nearest node in tree                                      │
   │   3. Extend toward sample (max 0.5 rad step)                        │
   │   4. Check validity (collision + limits)                            │
   │   5. Try to connect trees                                           │
   │                                                                     │
   └─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
4. PATH POST-PROCESSING
   ┌─────────────────────────────────────────────────────────────────────┐
   │                                                                     │
   │  Raw path:        ●──●──●──●──●──●──●──●──●  (many waypoints)       │
   │                                                                     │
   │  Simplified:      ●────────●────────●────●   (unnecessary removed)  │
   │                                                                     │
   │  Interpolated:    ●·●·●·●·●·●·●·●·●·●·●·●·●  (smooth trajectory)    │
   │                   └─────── 50 waypoints ──────┘                     │
   │                                                                     │
   └─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
5. OUTPUT
   ┌─────────────────────────────────────────────────────────────────────┐
   │  Trajectory: 50 waypoints, each is [q1, q2, q3, q4, q5, q6]         │
   │  Ready to send to robot controller                                  │
   └─────────────────────────────────────────────────────────────────────┘
```

---

## Key Algorithms

### RRTConnect (Bidirectional RRT)

```
WHY RRTConnect for robot arms?
  ✓ Fast for high-dimensional spaces (6D)
  ✓ Works well with narrow passages
  ✓ Bidirectional = grows from both ends
  ✓ No need for goal bias tuning

Algorithm:
  1. Initialize tree_a from start, tree_b from goal
  2. Loop:
     a. Sample random configuration q_rand
     b. Extend tree_a toward q_rand → q_new
     c. Try to connect tree_b to q_new
     d. If connected: path found!
     e. Swap tree_a and tree_b
  3. Return path through connection point
```

### Damped Least Squares IK

```
Problem: Given target pose, find joint angles
         (Used in EndEffectorGoal::sampleGoal)

Standard approach (Newton-Raphson):
  Δq = J⁻¹ · error
  Problem: J can be singular → no inverse!

Damped Least Squares:
  Δq = Jᵀ · (J·Jᵀ + λ²I)⁻¹ · error
  
  Where:
    J = 6×6 Jacobian matrix
    λ = damping factor (0.1 in our code)
    error = [position_error; orientation_error]

  Benefits:
    ✓ Always has a solution (no singularity issues)
    ✓ λ controls stability vs. speed trade-off
    ✓ Handles both position and orientation
```

### Collision Checking Strategy

```
HIERARCHICAL CHECKING (fast to slow):

1. Joint Limits (fastest)
   └─ Simple bound check: lower[i] ≤ q[i] ≤ upper[i]

2. Self-Collision
   └─ Check distances between non-adjacent links
   └─ O(n²) where n = 7 links, so 21 checks

3. Point-vs-Obstacle
   └─ Check each of 7 link positions against all obstacles
   └─ AABB: 6 comparisons per box
   └─ Sphere: 1 distance calculation
   └─ Cylinder: XY distance + Z bounds

4. Segment-vs-Obstacle (slowest)
   └─ Sample 5 points along each link segment
   └─ Check each sample point against obstacles

Early exit: Return false as soon as ANY check fails
```

---

## Summary Table

| Component | Responsibility |
|-----------|----------------|
| `Robot6DOF` | Forward/Inverse Kinematics, Jacobian, Joint limits |
| `CollisionChecker` | Environment + self-collision checking |
| `EndEffectorGoal` | Task-space goal with IK-based sampling |
| `RobotStateValidityChecker` | OMPL ↔ CollisionChecker bridge |
| `Robot6DOFPlanner` | Orchestrates planning, post-processing |
| `RRTConnect` | The actual path search algorithm |