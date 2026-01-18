/**
 * OMPL Example 2: SE3 Rigid Body Planning
 * 
 * This demonstrates:
 * - SE3StateSpace (position + orientation)
 * - Custom StateValidityChecker class
 * - Manual SpaceInformation setup
 * - Optimization objectives
 * - Multiple planner comparison
 */

 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/SE3StateSpace.h>
 #include <ompl/geometric/planners/rrt/RRTstar.h>
 #include <ompl/geometric/planners/rrt/RRTConnect.h>
 #include <ompl/geometric/planners/prm/PRM.h>
 #include <ompl/geometric/planners/kpiece/KPIECE1.h>
 #include <ompl/geometric/PathSimplifier.h>
 #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
 #include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
 #include <ompl/base/PlannerData.h>
 #include <iostream>
 #include <cmath>
 #include <vector>
 #include <chrono>
 
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
 
 /**
  * Custom StateValidityChecker
  * 
  * In real applications, this would interface with your collision checking library
  * (FCL, Bullet, etc.). Here we implement simple box/sphere collision checks.
  */
 class RigidBodyValidityChecker : public ob::StateValidityChecker
 {
 public:
     RigidBodyValidityChecker(const ob::SpaceInformationPtr &si)
         : ob::StateValidityChecker(si)
     {
         // Define some obstacles (axis-aligned bounding boxes)
         // Each box is defined by min and max corners
         obstacles_.push_back({{2.0, 2.0, 2.0}, {4.0, 4.0, 4.0}});  // Central box
         obstacles_.push_back({{6.0, 0.0, 0.0}, {8.0, 2.0, 2.0}});  // Lower corner
         obstacles_.push_back({{0.0, 6.0, 6.0}, {2.0, 8.0, 8.0}});  // Upper corner
     }
     
     /**
      * isValid() is the core method - determines if a state is collision-free
      * MUST be thread-safe since planners may call it from multiple threads
      */
     bool isValid(const ob::State *state) const override
     {
         // First check bounds (this is fast)
         if (!si_->satisfiesBounds(state))
             return false;
         
         // Extract position from SE3 state
         const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
         double x = se3state->getX();
         double y = se3state->getY();
         double z = se3state->getZ();
         
         // In reality, you'd also use the rotation for collision checking
         // const auto &rot = se3state->rotation();
         // rot.x, rot.y, rot.z, rot.w give the quaternion
         
         // Simple point-in-box collision check
         // In practice, you'd transform the robot geometry by the SE3 pose
         // and check against obstacles using a collision library
         for (const auto &box : obstacles_)
         {
             if (x >= box.min[0] && x <= box.max[0] &&
                 y >= box.min[1] && y <= box.max[1] &&
                 z >= box.min[2] && z <= box.max[2])
             {
                 return false;  // Collision!
             }
         }
         
         return true;  // Valid state
     }
     
     /**
      * clearance() returns the distance to the nearest obstacle
      * This is optional but enables clearance-based optimization
      */
     double clearance(const ob::State *state) const override
     {
         const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
         double x = se3state->getX();
         double y = se3state->getY();
         double z = se3state->getZ();
         
         double minDist = std::numeric_limits<double>::infinity();
         
         for (const auto &box : obstacles_)
         {
             // Distance to box (clamped)
             double dx = std::max({box.min[0] - x, 0.0, x - box.max[0]});
             double dy = std::max({box.min[1] - y, 0.0, y - box.max[1]});
             double dz = std::max({box.min[2] - z, 0.0, z - box.max[2]});
             double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
             
             minDist = std::min(minDist, dist);
         }
         
         return minDist;
     }
     
 private:
     struct AABB
     {
         double min[3];
         double max[3];
     };
     
     std::vector<AABB> obstacles_;
 };
 
 
 /**
  * Demonstrates SE3 planning with multiple planners
  */
 class SE3Planner
 {
 public:
     SE3Planner()
     {
         // 1. Create SE3 state space
         // SE3 = R^3 (position) x SO3 (orientation as quaternion)
         space_ = std::make_shared<ob::SE3StateSpace>();
         
         // 2. Set bounds for the position component
         ob::RealVectorBounds bounds(3);
         bounds.setLow(0.0);
         bounds.setHigh(10.0);
         space_->setBounds(bounds);
         
         // 3. Create SpaceInformation
         si_ = std::make_shared<ob::SpaceInformation>(space_);
         
         // 4. Set validity checker
         validityChecker_ = std::make_shared<RigidBodyValidityChecker>(si_);
         si_->setStateValidityChecker(validityChecker_);
         
         // 5. Set motion validity resolution
         // This determines how finely motions are checked for collisions
         // Lower values are safer but slower
         si_->setStateValidityCheckingResolution(0.005);  // 0.5%
         
         // 6. Setup must be called before planning
         si_->setup();
         
         // Print space info
         std::cout << "State space has " << space_->getDimension() << " dimensions\n";
         std::cout << "State validity checking resolution: " 
                   << si_->getStateValidityCheckingResolution() << "\n\n";
     }
     
     void setStartGoal(double sx, double sy, double sz,
                       double gx, double gy, double gz)
     {
         // Create problem definition
         pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
         
         // Start state
         ob::ScopedState<ob::SE3StateSpace> start(space_);
         start->setXYZ(sx, sy, sz);
         start->rotation().setIdentity();  // No rotation
         
         // Goal state  
         ob::ScopedState<ob::SE3StateSpace> goal(space_);
         goal->setXYZ(gx, gy, gz);
         goal->rotation().setIdentity();
         
         pdef_->setStartAndGoalStates(start, goal);
         
         // Set optimization objective
         auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
         pdef_->setOptimizationObjective(objective);
     }
     
     void solveWithPlanner(const std::string &plannerName, double timeout)
     {
         std::cout << "=== Solving with " << plannerName << " ===\n";
         
         // Create planner based on name
         ob::PlannerPtr planner;
         
         if (plannerName == "RRTConnect")
         {
             auto rrtc = std::make_shared<og::RRTConnect>(si_);
             rrtc->setRange(0.5);
             planner = rrtc;
         }
         else if (plannerName == "RRTstar")
         {
             auto rrtstar = std::make_shared<og::RRTstar>(si_);
             rrtstar->setRange(0.5);
             rrtstar->setGoalBias(0.05);
             rrtstar->setInformedSampling(true);
             planner = rrtstar;
         }
         else if (plannerName == "PRM")
         {
             auto prm = std::make_shared<og::PRM>(si_);
             planner = prm;
         }
         else if (plannerName == "KPIECE1")
         {
             auto kpiece = std::make_shared<og::KPIECE1>(si_);
             kpiece->setRange(0.5);
             planner = kpiece;
         }
         else
         {
             std::cerr << "Unknown planner: " << plannerName << "\n";
             return;
         }
         
         planner->setProblemDefinition(pdef_);
         planner->setup();
         
         // Clear any previous solution
         pdef_->clearSolutionPaths();
         
         // Solve
         auto startTime = std::chrono::high_resolution_clock::now();
         ob::PlannerStatus solved = planner->solve(timeout);
         auto endTime = std::chrono::high_resolution_clock::now();
         
         double duration = std::chrono::duration<double>(endTime - startTime).count();
         
         if (solved)
         {
             std::cout << "Solution found in " << duration << " seconds\n";
             
             // Get the solution path
             ob::PathPtr path = pdef_->getSolutionPath();
             auto *geoPath = path->as<og::PathGeometric>();
             
             std::cout << "Raw path: " << geoPath->getStateCount() << " states, "
                       << "length: " << geoPath->length() << "\n";
             
             // Simplify the path
             og::PathSimplifier simplifier(si_);
             simplifier.simplifyMax(*geoPath);
             
             std::cout << "Simplified: " << geoPath->getStateCount() << " states, "
                       << "length: " << geoPath->length() << "\n";
             
             // Extract planner data (tree/graph structure)
             ob::PlannerData data(si_);
             planner->getPlannerData(data);
             std::cout << "Explored " << data.numVertices() << " vertices, "
                       << data.numEdges() << " edges\n";
             
             // Print first few and last waypoints
             std::cout << "Path waypoints:\n";
             size_t n = geoPath->getStateCount();
             size_t show = std::min(n, (size_t)5);
             
             for (size_t i = 0; i < show; ++i)
             {
                 printState(geoPath->getState(i), i);
             }
             if (n > show)
             {
                 std::cout << "  ... (" << n - show - 1 << " more) ...\n";
                 printState(geoPath->getState(n - 1), n - 1);
             }
         }
         else
         {
             std::cout << "No solution found in " << duration << " seconds\n";
         }
         
         // Clear planner for next run
         planner->clear();
         
         std::cout << "\n";
     }
     
 private:
     void printState(const ob::State *state, size_t idx)
     {
         const auto *se3 = state->as<ob::SE3StateSpace::StateType>();
         std::cout << "  [" << idx << "] pos=(" 
                   << se3->getX() << ", " 
                   << se3->getY() << ", " 
                   << se3->getZ() << ") rot=(w="
                   << se3->rotation().w << ")\n";
     }
     
     std::shared_ptr<ob::SE3StateSpace> space_;
     ob::SpaceInformationPtr si_;
     ob::ProblemDefinitionPtr pdef_;
     std::shared_ptr<RigidBodyValidityChecker> validityChecker_;
 };
 
 
 /**
  * Demonstrates CompoundStateSpace for multi-robot or complex systems
  */
 void demonstrateCompoundSpace()
 {
     std::cout << "\n=== Compound State Space Example ===\n\n";
     
     // Create compound space: position + velocity
     auto posSpace = std::make_shared<ob::RealVectorStateSpace>(3);  // x, y, z
     auto velSpace = std::make_shared<ob::RealVectorStateSpace>(3);  // vx, vy, vz
     
     // Set bounds for each subspace
     ob::RealVectorBounds posBounds(3), velBounds(3);
     posBounds.setLow(-10); posBounds.setHigh(10);
     velBounds.setLow(-5); velBounds.setHigh(5);
     posSpace->setBounds(posBounds);
     velSpace->setBounds(velBounds);
     
     // Create compound space
     auto compoundSpace = std::make_shared<ob::CompoundStateSpace>();
     compoundSpace->addSubspace(posSpace, 1.0);  // weight = 1.0
     compoundSpace->addSubspace(velSpace, 0.5);  // weight = 0.5 (less important)
     
     std::cout << "Created compound space with " 
               << compoundSpace->getDimension() << " dimensions\n";
     std::cout << "  - Position subspace: 3D\n";
     std::cout << "  - Velocity subspace: 3D\n";
     
     // Accessing compound state values
     ob::ScopedState<ob::CompoundStateSpace> state(compoundSpace);
     
     // Access position component
     auto *posComponent = state->as<ob::RealVectorStateSpace::StateType>(0);
     posComponent->values[0] = 1.0;
     posComponent->values[1] = 2.0;
     posComponent->values[2] = 3.0;
     
     // Access velocity component
     auto *velComponent = state->as<ob::RealVectorStateSpace::StateType>(1);
     velComponent->values[0] = 0.5;
     velComponent->values[1] = 0.3;
     velComponent->values[2] = 0.1;
     
     std::cout << "\nSample state:\n";
     std::cout << "  Position: (" << posComponent->values[0] << ", "
               << posComponent->values[1] << ", "
               << posComponent->values[2] << ")\n";
     std::cout << "  Velocity: (" << velComponent->values[0] << ", "
               << velComponent->values[1] << ", "
               << velComponent->values[2] << ")\n";
 }
 
 
 int main()
 {
     std::cout << "OMPL SE3 Rigid Body Planning Example\n";
     std::cout << "=====================================\n\n";
     
     std::cout << "Environment:\n";
     std::cout << "  - 10x10x10 workspace\n";
     std::cout << "  - Box obstacle at (2,2,2)-(4,4,4)\n";
     std::cout << "  - Box obstacle at (6,0,0)-(8,2,2)\n";
     std::cout << "  - Box obstacle at (0,6,6)-(2,8,8)\n";
     std::cout << "  - Start: (0.5, 0.5, 0.5)\n";
     std::cout << "  - Goal: (9.5, 9.5, 9.5)\n\n";
     
     SE3Planner planner;
     planner.setStartGoal(0.5, 0.5, 0.5, 9.5, 9.5, 9.5);
     
     // Compare different planners
     planner.solveWithPlanner("RRTConnect", 5.0);
     planner.solveWithPlanner("RRTstar", 10.0);
     planner.solveWithPlanner("KPIECE1", 5.0);
     
     // Demonstrate compound state space
     demonstrateCompoundSpace();
     
     return 0;
 }