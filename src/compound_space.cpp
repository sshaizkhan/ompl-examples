/**
 * OMPL Example 3: Compound State Space and Custom State Space
 * 
 * This demonstrates:
 * - CompoundStateSpace for combining multiple spaces
 * - Weighted subspaces for distance calculation
 * - Custom projection evaluator for KPIECE
 * - Multiple robots in same space
 * - Goal regions (not just goal states)
 */

 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/base/spaces/SO2StateSpace.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/base/ProjectionEvaluator.h>
 #include <ompl/base/goals/GoalRegion.h>
 #include <ompl/geometric/planners/rrt/RRTConnect.h>
 #include <ompl/geometric/planners/kpiece/KPIECE1.h>
 #include <ompl/geometric/SimpleSetup.h>
 #include <iostream>
 #include <cmath>
 
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
 
 
 /**
  * Example 3A: Multi-Robot Planning
  * 
  * Two 2D point robots that must not collide with obstacles or each other
  */
 namespace MultiRobot
 {
     // Environment dimensions
     const double ENV_SIZE = 10.0;
     
     // Circular obstacles
     struct Circle { double x, y, r; };
     std::vector<Circle> obstacles = {
         {5.0, 5.0, 1.5},  // Center
         {2.5, 7.5, 1.0},
         {7.5, 2.5, 1.0}
     };
     
     // Robot radius for inter-robot collision checking
     const double ROBOT_RADIUS = 0.3;
     
     /**
      * State is: (x1, y1, x2, y2) - positions of both robots
      */
     bool isStateValid(const ob::State *state)
     {
         const auto *rvState = state->as<ob::RealVectorStateSpace::StateType>();
         
         double x1 = rvState->values[0];
         double y1 = rvState->values[1];
         double x2 = rvState->values[2];
         double y2 = rvState->values[3];
         
         // Check robot 1 against obstacles
         for (const auto &obs : obstacles)
         {
             double d1 = std::hypot(x1 - obs.x, y1 - obs.y);
             if (d1 < obs.r + ROBOT_RADIUS) return false;
             
             double d2 = std::hypot(x2 - obs.x, y2 - obs.y);
             if (d2 < obs.r + ROBOT_RADIUS) return false;
         }
         
         // Check inter-robot collision
         double robotDist = std::hypot(x1 - x2, y1 - y2);
         if (robotDist < 2 * ROBOT_RADIUS) return false;
         
         return true;
     }
     
     /**
      * Custom projection for KPIECE
      * Projects 4D state to 2D (average position of robots)
      */
     class MultiRobotProjection : public ob::ProjectionEvaluator
     {
     public:
         MultiRobotProjection(const ob::StateSpacePtr &space)
             : ob::ProjectionEvaluator(space)
         {
         }
         
         unsigned int getDimension() const override
         {
             return 2;  // 2D projection
         }
         
         void defaultCellSizes() override
         {
             cellSizes_.resize(2);
             cellSizes_[0] = 0.5;  // Grid cell size in x
             cellSizes_[1] = 0.5;  // Grid cell size in y
         }
         
         void project(const ob::State *state, 
                      Eigen::Ref<Eigen::VectorXd> projection) const override
         {
             const auto *rvState = state->as<ob::RealVectorStateSpace::StateType>();
             
             // Project to average position of both robots
             projection[0] = (rvState->values[0] + rvState->values[2]) / 2.0;
             projection[1] = (rvState->values[1] + rvState->values[3]) / 2.0;
         }
     };
     
     void run()
     {
         std::cout << "=== Multi-Robot Planning (2 point robots) ===\n\n";
         
         // 4D state space: (x1, y1, x2, y2)
         auto space = std::make_shared<ob::RealVectorStateSpace>(4);
         
         ob::RealVectorBounds bounds(4);
         bounds.setLow(0.0);
         bounds.setHigh(ENV_SIZE);
         space->setBounds(bounds);
         
         // Register custom projection
         space->registerDefaultProjection(
             std::make_shared<MultiRobotProjection>(space));
         
         og::SimpleSetup ss(space);
         ss.setStateValidityChecker(isStateValid);
         
         // Start: robots at opposite corners
         ob::ScopedState<> start(space);
         start[0] = 1.0; start[1] = 1.0;  // Robot 1 at (1,1)
         start[2] = 9.0; start[3] = 1.0;  // Robot 2 at (9,1)
         
         // Goal: robots swap positions
         ob::ScopedState<> goal(space);
         goal[0] = 9.0; goal[1] = 9.0;  // Robot 1 to (9,9)
         goal[2] = 1.0; goal[3] = 9.0;  // Robot 2 to (1,9)
         
         ss.setStartAndGoalStates(start, goal);
         
         // Use KPIECE which benefits from projection
         auto planner = std::make_shared<og::KPIECE1>(ss.getSpaceInformation());
         planner->setRange(0.5);
         ss.setPlanner(planner);
         
         ob::PlannerStatus solved = ss.solve(10.0);
         
         if (solved)
         {
             std::cout << "Solution found!\n";
             ss.simplifySolution();
             
             og::PathGeometric &path = ss.getSolutionPath();
             path.interpolate(20);
             
             std::cout << "Path (" << path.getStateCount() << " states):\n";
             for (size_t i = 0; i < path.getStateCount(); i += 4)
             {
                 const auto *s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                 std::cout << "  Robot1: (" << s->values[0] << ", " << s->values[1] << ") "
                           << "Robot2: (" << s->values[2] << ", " << s->values[3] << ")\n";
             }
         }
         else
         {
             std::cout << "No solution found.\n";
         }
         
         std::cout << "\n";
     }
 }
 
 
 /**
  * Example 3B: Mobile Robot with Heading (SE2-like using compound space)
  * 
  * Demonstrates building SE2 manually from R2 + SO2
  */
 namespace MobileRobot
 {
     bool isStateValid(const ob::State *state)
     {
         // Access compound state
         const auto *compState = state->as<ob::CompoundStateSpace::StateType>();
         
         // Get position subspace (index 0)
         const auto *posState = compState->as<ob::RealVectorStateSpace::StateType>(0);
         double x = posState->values[0];
         double y = posState->values[1];
         
         // Get heading subspace (index 1)
         // const auto *rotState = compState->as<ob::SO2StateSpace::StateType>(1);
         // double theta = rotState->value;  // Angle in radians
         
         // Simple obstacle check (circle at center)
         double dx = x - 5.0;
         double dy = y - 5.0;
         return (dx * dx + dy * dy) > 4.0;
     }
     
     void run()
     {
         std::cout << "=== Mobile Robot (SE2 via Compound Space) ===\n\n";
         
         // Create subspaces
         auto posSpace = std::make_shared<ob::RealVectorStateSpace>(2);
         auto rotSpace = std::make_shared<ob::SO2StateSpace>();
         
         // Set position bounds
         ob::RealVectorBounds posBounds(2);
         posBounds.setLow(0.0);
         posBounds.setHigh(10.0);
         posSpace->setBounds(posBounds);
         
         // Combine into compound space
         auto space = std::make_shared<ob::CompoundStateSpace>();
         space->addSubspace(posSpace, 1.0);  // Position with weight 1
         space->addSubspace(rotSpace, 0.5);  // Rotation with weight 0.5
         
         std::cout << "Compound space dimension: " << space->getDimension() << "\n";
         std::cout << "  Position subspace: " << posSpace->getDimension() << "D\n";
         std::cout << "  Rotation subspace: " << rotSpace->getDimension() << "D\n\n";
         
         og::SimpleSetup ss(space);
         ss.setStateValidityChecker(isStateValid);
         
         // Set start state
         ob::ScopedState<ob::CompoundStateSpace> start(space);
         start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = 1.0;
         start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = 1.0;
         start->as<ob::SO2StateSpace::StateType>(1)->value = 0.0;  // Facing right
         
         // Set goal state
         ob::ScopedState<ob::CompoundStateSpace> goal(space);
         goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = 9.0;
         goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = 9.0;
         goal->as<ob::SO2StateSpace::StateType>(1)->value = M_PI / 2.0;  // Facing up
         
         ss.setStartAndGoalStates(start, goal);
         
         ob::PlannerStatus solved = ss.solve(5.0);
         
         if (solved)
         {
             std::cout << "Solution found!\n";
             ss.simplifySolution();
             
             og::PathGeometric &path = ss.getSolutionPath();
             
             std::cout << "Path (" << path.getStateCount() << " states):\n";
             for (size_t i = 0; i < path.getStateCount(); ++i)
             {
                 const auto *s = path.getState(i)->as<ob::CompoundStateSpace::StateType>();
                 const auto *pos = s->as<ob::RealVectorStateSpace::StateType>(0);
                 const auto *rot = s->as<ob::SO2StateSpace::StateType>(1);
                 
                 std::cout << "  [" << i << "] pos=(" << pos->values[0] << ", " 
                           << pos->values[1] << ") theta=" << rot->value << " rad\n";
             }
         }
         
         std::cout << "\n";
     }
 }
 
 
 /**
  * Example 3C: Goal Region (not just a single goal state)
  * 
  * Sometimes the goal is a region, not a specific configuration
  */
 namespace GoalRegionExample
 {
     // Goal is any state within a sphere centered at (9, 9) with radius 1
     class CircularGoalRegion : public ob::GoalRegion
     {
     public:
         CircularGoalRegion(const ob::SpaceInformationPtr &si,
                           double cx, double cy, double radius)
             : ob::GoalRegion(si), cx_(cx), cy_(cy), radius_(radius)
         {
             // Set the threshold for isSatisfied()
             setThreshold(radius);
         }
         
         double distanceGoal(const ob::State *state) const override
         {
             const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
             double dx = s->values[0] - cx_;
             double dy = s->values[1] - cy_;
             return std::sqrt(dx * dx + dy * dy);
         }
         
     private:
         double cx_, cy_, radius_;
     };
     
     bool isStateValid(const ob::State *state)
     {
         const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
         double x = s->values[0];
         double y = s->values[1];
         
         // Obstacle at center
         return (std::hypot(x - 5, y - 5) > 2.0);
     }
     
     void run()
     {
         std::cout << "=== Goal Region Example ===\n\n";
         
         auto space = std::make_shared<ob::RealVectorStateSpace>(2);
         ob::RealVectorBounds bounds(2);
         bounds.setLow(0.0);
         bounds.setHigh(10.0);
         space->setBounds(bounds);
         
         auto si = std::make_shared<ob::SpaceInformation>(space);
         si->setStateValidityChecker(isStateValid);
         si->setup();
         
         auto pdef = std::make_shared<ob::ProblemDefinition>(si);
         
         // Start state
         ob::ScopedState<> start(space);
         start[0] = 1.0;
         start[1] = 1.0;
         pdef->addStartState(start);
         
         // Goal region: circle at (9, 9) with radius 1
         auto goalRegion = std::make_shared<CircularGoalRegion>(si, 9.0, 9.0, 1.0);
         pdef->setGoal(goalRegion);
         
         std::cout << "Start: (1, 1)\n";
         std::cout << "Goal: any point within radius 1 of (9, 9)\n\n";
         
         auto planner = std::make_shared<og::RRTConnect>(si);
         planner->setProblemDefinition(pdef);
         planner->setup();
         
         ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(5.0));
         
         if (solved)
         {
             std::cout << "Solution found!\n";
             ob::PathPtr path = pdef->getSolutionPath();
             auto *geoPath = path->as<og::PathGeometric>();
             
             // Check final state
             const auto *finalState = geoPath->getState(geoPath->getStateCount() - 1)
                                             ->as<ob::RealVectorStateSpace::StateType>();
             
             double distToGoalCenter = std::hypot(finalState->values[0] - 9.0,
                                                   finalState->values[1] - 9.0);
             
             std::cout << "Final position: (" << finalState->values[0] << ", " 
                       << finalState->values[1] << ")\n";
             std::cout << "Distance to goal center: " << distToGoalCenter 
                       << " (threshold: 1.0)\n";
         }
         
         std::cout << "\n";
     }
 }
 
 
 /**
  * Example 3D: Demonstrate distance functions in state spaces
  */
 namespace DistanceDemo
 {
     void run()
     {
         std::cout << "=== State Space Distance Functions ===\n\n";
         
         // R^2 uses Euclidean distance by default
         auto r2Space = std::make_shared<ob::RealVectorStateSpace>(2);
         ob::RealVectorBounds bounds(2);
         bounds.setLow(0.0);
         bounds.setHigh(10.0);
         r2Space->setBounds(bounds);
         
         ob::ScopedState<ob::RealVectorStateSpace> s1(r2Space), s2(r2Space);
         s1[0] = 0.0; s1[1] = 0.0;
         s2[0] = 3.0; s2[1] = 4.0;
         
         std::cout << "R^2 Space:\n";
         std::cout << "  s1 = (0, 0), s2 = (3, 4)\n";
         std::cout << "  Euclidean distance: " << r2Space->distance(s1.get(), s2.get()) << "\n\n";
         
         // SO2 uses angular distance
         auto so2Space = std::make_shared<ob::SO2StateSpace>();
         
         ob::ScopedState<ob::SO2StateSpace> a1(so2Space), a2(so2Space);
         a1->value = 0.0;
         a2->value = M_PI + 0.1;  // Just past π
         
         std::cout << "SO2 Space:\n";
         std::cout << "  theta1 = 0, theta2 = π + 0.1\n";
         std::cout << "  Angular distance (wraps around): " 
                   << so2Space->distance(a1.get(), a2.get()) << " rad\n";
         std::cout << "  (Should be close to π - 0.1 = " << M_PI - 0.1 << ")\n\n";
         
         // Compound space: weighted combination
         auto compSpace = std::make_shared<ob::CompoundStateSpace>();
         compSpace->addSubspace(r2Space, 1.0);
         compSpace->addSubspace(so2Space, 2.0);  // Rotation weighted more
         
         ob::ScopedState<ob::CompoundStateSpace> c1(compSpace), c2(compSpace);
         
         c1->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = 0.0;
         c1->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = 0.0;
         c1->as<ob::SO2StateSpace::StateType>(1)->value = 0.0;
         
         c2->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = 3.0;
         c2->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = 4.0;
         c2->as<ob::SO2StateSpace::StateType>(1)->value = M_PI / 2.0;
         
         std::cout << "Compound Space (R^2 + SO2, weights 1.0 and 2.0):\n";
         std::cout << "  Weighted distance: " << compSpace->distance(c1.get(), c2.get()) << "\n";
         std::cout << "  Components: sqrt(w1*d1^2 + w2*d2^2) = sqrt(1*25 + 2*(π/2)^2)\n";
     }
 }
 
 
 int main()
 {
     std::cout << "OMPL Compound State Space Examples\n";
     std::cout << "===================================\n\n";
     
     MultiRobot::run();
     MobileRobot::run();
     GoalRegionExample::run();
     DistanceDemo::run();
     
     return 0;
 }