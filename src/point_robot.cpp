/**
 * OMPL Example 1: 2D Point Robot Planning
 * 
 * This demonstrates the fundamentals:
 * - RealVectorStateSpace for R^2
 * - Custom validity checker (circular obstacle)
 * - SimpleSetup convenience class
 * - RRTConnect planner
 * - Path simplification and interpolation
 */

 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/geometric/planners/rrt/RRTConnect.h>
 #include <ompl/geometric/planners/rrt/RRTstar.h>
 #include <ompl/geometric/SimpleSetup.h>
 #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
 #include <iostream>
 #include <cmath>
 #include <vector>
 
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
 
 // Environment: 10x10 space with circular obstacles
 struct Environment
 {
     struct Obstacle
     {
         double cx, cy, radius;
     };
     
     std::vector<Obstacle> obstacles;
     
     Environment()
     {
         // Add some circular obstacles
         obstacles.push_back({5.0, 5.0, 2.0});   // Center obstacle
         obstacles.push_back({2.5, 7.5, 1.0});   // Upper-left
         obstacles.push_back({7.5, 2.5, 1.0});   // Lower-right
     }
     
     bool isCollisionFree(double x, double y) const
     {
         for (const auto &obs : obstacles)
         {
             double dx = x - obs.cx;
             double dy = y - obs.cy;
             if (dx * dx + dy * dy <= obs.radius * obs.radius)
                 return false;
         }
         return true;
     }
 };
 
 // Global environment (in practice, pass via closure or class)
 Environment env;
 
 // State validity checker function
 bool isStateValid(const ob::State *state)
 {
     // Cast to RealVectorStateSpace::StateType to access values
     const auto *rvState = state->as<ob::RealVectorStateSpace::StateType>();
     
     double x = rvState->values[0];
     double y = rvState->values[1];
     
     return env.isCollisionFree(x, y);
 }
 
 void planWithRRTConnect()
 {
     std::cout << "=== Planning with RRTConnect ===\n\n";
     
     // 1. Construct the state space: R^2
     // The dimension is specified in the constructor
     auto space = std::make_shared<ob::RealVectorStateSpace>(2);
     
     // 2. Set the bounds for the state space
     // Without bounds, sampling won't work properly
     ob::RealVectorBounds bounds(2);
     bounds.setLow(0.0);    // Lower bound for all dimensions
     bounds.setHigh(10.0);  // Upper bound for all dimensions
     space->setBounds(bounds);
     
     // 3. Create the SimpleSetup object
     // This is a convenience class that manages:
     // - SpaceInformation
     // - ProblemDefinition
     // - Planner (creates a default one if not specified)
     og::SimpleSetup ss(space);
     
     // 4. Set the state validity checker
     // This is CRITICAL - OMPL doesn't know about collision checking
     // You provide the function that determines if a state is valid
     ss.setStateValidityChecker(isStateValid);
     
     // 5. Define start state
     ob::ScopedState<ob::RealVectorStateSpace> start(space);
     start[0] = 1.0;  // x
     start[1] = 1.0;  // y
     
     // 6. Define goal state
     ob::ScopedState<ob::RealVectorStateSpace> goal(space);
     goal[0] = 9.0;  // x
     goal[1] = 9.0;  // y
     
     // 7. Set start and goal
     ss.setStartAndGoalStates(start, goal);
     
     // 8. Create and set the planner
     auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
     
     // Configure planner parameters
     planner->setRange(0.5);  // Maximum length of a motion to be added to the tree
     
     ss.setPlanner(planner);
     
     // 9. Attempt to solve the problem
     // The argument is the maximum time allowed for planning
     std::cout << "Starting planning...\n";
     ob::PlannerStatus solved = ss.solve(5.0);  // 5 second timeout
     
     if (solved)
     {
         std::cout << "Solution found!\n\n";
         
         // 10. Get the solution path
         og::PathGeometric &path = ss.getSolutionPath();
         
         std::cout << "Raw path has " << path.getStateCount() << " states\n";
         
         // 11. Simplify the path (remove unnecessary waypoints)
         ss.simplifySolution();
         std::cout << "Simplified path has " << path.getStateCount() << " states\n";
         
         // 12. Interpolate for smooth visualization
         // This adds intermediate states along the path
         path.interpolate(30);
         std::cout << "Interpolated path has " << path.getStateCount() << " states\n\n";
         
         // 13. Print the path
         std::cout << "Path waypoints (x, y):\n";
         for (size_t i = 0; i < path.getStateCount(); ++i)
         {
             const auto *s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
             std::cout << "  [" << i << "] (" << s->values[0] << ", " << s->values[1] << ")\n";
         }
         
         // 14. Check path length
         std::cout << "\nPath length: " << path.length() << "\n";
     }
     else
     {
         std::cout << "No solution found within time limit.\n";
     }
 }
 
 void planWithRRTstar()
 {
     std::cout << "\n=== Planning with RRT* (Optimal) ===\n\n";
     
     auto space = std::make_shared<ob::RealVectorStateSpace>(2);
     
     ob::RealVectorBounds bounds(2);
     bounds.setLow(0.0);
     bounds.setHigh(10.0);
     space->setBounds(bounds);
     
     og::SimpleSetup ss(space);
     ss.setStateValidityChecker(isStateValid);
     
     ob::ScopedState<ob::RealVectorStateSpace> start(space);
     start[0] = 1.0;
     start[1] = 1.0;
     
     ob::ScopedState<ob::RealVectorStateSpace> goal(space);
     goal[0] = 9.0;
     goal[1] = 9.0;
     
     ss.setStartAndGoalStates(start, goal);
     
     // Set optimization objective (minimize path length)
     auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(
         ss.getSpaceInformation());
     ss.getProblemDefinition()->setOptimizationObjective(objective);
     
     // Create RRT* planner
     auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
     planner->setRange(0.5);
     planner->setGoalBias(0.05);      // 5% chance to sample goal
     planner->setInformedSampling(true);  // Focus sampling after solution found
     
     ss.setPlanner(planner);
     
     std::cout << "Starting RRT* planning (will optimize over time)...\n";
     
     // Solve for longer to allow optimization
     ob::PlannerStatus solved = ss.solve(10.0);
     
     if (solved)
     {
         og::PathGeometric &path = ss.getSolutionPath();
         
         std::cout << "Solution found!\n";
         std::cout << "Path cost (length): " << path.length() << "\n";
         std::cout << "Path has " << path.getStateCount() << " states\n\n";
         
         // RRT* paths are already optimized, but we can still simplify
         ss.simplifySolution();
         
         path.interpolate(30);
         
         std::cout << "Final path waypoints:\n";
         for (size_t i = 0; i < path.getStateCount(); ++i)
         {
             const auto *s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
             std::cout << "  [" << i << "] (" << s->values[0] << ", " << s->values[1] << ")\n";
         }
     }
 }
 
 void demonstrateLowLevelAPI()
 {
     std::cout << "\n=== Low-Level API (Without SimpleSetup) ===\n\n";
     
     // This shows what SimpleSetup does behind the scenes
     
     // 1. State space
     auto space = std::make_shared<ob::RealVectorStateSpace>(2);
     ob::RealVectorBounds bounds(2);
     bounds.setLow(0.0);
     bounds.setHigh(10.0);
     space->setBounds(bounds);
     
     // 2. Space information (combines space with validity checking)
     auto si = std::make_shared<ob::SpaceInformation>(space);
     si->setStateValidityChecker(isStateValid);
     
     // Set collision checking resolution (how finely to check motion validity)
     // 0.01 means 1% of the space extent
     si->setStateValidityCheckingResolution(0.01);
     
     // Must call setup() to finalize SpaceInformation
     si->setup();
     
     // 3. Problem definition
     auto pdef = std::make_shared<ob::ProblemDefinition>(si);
     
     // Create states manually
     ob::ScopedState<> start(space);
     start[0] = 1.0;
     start[1] = 1.0;
     
     ob::ScopedState<> goal(space);
     goal[0] = 9.0;
     goal[1] = 9.0;
     
     pdef->setStartAndGoalStates(start, goal);
     
     // 4. Create planner
     auto planner = std::make_shared<og::RRTConnect>(si);
     planner->setProblemDefinition(pdef);
     planner->setup();
     
     // 5. Solve
     ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(5.0));
     
     if (solved)
     {
         std::cout << "Low-level API: Solution found!\n";
         ob::PathPtr path = pdef->getSolutionPath();
         std::cout << "Path length: " << path->length() << "\n";
     }
     
     // 6. You can extract planner data (the tree structure)
     ob::PlannerData data(si);
     planner->getPlannerData(data);
     std::cout << "Planner explored " << data.numVertices() << " vertices\n";
     std::cout << "Planner created " << data.numEdges() << " edges\n";
 }
 
 int main()
 {
     std::cout << "OMPL Point Robot Planning Examples\n";
     std::cout << "===================================\n\n";
     
     std::cout << "Environment:\n";
     std::cout << "  - 10x10 workspace\n";
     std::cout << "  - Circular obstacle at (5,5) with radius 2\n";
     std::cout << "  - Circular obstacle at (2.5,7.5) with radius 1\n";
     std::cout << "  - Circular obstacle at (7.5,2.5) with radius 1\n";
     std::cout << "  - Start: (1, 1)\n";
     std::cout << "  - Goal: (9, 9)\n\n";
     
     planWithRRTConnect();
     planWithRRTstar();
     demonstrateLowLevelAPI();
     
     return 0;
 }