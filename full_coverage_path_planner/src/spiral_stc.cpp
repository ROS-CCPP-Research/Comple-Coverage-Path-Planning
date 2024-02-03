#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include "full_coverage_path_planner/spiral_stc.h"
#include <pluginlib/class_list_macros.h>

using PoseStamped = geometry_msgs::PoseStamped;

static double euDist2D(const PoseStamped& p1, const PoseStamped& p2)
{
    return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2));
}

static PoseStamped parametricInterp1(const PoseStamped& p1, const PoseStamped& p2, double alpha)
{
    PoseStamped p;
    p.header.frame_id = p1.header.frame_id;
    p.pose.orientation = p1.pose.orientation;
    p.pose.position.x = p1.pose.position.x + alpha * (p2.pose.orientation.x - p1.pose.orientation.x);
    p.pose.position.y = p1.pose.position.y + alpha * (p2.pose.orientation.y - p1.pose.orientation.y);
    return p;
}

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::SpiralSTC, nav_core::BaseGlobalPlanner)

namespace full_coverage_path_planner
{
void SpiralSTC::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

  if (!initialized_)
  {

    // Create a publisher to visualize the plan
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh, private_named_nh("~/" + name);

    plan_pub_ = nh.advertise<nav_msgs::Path>("boustrophedon/path", 10,  true);
    // Try to request the cpp-grid from the cpp_grid map_server
    cpp_grid_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");

    // Define  robot radius (radius) parameter
    float robot_radius_default = 0.5f;
    // private_named_nh.param<float>("robot_radius", robot_radius_, robot_radius_default);
    ros::param::get("~robot_radius", robot_radius_);
    // Define  tool radius (radius) parameter
    float tool_radius_default = 0.5f;
    // private_named_nh.param<float>("tool_radius", tool_radius_, tool_radius_default);
    ros::param::get("~tool_radius", tool_radius_);

    ros::param::get("~robot_namespace", robotNamespace);

    ros::param::get("~start_pose", start_pose);

    

    initialized_ = true;
  }
}

std::list<gridNode_t> SpiralSTC::spiral(std::vector<std::vector<bool> > const& grid, std::list<gridNode_t>& init,
                                        std::vector<std::vector<bool> >& visited)
{
  int dx, dy, dx_prev, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();
  // Spiral filling of the open space
  // Copy incoming list to 'end'
  std::list<gridNode_t> pathNodes(init);
  // Create iterator for gridNode_t list and let it point to the last element of end
  std::list<gridNode_t>::iterator it = --(pathNodes.end());
  if (pathNodes.size() > 1)  // if list is length 1, keep iterator at end
    it--;                    // Let iterator point to second to last element

  gridNode_t prev = *(it);
  bool done = false;
  while (!done)
  {
    if (it != pathNodes.begin())
    {
      // turn ccw
      dx = pathNodes.back().pos.x - prev.pos.x;
      dy = pathNodes.back().pos.y - prev.pos.y;
      dx_prev = dx;
      dx = -dy;
      dy = dx_prev;
    }
    else
    {
      // Initialize spiral direction towards y-axis
      dx = 0;
      dy = 1;
    }
    done = true;

    for (int i = 0; i < 4; ++i)
    {
      x2 = pathNodes.back().pos.x + dx;
      y2 = pathNodes.back().pos.y + dy;
      if (x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows)
      {
        if (grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen)
        {
          Point_t new_point = { x2, y2 };
          gridNode_t new_node =
          {
            new_point,  // Point: x,y
            0,          // Cost
            0,          // Heuristic
          };
          prev = pathNodes.back();
          pathNodes.push_back(new_node);
          it = --(pathNodes.end());
          visited[y2][x2] = eNodeVisited;  // Close node
          done = false;
          break;
        }
      }
      // try next direction cw
      dx_prev = dx;
      dx = dy;
      dy = -dx_prev;
    }
  }
  return pathNodes;
}

std::list<Point_t> SpiralSTC::spiral_stc(std::vector<std::vector<bool> > const& grid,
                                          Point_t& init,
                                          int &multiple_pass_counter,
                                          int &visited_counter)
{
  int x, y, nRows = grid.size(), nCols = grid[0].size();
  // Initial node is initially set as visited so it does not count
  multiple_pass_counter = 0;
  visited_counter = 0;

  std::vector<std::vector<bool> > visited;
  visited = grid;  // Copy grid matrix
  x = init.x;
  y = init.y;

  Point_t new_point = { x, y };
  gridNode_t new_node =
  {
    new_point,  // Point: x,y
    0,          // Cost
    0,          // Heuristic
  };
  std::list<gridNode_t> pathNodes;
  std::list<Point_t> fullPath;
  pathNodes.push_back(new_node);
  visited[y][x] = eNodeVisited;

#ifdef DEBUG_PLOT
  ROS_INFO("Grid before walking is: ");
  printGrid(grid, visited, fullPath);
#endif

  pathNodes = SpiralSTC::spiral(grid, pathNodes, visited);                // First spiral fill
  std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints
  // Add points to full path
  std::list<gridNode_t>::iterator it;
  for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
  {
    Point_t newPoint = { it->pos.x, it->pos.y };
    visited_counter++;
    fullPath.push_back(newPoint);
  }
  // Remove all elements from pathNodes list except last element
  pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));

#ifdef DEBUG_PLOT
  ROS_INFO("Current grid after first spiral is");
  printGrid(grid, visited, fullPath);
  ROS_INFO("There are %d goals remaining", goals.size());
#endif
  while (goals.size() != 0)
  {
    // Remove all elements from pathNodes list except last element.
    // The last point is the starting point for a new search and A* extends the path from there on
    pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
    visited_counter--;  // First point is already counted as visited
    // Plan to closest open Node using A*
    // `goals` is essentially the map, so we use `goals` to determine the distance from the end of a potential path
    //    to the nearest free space
    bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
    if (resign)
    {
#ifdef DEBUG_PLOT
      ROS_INFO("A_star_to_open_space is resigning", goals.size());
#endif
      break;
    }

    // Update visited grid
    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      if (visited[it->pos.y][it->pos.x])
      {
        multiple_pass_counter++;
      }
      visited[it->pos.y][it->pos.x] = eNodeVisited;
    }
    if (pathNodes.size() > 0)
    {
      multiple_pass_counter--;  // First point is already counted as visited
    }

#ifdef DEBUG_PLOT
    ROS_INFO("Grid with path marked as visited is:");
    gridNode_t SpiralStart = pathNodes.back();
    printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());
#endif

    // Spiral fill from current position
    pathNodes = spiral(grid, pathNodes, visited);

#ifdef DEBUG_PLOT
    ROS_INFO("Visited grid updated after spiral:");
    printGrid(grid, visited, pathNodes, SpiralStart, pathNodes.back());
#endif

    goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints

    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      Point_t newPoint = { it->pos.x, it->pos.y };
      visited_counter++;
      fullPath.push_back(newPoint);
    }
  }

  return fullPath;
}

bool SpiralSTC::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  else
  {
    ROS_INFO("Initialized!");
  }

  // clock_t begin_c = clock();
  Point_t startPoint;

  /********************** Get grid from server **********************/
  std::vector<std::vector<bool> > grid;
  nav_msgs::GetMap grid_req_srv;
  ROS_INFO("Requesting grid!!");
  if (!cpp_grid_client_.call(grid_req_srv))
  {
    ROS_ERROR("Could not retrieve grid from map_server");
    return false;
  }

  // Determine the type of the grid
  const std::type_info &type = typeid(grid[0][0]);

  // Print the type name
  std::cout << "Grid element type: " << type.name() << std::endl;


   // This is the point from which path planning starts

  if (!parseGrid(grid_req_srv.response.map, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
  {
    ROS_ERROR("Could not parse retrieved grid");
    return false;
  }

#ifdef DEBUG_PLOT
  ROS_INFO("Start grid is:");
  std::list<Point_t> printPath;
  printPath.push_back(startPoint);
  printGrid(grid, grid, printPath);
#endif

  int occupiedCellCount = 0;

  for (const auto &row : grid) {
    for (const auto &cell : row) {
      if (cell) {
        std::cout << "1 "; // Occupied cell
        occupiedCellCount++;
      } else {
        std::cout << "0 "; // Free cell
      }
    }
    std::cout << std::endl;
  }

std::cout << "Occupied Cell Count: " << occupiedCellCount << std::endl;


  int area = 0;
  for (const auto &row : grid) {
    for (const auto &cell : row) {
      if (cell) {
        area += (cell >= 0 && cell <= 25) ? 1 : 0;
      }
    }
  }

std::cout << "Area: " << area << std::endl;


  std::list<Point_t> goalPoints = spiral_stc(grid,
                                              startPoint,
                                              spiral_cpp_metrics_.multiple_pass_counter,
                                              spiral_cpp_metrics_.visited_counter);
  ROS_INFO("naive cpp completed!");
  ROS_INFO("Converting path to plan");

  parsePointlist2Plan(start, goalPoints, plan);
  // Print some metrics:
  spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter
                                            - spiral_cpp_metrics_.multiple_pass_counter;
  spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
  ROS_INFO("Total visited: %d", spiral_cpp_metrics_.visited_counter);
  ROS_INFO("Total re-visited: %d", spiral_cpp_metrics_.multiple_pass_counter);
  ROS_INFO("Total accessible cells: %d", spiral_cpp_metrics_.accessible_counter);
  ROS_INFO("Total accessible area: %f", spiral_cpp_metrics_.total_area_covered);

  // TODO: Check if global path should be calculated repetitively or just kept
  // (also controlled by planner_frequency parameter in move_base namespace)

  ROS_INFO("Publishing plan!");
  for (const auto& poseStamped : plan) {
    std::cout << "PoseStamped - ";
    std::cout << "Position: (" << poseStamped.pose.position.x << ", "
              << poseStamped.pose.position.y << ", " << poseStamped.pose.position.z << ") ";
    std::cout << "Orientation: (" << poseStamped.pose.orientation.x << ", "
              << poseStamped.pose.orientation.y << ", " << poseStamped.pose.orientation.z
              << ", " << poseStamped.pose.orientation.w << ")\n";
}
  publishPlan(plan);
  ROS_INFO("Plan published!");
  ROS_DEBUG("Plan published");

  // Inorder to divide the path among agents, we need more points in between the corner points of the path.
    // We just perform a linear parametric up-sampling
    std::vector<PoseStamped> upSampled;
    ros::NodeHandle nh;

    ROS_INFO("Path computed. Up-sampling...");
    for (size_t i = 0; i < plan.size() - 1; ++i)
    {
        double mul = 0.5 * euDist2D(plan[i], plan[i + 1]) / floor(euDist2D(plan[i], plan[i + 1]) / robot_radius_);
        double a = 0;
        while (a < 1)
        {
            upSampled.push_back(parametricInterp1(plan[i], plan[i + 1], a));
            a += mul;
        }
        upSampled.push_back(plan[i + 1]);
    }
    ROS_INFO("Up-sampling complete");
    ROS_INFO("Waiting for number of agents");

    // Prepare publishers
    size_t nAgents = ros::topic::waitForMessage<std_msgs::UInt8>("number_of_agents")->data;
    std::vector<ros::Publisher> waypointPublishers;
    waypointPublishers.reserve(nAgents);
    for (auto i = 0; i < nAgents; ++i)
    {
        std::stringstream ss;
        ss << robotNamespace << "_" << i << "/waypoints";
        waypointPublishers.push_back(nh.advertise<nav_msgs::Path>(ss.str(), 100, true));
    }

    // Now divide the path approximately equally for each agent
    std::vector<nav_msgs::Path> agentPaths(nAgents);

    size_t length = upSampled.size() / nAgents;
    size_t leftover = upSampled.size() % nAgents;
    size_t begin = 0, end = 0;

    ROS_INFO("Publishing paths");
    for (size_t i = 0; i < std::min(nAgents, upSampled.size()); ++i)
    {
        end += leftover > 0 ? (length + !!(leftover--)) : length;

        agentPaths[i].header.frame_id = "map";
        agentPaths[i].header.stamp = ros::Time::now();
        agentPaths[i].poses.reserve(end - begin);

        for (size_t j = begin; j < end; ++j)
        {
            agentPaths[i].poses.push_back(upSampled[j]);
        }
        waypointPublishers[i].publish(agentPaths[i]);
        begin = end;
    }
    ROS_ERROR_STREAM(upSampled.size());
    size_t s = 0;
    for (size_t i = 0; i < nAgents; ++i)
    {
        s += agentPaths[i].poses.size();
    }
    ROS_ERROR_STREAM(s);

    // clock_t end_c = clock();
    // double elapsed_secs = static_cast<double>(end_c - begin_c) / CLOCKS_PER_SEC;
    // std::cout << "elapsed time: " << elapsed_secs << "\n";

    ros::spin();
    return true;
}
}  // namespace full_coverage_path_planner

int main(int argc, char** argv)
{

    ros::init(argc, argv, "SpiralSTC");
    // Create an instance of the SpiralSTC class
    full_coverage_path_planner::SpiralSTC mySpiralPlanner;

    // Initialize the planner (you might need to provide necessary parameters)
    mySpiralPlanner.initialize("SpiralSTC", nullptr);

    // Create dummy start and goal poses
    geometry_msgs::PoseStamped start, goal;

    // Call the makePlan function
    std::vector<geometry_msgs::PoseStamped> plan;
    mySpiralPlanner.makePlan(start, goal, plan);

    // Do something with the generated plan if needed

    return EXIT_SUCCESS;
}