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

const int NARROW_AREA_THRESHOLD = 3;

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::SpiralSTC, nav_core::BaseGlobalPlanner)

int pattern_dir_ = point;


namespace full_coverage_path_planner
{
void SpiralSTC::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

  if (!initialized_)
  {

    // Create a publisher to visualize the plan
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh, private_named_nh("~/" + name);

    plan_pub_ = private_named_nh.advertise<nav_msgs::Path>("plan", 1);

    // Try to request the cpp-grid from the cpp_grid map_server
    cpp_grid_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");

    // Get the cost map:
    costmap_ros_ = costmap_ros;
        // are we making a local copy(global variable) of the costmap here?-> Yes
    costmap_ = costmap_ros->getCostmap();
        // all with _ end are global variable...

    // Define  robot radius (radius) parameter
    float robot_radius_default = 0.5f;
    private_named_nh.param<float>("robot_radius", robot_radius_, robot_radius_default);
    // Define  tool radius (radius) parameter
    float tool_radius_default = 0.5f;
    private_named_nh.param<float>("tool_radius", tool_radius_, tool_radius_default);

    initialized_ = true;
  }
}


std::list<std::vector<Point_t>> SpiralSTC::explore_subregions(const std::vector<std::vector<bool>>& environment,
                                                               int explored_width, int explored_height,
                                                               int sub_area_count) {
    std::list<std::vector<Point_t>> sub_regions;

    // Calculate the number of columns and rows based on the sub_area_count
    int num_columns = std::sqrt(sub_area_count);
    int num_rows = sub_area_count / num_columns;

    // Calculate the width and height of each sub-region
    int sub_region_width = explored_width / num_columns;
    int sub_region_height = explored_height / num_rows;

    // Find the boundaries of the explored area
    int min_x = explored_width; // Set to maximum possible value
    int max_x = 0;
    int min_y = explored_height; // Set to maximum possible value
    int max_y = 0;

    // Iterate through the environment grid to find the boundaries of the explored area
    for (int y = 0; y < explored_height; ++y) {
        for (int x = 0; x < explored_width; ++x) {
            if (environment[y][x]) { // Explored area
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }
        }
    }

    // Calculate the starting point of the explored area
    int start_x = min_x;
    int start_y = min_y;

    // Iterate through the explored area and create sub-regions
    for (int row = 0; row < num_rows; ++row) {
        for (int col = 0; col < num_columns; ++col) {
            int sub_region_x = start_x + col * sub_region_width;
            int sub_region_y = start_y + row * sub_region_height;

            // Create a new sub-region with appropriate boundaries
            std::vector<Point_t> sub_region = {
                {sub_region_x, sub_region_y},  // Top-left corner
                {sub_region_x + sub_region_width - 1, sub_region_y},  // Top-right corner
                {sub_region_x + sub_region_width - 1, sub_region_y + sub_region_height - 1},  // Bottom-right corner
                {sub_region_x, sub_region_y + sub_region_height - 1}  // Bottom-left corner
            };
            sub_regions.push_back(sub_region);
        }
    }

    return sub_regions;
}

// boustrophedon (only the mountain pattern up and down)... stc(spanning key tree coverage----> reach deadend... still
// have area to cover (a-star)...->

std::list<gridNode_t> SpiralSTC::boustrophedon(std::vector<std::vector<bool>> const& grid,
                                                      std::list<gridNode_t>& init,
                                                      std::vector<std::vector<bool>>& visited)

///
// std::list<gridNode_t> BoustrophedonSTC::boustrophedon(std::vector<std::vector<bool> > const& grid,
// std::list<gridNode_t>& init, std::vector<std::vector<bool> >& visited, /*/GLOBAL VAR (VERT/HORIZONTAL)/*/)
{
    int dx, dy, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();
    // Mountain pattern filling of the open space
    // Copy incoming list to 'end'
    std::list<gridNode_t> pathNodes(init);
    // Set starting pos
    x2 = pathNodes.back().pos.x;
    y2 = pathNodes.back().pos.y;
    // set initial direction based on space visible from initial pos
    /*/  IF GLOBAL VAR = 0*/

    int robot_dir = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, point);
    /*  GLOBAL VAR++*/
    /*/   if global var ==1 /*/
    // int robot_dir =

    // set dx and dy based on robot_dir
    switch (robot_dir)
    {
    case east:  // 1
        dx = +1;
        dy = 0;
        break;
    case west:  // 2
        dx = -1;
        dy = 0;
        break;
    case north:  // 3
        dx = 0;
        dy = +1;
        break;
    case south:  // 4
        dx = 0;
        dy = -1;
        break;
    default:
        ROS_ERROR(
            "Full Coverage Path Planner: NO INITIAL ROBOT DIRECTION CALCULATED. "
            "This is a logic error that must be fixed by editing spiral_stc.cpp. Will travel east for now.");
        robot_dir = east;
        dx = +1;
        dy = 0;
        break;
    }

    bool done = false;
    while (!done)
    {
        // 1. drive straight until not a valid move (hit occupied cell or at end of map)

        bool hitWall = false;
        while (!hitWall)
        {
            x2 += dx;
            y2 += dy;
            if (!validMove(x2, y2, nCols, nRows, grid, visited))
            {
                hitWall = true;
                x2 = pathNodes.back().pos.x;
                y2 = pathNodes.back().pos.y;
                break;
            }
            if (!hitWall)
            {
                addNodeToList(x2, y2, pathNodes, visited);
            }
        }

        // 2. check left and right after hitting wall, then change direction
        if (robot_dir == north || robot_dir == south)
        {
            // if going north/south, then check if (now if it goes east/west is valid move, if it's not, then deadend)
            if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited) &&
                !validMove(x2 - 1, y2, nCols, nRows, grid, visited))
            {
                // dead end, exit
                done = true;
                break;
            }
            else if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited))
            {
                // east is occupied, travel towards west
                x2--;
                pattern_dir_ = west;
            }
            else if (!validMove(x2 - 1, y2, nCols, nRows, grid, visited))
            {
                // west is occupied, travel towards east
                x2++;
                pattern_dir_ = east;
            }
            else
            {
                // both sides are opened. If you don't have a preferred turn direction, travel towards most open
                // direction
                if (!(pattern_dir_ == east || pattern_dir_ == west))
                {
                    
                    if (validMove(x2, y2 + 1, nCols, nRows, grid, visited))
                    {
                        pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, north);
                    }
                    else
                    {
                        pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, south);
                    }
                    ROS_INFO("rotation dir with most space successful");
                }
                // Get into this following state-> (blocked or visited. valid move) preferred turn direction ***->
                // variable pattern direction***=> top if right here***-> pattern direction not East r West***-> ( if no
                // preferred turn direction---> travel to most open)
                if (pattern_dir_ = east)
                {
                    x2++;
                }
                else if (pattern_dir_ = west)
                {
                    x2--;
                }
            }

            // add Node to List
            addNodeToList(x2, y2, pathNodes, visited);

            // change direction 180 deg (this is when after hit wall, increment by 1 node, then head backwards... this
            // gets added to path list when the loop goes back up)
            if (robot_dir == north)
            {
                robot_dir = south;
                dy = -1;
            }

            else if (robot_dir == south)
            {
                robot_dir = north;
                dy = 1;
            }
        }
        else if (robot_dir == east || robot_dir == west)
        {
            if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited) &&
                !validMove(x2, y2 - 1, nCols, nRows, grid, visited))
            {
                // dead end, exit
                done = true;
                break;
            }
            else if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited))
            {
                // north is occupied, travel towards south
                y2--;
                pattern_dir_ = south;
            }
            else if (!validMove(x2, y2 - 1, nCols, nRows, grid, visited))
            {
                // south is occupied, travel towards north
                y2++;
                pattern_dir_ = north;
            }
            else
            {
                // both sides are opened. If don't have a prefered turn direction, travel towards farthest edge
                if (!(pattern_dir_ == north || pattern_dir_ == south))
                {
                    if (validMove(x2 + 1, y2, nCols, nRows, grid, visited))
                    {
                        pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, east);
                    }
                    else
                    {
                        pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, west);
                    }
                }
                if (pattern_dir_ = north)
                {
                    y2++;
                }
                else if (pattern_dir_ = south)
                {
                    y2--;
                }
            }

            // add Node to List
            addNodeToList(x2, y2, pathNodes, visited);

            // change direction 180 deg
            if (robot_dir == east)
            {
                robot_dir = west;
                dx = -1;
            }
            else if (robot_dir == west)
            {
                robot_dir = east;
                dx = 1;
            }
        }
    }
    // Log
    // printPathNodes(pathNodes);
    return pathNodes;
}


std::list<Point_t> SpiralSTC::boustrophedon_stc(std::vector<std::vector<bool>> const& grid, Point_t& init,
                                                       int& multiple_pass_counter, int& visited_counter)
{
    // what's multiple pass counter->  while update visited (output statistical....->  log )

    int x, y, nRows = grid.size(), nCols = grid[0].size();
    pattern_dir_ = point;
    // initialize something no direction associated with it yet***->  (Default)
    // Initial node is initially set as visited so it does not count
    multiple_pass_counter = 0;
    visited_counter = 0;

    std::vector<std::vector<bool>> visited;
    visited = grid;  // Copy grid matrix
    x = init.x;
    y = init.y;

    // add initial point to pathNodes
    std::list<gridNode_t> pathNodes;
    std::list<Point_t> fullPath;

    addNodeToList(x, y, pathNodes, visited);

    std::list<Point_t> goals =
        map_2_goals(visited, eNodeOpen);  // Retrieve all goalpoints (Cells not visited)--- all open cells
    ///////////
    std::cout << "Goals Left: " << goals.size() << std::endl;
    // how many goals to start with???(all cells not visited?)
    std::list<gridNode_t>::iterator it;

#ifdef DEBUG_PLOT
    ROS_INFO("Grid before walking is: ");
    printGrid(grid, visited, fullPath);
#endif

    while (goals.size() != 0)
    {
        // boustrophedon pattern from current position
        // goal ****-
        pathNodes = boustrophedon(grid, pathNodes, visited);

        // std::cout << "path nodes: ";
        // for (const auto& node : pathNodes) {
        //   // Print specific information from gridNode_t here (e.g., coordinates, value)
        //   std::cout << "(" << node.pos.x << ", " << node.pos.y << "), ";
        // }
        // std::cout <<"---------------------------------------------------------"<< std::endl;
        // return a list of points***
        
#ifdef DEBUG_PLOT
        ROS_INFO("Visited grid updated after boustrophedon:");
        printGrid(grid, visited, pathNodes, PatternStart, pathNodes.back());
#endif
        ///////
        for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
        {
            Point_t newPoint = {it->pos.x, it->pos.y};
            // ?????is this a pointer or another operation? (Above).
            visited_counter++;
            fullPath.push_back(newPoint);
            // what is fullpath pushback again?-> push all the points in to the path
        }

        ////////////////////////////////// where is it marking all boustrphedon is visited?

        goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints
        // ????????? why except last element? Remove all elements from pathNodes list except last element.
        // The last point is the starting point for a new search and A* extends the path from there on

        //????? what are we doing here again? why are we erasing the elements???-> is it to start the new path?

        pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
        visited_counter--;  // First point is already counted as visited
        // Plan to closest open Node using A*
        // Pathnodes.back(is starting point)`goals` is essentially the map, so we use `goals` to determine the distance
        // from the end of a potential path
        //    to the nearest free space
        bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
        if (resign)
        {
            ROS_WARN(
                "A_star_to_open_space is resigning! This may be due to the open cells outside of the "
                "obstacle boundary. Goals Left: %lu",
                goals.size());
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
        gridNode_t PatternStart = pathNodes.back();
        printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());
#endif
    }
    return fullPath;
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


// std::list<gridNode_t> SpiralSTC::new_spiral(std::vector<std::vector<bool> > const& grid, std::list<gridNode_t>& init,
//                                         std::vector<std::vector<bool> >& visited,
//                                         std::vector<std::vector<bool>> &narrow_area_grid_points)
// {
//   int dx, dy, dx_prev, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();

//   // Spiral filling of the open space
//   // Copy incoming list to 'end'
//   std::list<gridNode_t> pathNodes(init);

//   // Create iterator for gridNode_t list and let it point to the last element of end
//   std::list<gridNode_t>::iterator it = --(pathNodes.end());

//   if (pathNodes.size() > 1)  // if list is length 1, keep iterator at end
//     it--;                    // Let iterator point to second to last element

//   gridNode_t prev = *(it);
  
//   bool done = false;
//   while (!done)
//   {
//     if (it != pathNodes.begin())
//     {
//       // turn ccw
//       dx = pathNodes.back().pos.x - prev.pos.x;
//       dy = pathNodes.back().pos.y - prev.pos.y;
//       dx_prev = dx;
//       dx = -dy;
//       dy = dx_prev;
//     }
//     else
//     {
//       // Initialize spiral direction towards y-axis
//       dx = 0;
//       dy = 1;
//     }
//     done = true;

//     for (int i = 0; i < 4; ++i)
//     {
//       x2 = pathNodes.back().pos.x + dx;
//       y2 = pathNodes.back().pos.y + dy;
//       if (x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows)
//       {
//         if (grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen && narrow_area_grid_points[x2][y2])
//         {
//           Point_t new_point = { x2, y2 };
//           gridNode_t new_node =
//           {
//             new_point,  // Point: x,y
//             0,          // Cost
//             0,          // Heuristic
//           };
//           prev = pathNodes.back();
//           pathNodes.push_back(new_node);
//           it = --(pathNodes.end());
//           visited[y2][x2] = eNodeVisited;  // Close node
//           done = false;
//           break;

//           dx_prev = dx;
//           dx = dy;
//           dy = -dx_prev;
//         }
//         else if(grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen && !narrow_area_grid_points[x2][y2])
//         {
//           bool b_done = false;
//           while (!b_done)
//           {

//             bool hitWall = false;
//             while (!hitWall)
//             {
//                 if(narrow_area_grid_points[y2-dy][x2-dx]){
//                   x2 += dx;
//                   y2 += dy;
//                 }
                
//                 if (!validMoveInNarrow(x2, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
//                 {
//                     hitWall = true;
//                     // x2 = pathNodes.back().pos.x;
//                     // y2 = pathNodes.back().pos.y;
//                     break;
//                 }
//                 if (!hitWall)
//                 {
//                     Point_t new_point = { x2, y2 };
//                     gridNode_t new_node =
//                     {
//                       new_point,  // Point: x,y
//                       0,          // Cost
//                       0,          // Heuristic
//                     };
//                     prev = pathNodes.back();
//                     pathNodes.push_back(new_node);
//                     it = --(pathNodes.end());
//                     visited[y2][x2] = eNodeVisited;  // Close node
//                 }
//             }

//             if(!narrow_area_grid_points[x2][y2]){

//                 // check left and right after hitting wall, then change direction
//               if ((dx == 0 && dy == +1) || (dx == 0 && dy == -1)){

//                  x2 = pathNodes.back().pos.x;
//                  y2 = pathNodes.back().pos.y;

//                 if (!validMoveInNarrow(x2 + 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points) &&
//                     !validMoveInNarrow(x2 - 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
//                 {
//                     // dead end, exit
//                     done = true;
//                     break;
//                 }
//                 else if (!validMoveInNarrow(x2 + 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
//                 {
//                     // east is occupied, travel towards west
//                     x2--;
//                     pattern_dir_ = west;
//                 }
//                 else if (!validMoveInNarrow(x2 - 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
//                 {
//                     // west is occupied, travel towards east
//                     x2++;
//                     pattern_dir_ = east;
//                 }
//                   else
//                   {
//                       // both sides are opened. If you don't have a preferred turn direction, travel towards most open
//                       // direction
//                       if (!(pattern_dir_ == east || pattern_dir_ == west))
//                       {
//                           if(!narrow_area_grid_points[y2+1][x2]){
//                             pattern_dir_ = east;
//                           }
//                           else{
//                             pattern_dir_ = west;
//                           }
//                       }

//                       if (pattern_dir_ = east)
//                       {
//                           x2++;
//                       }
//                       else if (pattern_dir_ = west)
//                       {
//                           x2--;
//                       }
//                   }
//                   Point_t new_point = { x2, y2 };
//                     gridNode_t new_node =
//                     {
//                       new_point,  // Point: x,y
//                       0,          // Cost
//                       0,          // Heuristic
//                     };
//                     prev = pathNodes.back();
//                     pathNodes.push_back(new_node);
//                     it = --(pathNodes.end());
//                     visited[y2][x2] = eNodeVisited;  // Close node

//                     if (dx == 0 && dy == +1)
//                     {
//                         dy = -1;
//                     }
//                     else if (dx == 0 && dy == -1)
//                     {
//                         dy = 1;
//                     }
//                 }

//               else if((dx == +1 && dy == 0) || (dx == -1 && dy == 0))
//                 {

//                   x2 = pathNodes.back().pos.x;
//                   y2 = pathNodes.back().pos.y;

//                   if (!validMoveInNarrow(x2, y2+1, nCols, nRows, grid, visited,narrow_area_grid_points) &&
//                       !validMoveInNarrow(x2, y2-1, nCols, nRows, grid, visited,narrow_area_grid_points))
//                   {
//                       // dead end, exit
//                       done = true;
//                       break;
//                   }
//                   else if (!validMoveInNarrow(x2, y2+1, nCols, nRows, grid, visited,narrow_area_grid_points))
//                   {
//                       // east is occupied, travel towards west
//                       y2--;
//                       pattern_dir_ = south;
//                   }
//                   else if (!validMoveInNarrow(x2, y2-1, nCols, nRows, grid, visited,narrow_area_grid_points))
//                   {
//                       // west is occupied, travel towards east
//                       y2++;
//                       pattern_dir_ = north;
//                   }
//                     else
//                     {
//                         // both sides are opened. If you don't have a preferred turn direction, travel towards most open
//                         // direction
//                         if (!(pattern_dir_ == north || pattern_dir_ == south))
//                         {
//                             if(!narrow_area_grid_points[y2][x2+1]){
//                               pattern_dir_ = north;
//                             }
//                             else{
//                               pattern_dir_ = south;
//                             }
//                         }

//                         if (pattern_dir_ = north)
//                         {
//                             y2++;
//                         }
//                         else if (pattern_dir_ = south)
//                         {
//                             y2--;
//                         }
//                     }
//                     Point_t new_point = { x2, y2 };
//                       gridNode_t new_node =
//                       {
//                         new_point,  // Point: x,y
//                         0,          // Cost
//                         0,          // Heuristic
//                       };
//                       prev = pathNodes.back();
//                       pathNodes.push_back(new_node);
//                       it = --(pathNodes.end());
//                       visited[y2][x2] = eNodeVisited;  // Close node

//                       if (dx == +1 && dy == 0)
//                       {
//                           dx = -1;
//                       }
//                       else if (dx == -1 && dy == 0)
//                       {
//                           dx = 1;
//                       }

//                 }
//             }
//             else{
//               dx_prev = dx;
//               dx = dy;
//               dy = -dx_prev;
//               b_done = true;
//               break;
//             }
//           }
//         }

//         }
//       }
//       // try next direction cw
//       // dx_prev = dx;
//       // dx = dy;
//       // dy = -dx_prev;
//     }
//     return pathNodes;
//   }


std::list<gridNode_t> SpiralSTC::new_spiral(std::vector<std::vector<bool> > const& grid, std::list<gridNode_t>& init,
                                        std::vector<std::vector<bool> >& visited,
                                        std::vector<std::vector<bool>> const& narrow_area_grid_points)
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

          if(narrow_area_grid_points[y2][x2] == 1){

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
          else if(narrow_area_grid_points[y2][x2] == 0){

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

            bool b_n_f = false;
            while(!b_n_f)
            {
                bool hitWall = false;
                while (!hitWall)
                {
                    x2 += dx;
                    y2 += dy;
                    if (!validMoveInNarrow(x2, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
                    {
                        hitWall = true;
                        prev = pathNodes.back();
                        x2 = pathNodes.back().pos.x;
                        y2 = pathNodes.back().pos.y;
                        break;
                    }
                    if (!hitWall)
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
                    }
                }

                if((dx == 0 && dy == 1) || (dx == 0 && dy == -1)) // dir is down or up
                {
                  if (!validMoveInNarrow(x2 + 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points) &&
                      !validMoveInNarrow(x2 - 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
                  {
                      // dead end, exit
                      b_n_f = true;
                      break;
                  }
                  else if (!validMoveInNarrow(x2 + 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
                  {
                      // east is occupied, travel towards west
                      x2--;
                      pattern_dir_ = west;
                  }
                  else if (!validMoveInNarrow(x2 - 1, y2, nCols, nRows, grid, visited,narrow_area_grid_points))
                  {
                      // west is occupied, travel towards east
                      x2++;
                      pattern_dir_ = east;
                  }
                  else
                  {

                    if (!(pattern_dir_ == east || pattern_dir_ == west)){
                          b_n_f = true;
                          break;
                    }

                    if (pattern_dir_ = east)
                    {
                        x2++;
                    }
                    else if (pattern_dir_ = west)
                    {
                        x2--;
                    }
                    
                  }

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
                  visited[y2][x2] = eNodeVisited;

                  if (dx == 0 && dy == 1)
                  {
                      dy = -1;
                  }

                  else if (dx == 0 && dy == -1)
                  {
                      dy = 1;
                  }

                }

                else if((dx == 1 && dy == 0) || (dx == -1 && dy == 0)) // dir is right or left
                {
                  if (!validMoveInNarrow(x2, y2 + 1, nCols, nRows, grid, visited,narrow_area_grid_points) &&
                      !validMoveInNarrow(x2, y2 - 1, nCols, nRows, grid, visited,narrow_area_grid_points))
                  {
                      // dead end, exit
                      b_n_f = true;
                      break;
                  }
                  else if (!validMoveInNarrow(x2, y2 + 1, nCols, nRows, grid, visited,narrow_area_grid_points))
                  {
                      // east is occupied, travel towards west
                      y2--;
                      pattern_dir_ = north;
                  }
                  else if (!validMoveInNarrow(x2, y2 - 1, nCols, nRows, grid, visited,narrow_area_grid_points))
                  {
                      // west is occupied, travel towards east
                      y2++;
                      pattern_dir_ = south;
                  }
                  else
                  {

                    if (!(pattern_dir_ == north || pattern_dir_ == south)){
                          b_n_f = true;
                          break;
                    }

                    if (pattern_dir_ = south)
                    {
                        y2++;
                    }
                    else if (pattern_dir_ = north)
                    {
                        y2--;
                    }
                    
                  }

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
                  visited[y2][x2] = eNodeVisited;

                  if(dx == 1 && dy == 0){
                      dx = -1;
                  }
                  else if (dx == -1 && dy == 0)
                  {
                      dx = 1;
                  }

                }
            }

          }
          
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


std::list<Point_t> SpiralSTC::new_spiral_stc(std::vector<std::vector<bool> > const& grid,
                                          Point_t& init,
                                          int &multiple_pass_counter,
                                          int &visited_counter,
                                          std::vector<std::vector<bool>> const& narrow_area_grid_points)
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

  pathNodes = SpiralSTC::new_spiral(grid, pathNodes, visited,narrow_area_grid_points);                // First spiral fill
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
    pathNodes = new_spiral(grid, pathNodes, visited,narrow_area_grid_points);

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

  // clear the plan, just in case
  plan.clear();
  costmap_ = costmap_ros_->getCostmap();
  // is the costmap_ros_ a global variable ?
  // this is updated cost map??

  clock_t begin_c = clock();
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
  publishPlan(plan);
  ROS_INFO("Plan published!");
  ROS_DEBUG("Plan published");

    clock_t end_c = clock();
    double elapsed_secs = static_cast<double>(end_c - begin_c) / CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_secs << "\n";

    return true;
}
}  // namespace full_coverage_path_planner
