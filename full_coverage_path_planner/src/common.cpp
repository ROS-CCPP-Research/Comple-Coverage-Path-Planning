#include <algorithm>
#include <iostream>
#include <limits>
#include <list>
#include <vector>
#include <algorithm> 
#include <full_coverage_path_planner/common.h>

std::vector<std::pair<int, int>> bfs_directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};


int distanceToClosestPoint(Point_t poi, std::list<Point_t> const& goals)
{
  // Return minimum distance from goals-list
  int min_dist = INT_MAX;
  std::list<Point_t>::const_iterator it;
  for (it = goals.begin(); it != goals.end(); ++it)
  {
    int cur_dist = distanceSquared((*it), poi);
    if (cur_dist < min_dist)
    {
      min_dist = cur_dist;
    }
  }
  return min_dist;
}

int distanceSquared(const Point_t& p1, const Point_t& p2)
{
  int dx = p2.x - p1.x;
  int dy = p2.y - p1.y;

  int dx2 = dx * dx;
  if (dx2 != 0 && dx2 / dx != dx)
  {
    throw std::range_error("Integer overflow error for the given points");
  }

  int dy2 = dy * dy;
  if (dy2 != 0 && dy2 / dy != dy)
  {
    throw std::range_error("Integer overflow error for the given points");
  }

  if (dx2 > std::numeric_limits< int >::max() - dy2)
    throw std::range_error("Integer overflow error for the given points");
  int d2 = dx2 + dy2;

  return d2;
}

/**
 * Sort vector<gridNode> by the heuristic value of the last element
 * @return whether last elem. of first has a larger heuristic value than last elem of second
 */
bool sort_gridNodePath_heuristic_desc(const std::vector<gridNode_t> &first, const std::vector<gridNode_t> &second)
{
  return (first.back().he > second.back().he);
}

bool a_star_to_open_space(std::vector<std::vector<bool> > const &grid, gridNode_t init, int cost,
                          std::vector<std::vector<bool> > &visited, std::list<Point_t> const &open_space,
                          std::list<gridNode_t> &pathNodes)
{
  uint dx, dy, dx_prev, nRows = grid.size(), nCols = grid[0].size();

  std::vector<std::vector<bool> > closed(nRows, std::vector<bool>(nCols, eNodeOpen));
  // All nodes in the closest list are currently still open

  closed[init.pos.y][init.pos.x] = eNodeVisited;  // Of course we have visited the current/initial location
#ifdef DEBUG_PLOT
  std::cout << "A*: Marked init " << init << " as eNodeVisited (true)" << std::endl;
  printGrid(closed);
#endif

  std::vector<std::vector<gridNode_t> > open1(1, std::vector<gridNode_t>(1, init));  // open1 is a *vector* of paths

  while (true)
  {
#ifdef DEBUG_PLOT
    std::cout << "A*: open1.size() = " << open1.size() << std::endl;
#endif
    if (open1.size() == 0)  // If there are no open paths, there's no place to go and we must resign
    {
      // Empty end_node list and add init as only element
      pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
      pathNodes.push_back(init);
      return true;  // We resign, cannot find a path
    }
    else
    {
      // Sort elements from high to low (because sort_gridNodePath_heuristic_desc uses a > b)
      std::sort(open1.begin(), open1.end(), sort_gridNodePath_heuristic_desc);

      std::vector<gridNode_t> nn = open1.back();  // Get the *path* with the lowest heuristic cost
      open1.pop_back();  // The last element is no longer open because we use it here, so remove from open list
#ifdef DEBUG_PLOT
      std::cout << "A*: Check out path from" << nn.front().pos << " to " << nn.back().pos
      << " of length " << nn.size() << std::endl;
#endif

      // Does the path nn end in open space?
      if (visited[nn.back().pos.y][nn.back().pos.x] == eNodeOpen)
      {
        // If so, we found a path to open space
        // Copy the path nn to pathNodes so we can report that path (to get to open space)
        std::vector<gridNode_t>::iterator iter;
        for (iter = nn.begin(); iter != nn.end(); ++iter)
        {
          pathNodes.push_back((*iter));
        }

        return false;  // We do not resign, we found a path
      }
      else  // Path nn does not lead to open space
      {
        if (nn.size() > 1)
        {
          // Create iterator for gridNode_t list and let it point to the last element of nn
          std::vector<gridNode_t>::iterator it = --(nn.end());
          dx = it->pos.x - (it - 1)->pos.x;
          dy = it->pos.y - (it - 1)->pos.y;
          // TODO docs: this seems to cycle through directions
          // (notice the shift-by-one between both sides of the =)
          dx_prev = dx;
          dx = -dy;
          dy = dx_prev;
        }
        else
        {
          dx = 0;
          dy = 1;
        }

        // For all nodes surrounding the end of the end of the path nn
        for (uint i = 0; i < 4; ++i)
        {
          Point_t p2 =
          {
            static_cast<int>(nn.back().pos.x + dx),
            static_cast<int>(nn.back().pos.y + dy),
          };

#ifdef DEBUG_PLOT
          std::cout << "A*: Look around " << i << " at p2=(" << p2 << std::endl;
#endif

          if (p2.x >= 0 && p2.x < nCols && p2.y >= 0 && p2.y < nRows)  // Bounds check, do not sep out of map
          {
            // If the new node (a neighbor of the end of the path nn) is open, append it to newPath ( = nn)
            // and add that to the open1-list of paths.
            // Because of the pop_back on open1, what happens is that the path is temporarily 'checked out',
            // modified here, and then added back (if the condition above and below holds)
            if (closed[p2.y][p2.x] == eNodeOpen && grid[p2.y][p2.x] == eNodeOpen)
            {
#ifdef DEBUG_PLOT
              std::cout << "A*: p2=" << p2 << " is OPEN" << std::endl;
#endif
              std::vector<gridNode_t> newPath = nn;
              // # heuristic  has to be designed to prefer a CCW turn
              Point_t new_point = { p2.x, p2.y };
              gridNode_t new_node =
              {
                new_point,                                                                             // Point: x,y
                static_cast<int>(cost + nn.back().cost),                                               // Cost
                static_cast<int>(cost + nn.back().cost + distanceToClosestPoint(p2, open_space) + i),
                // Heuristic (+i so CCW turns are cheaper)
              };
              newPath.push_back(new_node);
              closed[new_node.pos.y][new_node.pos.x] = eNodeVisited;  // New node is now used in a path and thus visited

#ifdef DEBUG_PLOT
              std::cout << "A*: Marked new_node " << new_node << " as eNodeVisited (true)" << std::endl;
              std::cout << "A*: Add path from " << newPath.front().pos << " to " << newPath.back().pos
              << " of length " << newPath.size() << " to open1" << std::endl;
#endif
              open1.push_back(newPath);
            }
#ifdef DEBUG_PLOT
            else
            {
              std::cout << "A*: p2=" << p2 << " is not open: "
                        "closed[" << p2.y << "][" << p2.x << "]=" << closed[p2.y][p2.x] << ", "
                        "grid["  << p2.y << "][" << p2.x << "]=" << grid[p2.y][p2.x] << std::endl;
            }
#endif
          }
#ifdef DEBUG_PLOT
          else
          {
            std::cout << "A*: p2=(" << p2.x << ", " << p2.y << ") is out of bounds" << std::endl;
          }
#endif
          // Cycle around to next neighbor, CCW
          dx_prev = dx;
          dx = dy;
          dy = -dx_prev;
        }
      }
    }
  }
}

void printGrid(std::vector<std::vector<bool> > const& grid, std::vector<std::vector<bool> > const& visited,
               std::list<Point_t> const& path)
{
  for (uint iy = grid.size() - 1; iy >= 0; --iy)
  {
    for (uint ix = 0; ix < grid[0].size(); ++ix)
    {
      if (visited[iy][ix])
      {
        if (ix == path.front().x && iy == path.front().y)
        {
          std::cout << "\033[1;32m▓\033[0m";  // Show starting position in green color
        }
        else if (ix == path.back().x && iy == path.back().y)
        {
          std::cout << "\033[1;31m▓\033[0m";  // Show stopping position in red color
        }
        else if (visited[iy][ix] && grid[iy][ix])
        {
          std::cout << "\033[1;33m▓\033[0m";  // Show walls in yellow color
        }
        else
        {
          std::cout << "\033[1;36m▓\033[0m";
        }
      }
      else
      {
        std::cout << "\033[1;37m▓\033[0m";
      }
    }
    std::cout << "\n";
  }
}

void printGrid(std::vector<std::vector<bool> > const& grid, std::vector<std::vector<bool> > const& visited,
               std::list<gridNode_t> const& path, gridNode_t start, gridNode_t end)
{
  for (uint iy = grid.size() - 1; iy >= 0; --iy)
  {
    for (uint ix = 0; ix < grid[0].size(); ++ix)
    {
      if (visited[iy][ix])
      {
        if (ix == start.pos.x && iy == start.pos.y)
        {
          std::cout << "\033[1;32m▓\033[0m";  // Show starting position in green color
        }
        else if (ix == end.pos.x && iy == end.pos.y)
        {
          std::cout << "\033[1;31m▓\033[0m";  // Show stopping position in red color
        }
        else if (visited[iy][ix] && grid[iy][ix])
        {
          std::cout << "\033[1;33m▓\033[0m";  // Show walls in yellow color
        }
        else
        {
          std::cout << "\033[1;36m▓\033[0m";
        }
      }
      else
      {
        std::cout << "\033[1;37m▓\033[0m";
      }
    }
    std::cout << "\n";
  }
}

void printGrid(std::vector<std::vector<bool> > const& grid)
{
  for (uint iy = grid.size() - 1; iy >= 0; --iy)
  {
    for (uint ix = 0; ix < grid[0].size(); ++ix)
    {
      if (grid[iy][ix])
      {
        std::cout << "\033[1;36m▓\033[0m";
      }
      else
      {
        std::cout << "\033[1;37m▓\033[0m";
      }
    }
    std::cout << "\n";
  }
}

void printGridBinary(std::vector<std::vector<bool> > const& grid)
{
  for (const auto& row : grid) {
        for (bool cell : row) {
            // Print '1' for true and '0' for false
            std::cout << (cell ? "1 " : "0 ");
        }
        std::cout << std::endl;
    }
}

std::list<Point_t> map_2_goals(std::vector<std::vector<bool> > const& grid, bool value_to_search)
{
  std::list<Point_t> goals;
  int ix, iy;
  uint nRows = grid.size();
  uint nCols = grid[0].size();
  for (iy = 0; iy < nRows; ++(iy))
  {
    for (ix = 0; ix < nCols; ++(ix))
    {
      if (grid[iy][ix] == value_to_search)
      {
        Point_t p = { ix, iy };  // x, y
        goals.push_back(p);
      }
    }
  }
  return goals;
}

bool validMove(int x2, int y2, int nCols, int nRows, std::vector<std::vector<bool>> const& grid,
               std::vector<std::vector<bool>> const& visited)
{
    return (x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows)                 // path node is within the map
           && (grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen);  // the path node is unvisited
    // ??????????? meaning, not visited, and no obstacles.
}

void addNodeToList(int x2, int y2, std::list<gridNode_t>& pathNodes, std::vector<std::vector<bool>>& visited)
{
    Point_t new_point = {x2, y2};
    // clang-format off
    gridNode_t new_node = {  // NOLINT
        new_point,  // Point: x,y
        0,          // Cost
        0,          // Heuristic
    };
    // clang-format on
    pathNodes.push_back(
        new_node);  // turn point into gridnode and pushback in to path node to add new node!! ** add make it visited
    visited[y2][x2] = eNodeVisited;  // Close node
    return;
}

int dirWithMostSpace(int x_init, int y_init, int nCols, int nRows, std::vector<std::vector<bool>> const& grid,
                     std::vector<std::vector<bool>> const& visited, int ignoreDir)
{
    // this array stores how far the robot can travel in a straight line for each direction
    int free_space_in_dir[5] = {0};
    // for each direction
    for (int i = 1; i < 5; i++)
    {
        // start from starting pos
        int x2 = x_init;
        int y2 = y_init;
        do
        {  // loop until hit wall
            switch (i)
            {
            case east:
                x2++;
                break;
            case west:
                x2--;
                break;
            case north:
                y2++;
                break;
            case south:
                y2--;
                break;
            default:
                break;
            }
            free_space_in_dir[i]++;
            // counter for space
        } while (validMove(x2, y2, nCols, nRows, grid, visited));  // NOLINT
    }

    //????????? use the biggest value***->
    // set initial direction towards direction with most travel possible

    int indexValue = 0;
    int robot_dir = 1;
    for (int i = 1; i <= 4; i++)
    {
        // std::cout << "free space in " << i << ": " << free_space_in_dir[i] << std::endl;
        if (free_space_in_dir[i] > indexValue && i != ignoreDir)
        {
            robot_dir = i;
            indexValue = free_space_in_dir[i];
        }
    }
    return robot_dir;
}



void getExploredAreaDimensions(const std::vector<std::vector<bool>>& environment,
                               int& explored_height, int& explored_width) {
    int min_x = environment[0].size(); // Set to maximum possible value
    int max_x = 0;
    int min_y = environment.size();    // Set to maximum possible value
    int max_y = 0;

    // Iterate through the environment grid to find the boundaries of the explored area
    for (int y = 0; y < environment.size(); ++y) {
        for (int x = 0; x < environment[0].size(); ++x) {
            if (environment[y][x]) { // Explored area
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }
        }
    }

    // Calculate dimensions of the explored area
    explored_height = max_y - min_y + 1;
    explored_width = max_x - min_x + 1;
}

void bfs(int x, int y,int sub_nRows, int sub_nCols,
          std::vector<std::vector<bool>> const& sub_grid, 
          std::vector<std::vector<bool>>& visited,
          std::vector<std::vector<Node*>>& graph,
          Node* & root) {


    std::queue<std::pair<int, int>> q;
    q.push({x, y});
    visited[x][y] = true;

    Node* startNode = new Node(x, y);
    graph[x][y] = startNode;
    root = startNode;

    while (!q.empty()) {
        std::pair<int, int> curr = q.front();
        q.pop();
        int currX = curr.first;
        int currY = curr.second;

        // Process current node
        // std::cout << "(" << currX << ", " << currY << "),";

        // Explore neighbors
        int index = 0;
        for (const auto& dir : bfs_directions) {
            int newX = currX + dir.first;
            int newY = currY + dir.second;
            if (newX >= 0 && newX < sub_nRows && newY >= 0 && newY < sub_nCols &&
                sub_grid[newX][newY] == 0 && !visited[newX][newY]) {
                q.push({newX, newY});
                visited[newX][newY] = true;

                Node* newNode = new Node(newX, newY);
                graph[newX][newY] = newNode;

                // Connect the new node to the current node
                graph[currX][currY]->neighbors.push_back(newNode);

                if(index==0){
                  root->up = newNode;
                  newNode->down = root;
                }
                else if(index == 1){
                  root->left = newNode;
                  newNode->up = root;

                }
                else if(index == 2){
                  root->back = newNode;
                }
                else{
                  root->right = newNode;
                  newNode->back = root;
                }

                if (newX > currX && newY == currY) {
                    graph[currX][currY]->left = newNode;
                } else if (newX == currX && newY > currY) {
                    graph[currX][currY]->right = newNode;
                }
            }
        }
    }
}


void visualizeGraph(const std::vector<std::vector<Node*>>& graph) {
    std::ofstream dotFile("graph.dot");
    dotFile << "graph {\n";
    std::unordered_set<Node*> visited;

    for (const auto& row : graph) {
        for (const auto& node : row) {
            if (node && visited.find(node) == visited.end()) {
                visited.insert(node);
                dotFile << "  \"" << node->x << "_" << node->y << "\";\n"; // Enclose node name in double quotes
                for (const auto& neighbor : node->neighbors) {
                    dotFile << "  \"" << node->x << "_" << node->y << "\" -- \"" << neighbor->x << "_" << neighbor->y << "\";\n"; // Enclose node names in double quotes
                }
            }
        }
    }

    dotFile << "}\n";
    dotFile.close();

    // Convert dot file to image using Graphviz (dot) command
    system("dot -Tpng -O graph.dot");
}


void printGraph(const std::vector<std::vector<Node*>>& graph,int sub_nRows, int sub_nCols, std::vector<std::vector<bool>> const& sub_grid) {
    // Print the nodes and connections in the graph
    std::cout<<"print graph"<<std::endl;
    for (int i = 0; i < sub_nRows; ++i) {
        for (int j = 0; j < sub_nCols; ++j) {
            if (sub_grid[i][j]) {
                std::cout << "(" << i << "," << j << "): ";
                for (const auto& neighbor : graph[i][j]->neighbors) {
                    std::cout << "(" << neighbor->x << "," << neighbor->y << ") ";
                }
                std::cout << std::endl;
            }
        }
    }
}


// Function to explore the free area starting from a given point
void explore_free_area(std::vector<std::vector<bool>>& matrix, int start_x, int start_y, std::vector<std::vector<bool>>& visited, std::vector<Point_t>& boundary) {
    int rows = matrix.size();
    int cols = matrix[0].size();
    
    // Define the directions to explore: up, down, left, right
    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    
    // BFS exploration starting from the given point
    std::queue<Point_t> q;
    q.push({start_x, start_y});
    visited[start_x][start_y] = true;
    
    while (!q.empty()) {
        Point_t current = q.front();
        q.pop();
        boundary.push_back(current);
        
        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && matrix[nx][ny] == 0 && !visited[nx][ny]) {
                q.push({nx, ny});
                visited[nx][ny] = true;
            }
        }
    }
    std::sort(boundary.begin(), boundary.end(), comparePoints);
}

// Function to print the matrix based on the boundary list
void print_matrix(std::vector<std::vector<bool>>& matrix, std::vector<Point_t>& boundary) {
    int rows = matrix.size();
    int cols = matrix[0].size();
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            bool inside_boundary = false;
            for (const auto& point : boundary) {
                if (point.y == i && point.x== j) {
                    inside_boundary = true;
                    break;
                }
            }
            std::cout << (inside_boundary ? '0' : '1') << " ";
        }
        std::cout << std::endl;
    }
}

// Function to partition the free area into smaller regions
std::list<std::vector<Point_t>> partition_free_area(std::list<Point_t>& boundary, int partition_count) {
    std::list<std::vector<Point_t>> partitions;
    std::vector<Point_t> sub;

    // for (const auto part : partitions){
    //   sub.push_back(part.)
    // }
    


    return partitions;
}

void create_explored_grid(std::vector<std::vector<bool>> matrix, std::vector<Point_t> boundary,std::vector<std::vector<bool>>& explored_free_area_grid) {
    int rows = matrix.size();
    int cols = matrix[0].size();
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            bool inside_boundary = false;
            for (const auto& point : boundary) {
                if (point.x == i && point.y == j) {
                    inside_boundary = true;
                    break;
                }
            }
            if(inside_boundary){
              explored_free_area_grid[i][j] = false;
            }
            else{
              explored_free_area_grid[i][j] = true;
            }
        
        }
    }
}

void create_explored_grid_node(std::vector<std::vector<bool>> matrix, std::vector<Node*> boundary,std::vector<std::vector<bool>>& explored_free_area_grid) {
    int rows = matrix.size();
    int cols = matrix[0].size();
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            bool inside_boundary = false;
            for (const auto& point : boundary) {
                if (point->x == i && point->y == j) {
                    inside_boundary = true;
                    break;
                }
            }
            if(inside_boundary){
              explored_free_area_grid[i][j] = false;
            }
            else{
              explored_free_area_grid[i][j] = true;
            }
        
        }
    }
}

void preOrder(  Node* root, 
                int depth, 
                Node*& currentStart, 
                std::vector<Node*>& current_root, 
                std::vector<std::vector<Node*>>& narrow_area_points,
                std::vector<std::vector<bool>>& narrow_area_grid_points,
                std::vector<Node*>& temp_root
            ){

        if (root == nullptr) {
                return;
        }
        for (int i = 0; i < depth; ++i) {
        std::cout << "  ";
        }

        preOrder(root->left,depth+1,currentStart,current_root,narrow_area_points,narrow_area_grid_points,temp_root);

        preOrder(root->right,depth+1,currentStart,current_root,narrow_area_points,narrow_area_grid_points,temp_root);

        if(currentStart == nullptr){
            currentStart = root;
        }

        if(root->left != nullptr){

            current_root.push_back(root);

            if(current_root.size() % 2 ==1 || current_root.size() <= 2){

                if (!narrow_area_points.empty() &&
                    (current_root.front()->y == narrow_area_points.back().front()->y ||
                    current_root.back()->y == narrow_area_points.back().back()->y )
                ) {

                        narrow_area_points.back().insert(narrow_area_points.back().end(),
                                      current_root.begin(), current_root.end());
                }
                else{
                    narrow_area_points.push_back(current_root);
                }

                for(const auto point : current_root){
                    narrow_area_grid_points[point->x][point->y] = false;
                }
                current_root.clear();
            }
        }
        else if(root != nullptr){
            
            current_root.push_back(root);
        }
        
        std::cout << "(" << root->x << "," << root->y << ")" << std::endl;

}

void preOrderPartition(Node* node, int single_partitin_point_count, std::vector<std::vector<Node*>>& partitions, std::vector<Node*>& partition_point){

        if(!node){
          return;
        } 

        partition_point.push_back(node);

        if(partition_point.size()>=single_partitin_point_count){
          partitions.push_back(partition_point);
          partition_point.clear();
          partition_point.push_back(node);
        }

        preOrderPartition(node->left,single_partitin_point_count,partitions,partition_point);

        preOrderPartition(node->right,single_partitin_point_count,partitions,partition_point);

        

    }

size_t getTotalNodeCount(const std::vector<std::vector<Node*>>& explored_area_graph) {
    size_t count = 0;
    for (const auto& row : explored_area_graph) {
        for (const auto* node : row) {
            if (node != nullptr) {
                ++count; // Count each non-null node
            }
        }
    }
    return count;
}