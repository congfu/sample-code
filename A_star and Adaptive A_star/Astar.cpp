#include <math.h>
#include <algorithm>
#include <iostream>
#include <string>
#include "Astar.h"  

using namespace std;

void update_Map(const vector<vector<string>>& maze, vector<vector<string>>& relativeMap, Point* start)
{
  for (int x = start->x - 1; x <= start->x + 1; x++)
  {
    for (int y = start->y - 1; y <= start->y + 1; y++)
    {
      if (abs(start->x - x) + abs(start->y - y) == 1)
      {
        if (x<0 || x>relativeMap.size() - 1
          || y<0 || y>relativeMap[0].size() - 1)
        {
          // do nothing
        }
        else if (maze[x][y] == "x") // update the map when see the obstacle
        {
          relativeMap[x][y] = "x";
        }
      }
    }
  }
}

void printMap(const vector<vector<string>>& map, int rowSize, int colSize)
{
  for (int i = 0; i < rowSize; i++)
  {
    for (int j = 0; j < colSize; j++)
    {
      cout << map[i][j] << " ";
    }
    cout << endl;
  }
}

void Astar::InitAstar(std::vector<std::vector<string>> &_maze)
{
  maze = _maze;
  // clear the openlist
  openList.clear();
  // clear the openlist_pq
  while (!openList_pq.empty())
  {
    openList_pq.pop();
  }
}

double Astar::calcG(Point *temp_start, Point *point)
{
  //double parentG = (point->parent == NULL ? 0 : point->parent->G);
  double parentG = (temp_start == NULL ? 0 : temp_start->G);
  return parentG + 1;
}

double Astar::calcH(Point *point, Point *end)
{
  return abs(end->x - point->x) + abs(end->y - point->y);
}

double Astar::calcF(Point *point)
{
  return point->G + point->H;
}

Point *Astar::getLeastFpoint()
{
  // pop the node with the least f value in the priority queue
  if (!openList_pq.empty())
  {
    return openList_pq.top();
  }
  return NULL;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
  Point* startNode = new Point(startPoint.x, startPoint.y);
  // push start node in the openlist
  openList.push_back(startNode);
  openList_pq.push(startNode);
  while (!openList_pq.empty())
  {
    auto curPoint = getLeastFpoint();
    totalExpandedNode += 1;
    openList.remove(curPoint);
    openList_pq.pop();
    closeList.push_back(curPoint);

    auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
    for (auto &target : surroundPoints)
    {

      if (!isInList(openList, target))
      {
        target->parent = curPoint;

        target->G = calcG(curPoint, target);
        // if this node is expanded in the previous search, then use more informed heuristic
        if (expandedSet.find(to_string(target->x) + to_string(target->y)) != expandedSet.end())
        {
          target->H = expandedSet[to_string(target->x) + to_string(target->y)]->H;
        }
        else
        {
          target->H = calcH(target, &endPoint);
        }
        target->F = calcF(target);

        openList.push_back(target);
        openList_pq.push(target);
      }

      else // in the openlist 
      {
        // if new g value is less than old g value, then update the parent node of this node
        double tempG = calcG(curPoint, target);
        if (tempG<target->G)
        {
          target->parent = curPoint;

          target->G = tempG;
          target->F = calcF(target);
        }
      }

      // check if goal node is in the openlist
      Point *resPoint = isInList(openList, &endPoint);
      if (resPoint)
      {
        closeList.push_back(resPoint);
        return resPoint;
      }
    }
  }
  
  return NULL;
}

std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
  // clear closelist
  closeList.clear();
  // find the shortest path to the goal, return the goal point
  Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
  // from goal point, find the whole path through parent pointer of each node
  std::list<Point *> path;
  while (result)
  {
    path.push_front(result);
    result = result->parent;
  }
  return path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
  // check if certain point is in the list
  for (auto p : list)
    if (p->x == point->x&&p->y == point->y)
      return p;
  return NULL;
}

bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{
  if (target->x<0 || target->x>maze.size() - 1         // boundary
    || target->y<0 || target->y>maze[0].size() - 1     // boundary
    || maze[target->x][target->y] == "x"               // obstacle
    || target->x == point->x&&target->y == point->y    // current point
    || isInList(closeList, target))                    // in closeilst
    return false;
  else
  { 
    return true;
  }
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const
{
  std::vector<Point *> surroundPoints;

  // get surrounding points of current point
  for (int x = point->x - 1; x <= point->x + 1; x++)
    for (int y = point->y - 1; y <= point->y + 1; y++)
      if (abs(point->x - x) + abs(point->y - y) == 1) // only check four direction, up/down/left/right
      {
        if (isCanreach(point, new Point(x, y), isIgnoreCorner))
          surroundPoints.push_back(new Point(x, y));
      }
  return surroundPoints;
}

void Astar::updateHeuristic()
{
  list<Point *>::iterator it = closeList.end();
  it--;
  Point goal = **it;

  for(auto p : closeList)
  {
    p->H = goal.G - p->G;
    // if the node is in the expanded set, then update the heuristic value according to previous search result
    // if not, then insert the point into the map with the updated heuristic
    if (expandedSet.find(to_string(p->x) + to_string(p->y)) != expandedSet.end())
    {
      expandedSet[to_string(p->x) + to_string(p->y)] = p;
    }
    else
    {
      expandedSet.insert({ to_string(p->x) + to_string(p->y), p });
    }
    
  }
  closeList.clear();
}
