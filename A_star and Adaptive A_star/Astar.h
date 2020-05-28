#ifndef ASTAR_H_
#define ASTAR_H_

#include<iostream>
#include<vector>
#include<list>
#include<queue>
#include <unordered_map>

using namespace std;

const double kCost1 = 1.0;

// point struct on the map
struct Point
{
  int x, y; // position
  double F, G, H; // cost function
  Point *parent; // parent node
  Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)
  {
  }
};

// customized comparator for priority queue
class MyCompare
{
  public:
    bool operator()(Point* left, Point* right) const
    {
      //if (left->F == right -> F) return left->G < right->G; // in favor of g value
      return left->F > right->F;
    }
};

class Astar
{
  public:
    int totalExpandedNode = 0;
    // update the map that A* will search on
    void InitAstar(std::vector<std::vector<string>> &_maze);
    // search A* path
    std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    // update the heuristic of the nodes that have been expanded from last search
    void updateHeuristic();

  private:
    Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    // get the surrounding points of current point, which is the potential points that could be visited later
    std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
    // check if certain point could be reached
    bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const;
    // check if certain point is in the list
    Point *isInList(const std::list<Point *> &list, const Point *point) const;
    // get the point with the least f value
    Point *getLeastFpoint();

    double calcG(Point *temp_start, Point *point);
    double calcH(Point *point, Point *end);
    double calcF(Point *point);
  private:
    std::vector<std::vector<string>> maze; // map
    // a list that contains nodes that have been visited but not expanded
    std::list<Point *> openList;
    // a priority queue that contains nodes that have been visited but not expanded
    std::priority_queue<Point*, vector<Point*>, MyCompare> openList_pq; //contains node that has been 
    // a list that contains nodes that have been expanded in one search episode
    std::list<Point *> closeList;
    // a hash map contains nodes expanded across all the search episodes until now
    std::unordered_map<string, Point*> expandedSet;
    int cost = 0;
};


void printMap(const vector<vector<string>>& map, int rowSize, int colSize);
void update_Map(const vector<vector<string>>& maze, vector<vector<string>>& relativeMap, Point* start);

#endif
