#pragma once
#include<iostream>
#include<vector>
#include<list>
#include<queue>

using namespace std;

const int kCost1 = 1;
const int kCost2 = sqrt(2);

//class Node
//{
//  public:
//  int x,y; // node position
//  int f,g,h; // cost funtion
//  Node *parent; // parent node 
//  Node(int x0, int y0):x(x0), y(y0), f(0), g(0), h(0), parent(NULL){}
//};

struct Point
{
  int x, y;
  int F, G, H;
  Point *parent;
  Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)
  {
  }
};

class MyCompare
{
public:
  bool operator()(Point left, Point right) const
  {
    return left.F>right.F;
  }
};

class Astar
{
public:
  void InitAstar(std::vector<std::vector<int>> &_maze);
  std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);

private:
  Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
  std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
  bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const;
  Point *isInList(const std::list<Point *> &list, const Point *point) const;
  Point *getLeastFpoint();

  int calcG(Point *temp_start, Point *point);
  int calcH(Point *point, Point *end);
  int calcF(Point *point);
private:
  std::vector<std::vector<int>> maze;
  std::list<Point *> openList;
  std::list<Point *> closeList;
};