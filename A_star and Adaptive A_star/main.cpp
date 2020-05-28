#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include "Astar.h"  
using namespace std;

int main(int argc, char** argv)
{

  bool adaptive = false; // trigger of adaptive A*
  bool initial = true; // initial plan or not

  // check input
  if (argc == 4)
  {
    adaptive = true;
  }

  // input file path
  ifstream inFile;
  string inFileName = argv[2];
  inFileName = "inputs/" + inFileName;

  // column and row size of the map
  int colSize = 0;
  int rowSize = 0;
  string Target = "g";
  string Start = "s";

  // start ang target position
  int startRow = 0;
  int startCol = 0;
  int targetRow = 0;
  int targetCol = 0;

  // read map file
  vector<vector<string>> maze;
  inFile.open(inFileName.c_str());
  if (inFile.good())
  {
    inFile >> rowSize;
    inFile >> colSize;
    for (int i = 0; i < rowSize; i++)
    {
      vector<string> mazeRow;
      for (int j = 0; j < colSize; j++)
      {        
        if (!inFile.eof())
        {
          string value;
          inFile >> value;
          if (value == Target)
          {
            targetRow = i;
            targetCol = j;
            mazeRow.push_back("g");
          }
          else if (value == Start) {
            startRow = i;
            startCol = j;
            mazeRow.push_back("s");
          }
          else if (value == "x"){
            mazeRow.push_back("x");
          }
          else {
            mazeRow.push_back("_");
          }
        }       
      }
      maze.push_back(mazeRow);
    }
  }
  cout << "Original grid map:" << endl;
  
  // print global map 
  printMap(maze, rowSize, colSize);

  cout << "Ready? Go!" << endl;

  // initial relative map
  vector<vector<string>> relativeMap(rowSize, vector<string>(colSize));
  for (int i = 0; i < rowSize; i++)
  {
    for (int j = 0; j < colSize; j++)
    {
       if (i == targetRow && j == targetCol)
       {
         relativeMap[i][j] = "g";
       }
       else if (i == startRow && j == startCol)
       {
         relativeMap[i][j] = "s";
       }
       else
       {
         relativeMap[i][j] = "_";
       }
    }
  }

  // start point
  Point start(startRow, startCol);
  Point end(targetRow, targetCol);

  // initial update
  update_Map(maze, relativeMap, &start);

  bool success = false;
  Astar astar;

  int planNumber = 1;

  while(!success)
  {

      astar.InitAstar(relativeMap);
      // adaptive astar, update heuristic of expanded node in closed list 
      if (initial == false && adaptive == true)
      {
        astar.updateHeuristic();
      }
      if (planNumber == 1)
      {
        cout << "# of plan: " << planNumber << "(initial)" << start.x << start.y << endl;
        initial = false;
      }
      else
      {
        cout << "# of plan: " << planNumber << "(replan)" << start.x << start.y << endl;
      }
      
      // find shortest path from current start point to the goal point
      list<Point *> path = astar.GetPath(start, end, false);

      // check path
      if (!path.empty())
      {
        // draw path
        vector<vector<string>> printedMap = relativeMap;
        for (auto p = ++path.begin(); p != --path.end(); p++)
        {
          printedMap[(*p)->x][(*p)->y] = "o";
        }
        printMap(printedMap, rowSize, colSize);
      }
      else
      {
        printMap(relativeMap, rowSize, colSize);
        cout << "No route found" << endl;
        cout << "Total expanded nodes: " << astar.totalExpandedNode << endl;
        break;
      }

      // iterate over path(try to move along path)
      for (auto p = path.begin(); p != path.end(); p++)
      {
        //check availibility
        if (maze[(*p)->x][(*p)->y] == "x") // encounter obstacle
        {
          p--;
          start = **p;
          break;  //replan from last position
        }
        else if (maze[(*p)->x][(*p)->y] == "g")
        {
          success = true;
          cout << "success" << endl;
          cout << "Total expanded nodes: " << astar.totalExpandedNode << endl;
        }
        else
        {
          relativeMap[start.x][start.y] = "_";
          start = **p;
          relativeMap[start.x][start.y] = "s";
          //update map
          update_Map(maze, relativeMap, &start);
        }
      }
      planNumber++;
  }

  return 0;
}
