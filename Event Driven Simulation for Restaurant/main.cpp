#include <iostream>
using namespace std;

#include "SortedListClass.h"
#include "FIFOQueueClass.h"
#include "SimulationClass.h"
#include "getUserInput.h"
// Name: Cong Fu
// Last Modified: 11/30/2017
// Description: In this project I will implement a simulation
//              The simulation to implement will be a server simulation
//              at a fast food restaurant.

int main()
{
  int closeTime; // restaurant close time
  int seed; // seed to generate random number
  int min; // uniform distribution min value
  int max; // uniform distribution max value
  double mean; // normal distribution mean value
  double stdDev; // normal distribution standard deviation value
  // input stop time
  cout << "Input restaurant close time: ";
  closeTime = getUserInput< int >();
  // input random number seed
  cout << "Input random number seed: ";
  seed = getUserInput< int >();
  // input uniform distribution min value
  cout << "Input uniform distribution min value: ";
  min = getUserInput< int >();
  // input uniform distribution max value
  cout << "Input uniform distribution max value: ";
  max = getUserInput< int >();
  // input normal distribution mean value
  cout << "Input normal distribution mean value: ";
  mean = getUserInput< double >();
  // input normal distribution standard deviation value
  cout << "Input normal distribution standard deviation value: ";
  stdDev = getUserInput< double >();
  cout << endl;
  // creat simulation object
  SimulationClass testSimulation(closeTime, seed, min, max, mean, stdDev);
  testSimulation.setUpSimulation();
  testSimulation.startSimulation();
  testSimulation.showStatistics();

  return 0;
}
