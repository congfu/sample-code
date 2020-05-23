#include <iostream>
#include <string>
using namespace std;

#include "SimulationClass.h"
#include "SortedListClass.h"
#include "FIFOQueueClass.h"
#include "EventClass.h"
#include "CustomerClass.h"
#include "random.h"

SimulationClass::SimulationClass(int inTime,
                                 int inSeed,
                                 int inMin,
                                 int inMax,
                                 double inMean,
                                 double inStdDev)
{
  closeTime = inTime; // restaurant close time
  setSeed(inSeed);
  queueHasCustomer = false;
  arrivalTime = 0; // used to store arrival time
  customerNum = 0; // total costumer number
  customerNumNeedWaiting = 0; // total number of customers who need to wait
  customerNumInline = 0; // number of customers waiting in line 
  longestLineNum = 0; // longest line customer number
  // Waiting time of the customer who wait for the longest time
  longestCustomerWatingTime = 0;
  totalWaitingTime = 0; // total waiting time for all customer
  totalServiceTime = 0;
  min = inMin; // uniform distribution min value
  max = inMax; // uniform distribution max value
  mean = inMean; // normal distribution mean value
  stdDev= inStdDev; // normal distribution standard deviation value
  cout << "Simulation start! Restaurant close time is "
       << closeTime << endl << endl;
}

void SimulationClass::setUpSimulation()
{
  cout << "Restaurant open!" << endl << endl;
  arrivalTimeInverval = getUniform(min, max);
  arrivalTime += arrivalTimeInverval; // first customer arrival time
  departureTime = arrivalTime; // set initial departure time offset
  // first customer arrived event, to start the simulation
  EventClass newArrivalEvent(arrivalTime, "arrival");
  eventList.insertValue(newArrivalEvent);
}

void SimulationClass::startSimulation()
{
  
  while (eventList.getNumElems() != 0)
  {
    EventClass tempEvent;
    eventList.removeFront(tempEvent); // handle first event in event list
    
    if (tempEvent.getType() == "arrival")
    {
      // handle arrival event
      handleArrivalEvent();
    }
    else if (tempEvent.getType() == "departure")
    {
      // handle departure event
      cout << "Server finished serving previous customer at time :"
           << tempEvent.getTime() << endl << endl;
      handleDepartureEvent();
    }
  }

  // output simulation end prompt
  cout << "All cusotmers have been served and simulation ends!!!"
       << endl << endl;
}

void SimulationClass::handleDepartureEvent()
{
  int oneCustomerWaitingTime = 0;

  customerNumInline--; // decrease customer number in line by one
  
  // dequeue first customer in FIFOqueue and serve this customer
  CustomerClass servicingCustomer;
  queueHasCustomer = queue.dequeue(servicingCustomer);

  // record longest waiting time for one customer
  if (queueHasCustomer)
  {
    oneCustomerWaitingTime = servicingCustomer.getWaitingTime(departureTime);
    totalWaitingTime += oneCustomerWaitingTime;
  }

  if (oneCustomerWaitingTime > longestCustomerWatingTime)
  {
    longestCustomerWatingTime = oneCustomerWaitingTime;
  }
  
  // decide whether need to serve customer
  if (queueHasCustomer)
  {
    // FIFOqueue has customer
    // serve first customer in queue
    serveCustomer();
    cout << "Customer #" << servicingCustomer.getIdx() 
         << " start to be served!" << endl << endl;
  }
}

void SimulationClass::handleArrivalEvent()
{
  // add arrived customer to FIFOqueue
  CustomerClass newCustomer(++customerNum, arrivalTime);


  customerNumInline++; // increase customer number in line by one
  // store the longest line 
  if (customerNumInline > longestLineNum)
  {
    longestLineNum = customerNumInline;
  }

  cout << "Customer #" << customerNum << " has arrived at time: " 
       << arrivalTime << endl;


  // if there's other customer waiting in line 
  // add this customer in the queue
  if (queueHasCustomer)
  {
    customerNumNeedWaiting++;
    // add this customer in FIFOqueue
    queue.enqueue(newCustomer);
    cout << "Customer #" << customerNum << " is waiting in line! "
         << "And there are " << customerNumInline - 1 << " people"
         << " in front of him/her." << endl << endl;
  }
  // if there is no customer waiting in line
  // directly serce this customer
  else
  {
    serveCustomer();
    cout << "There is no people in front of "
         << "Customer #" << newCustomer.getIdx() << endl;
    cout << "Customer #" << newCustomer.getIdx()
         << " start to be served!" << endl << endl;
    queueHasCustomer = true;
  }

  // determine when will next customer arrive
  arrivalTimeInverval = getUniform(min, max);

  // update arrival time
  arrivalTime += arrivalTimeInverval;

  if (arrivalTime < closeTime)
  {
    EventClass newArrivalEvent(arrivalTime, "arrival");
    // insert arrival event to the event list
    eventList.insertValue(newArrivalEvent);
  }
  else
  {
    cout << "Restaurant accept no more new customer!!!" << endl << endl;
  }
}

void SimulationClass::serveCustomer()
{
  // determine how long this customer will be served
  // and generate departure event
  serviceTime = getNormal(mean, stdDev);

  if (queueHasCustomer)
  {
    departureTime += serviceTime; // customer departure time
  }
  else
  {
    departureTime = arrivalTime + serviceTime; // customer departure time
  }

  totalServiceTime += serviceTime;
  // creat new departure event
  EventClass newDepartureEvent(departureTime, "departure");
  // insert departure event to the event list
  eventList.insertValue(newDepartureEvent);
}

void SimulationClass::showStatistics()
{
  double percentageService;
  double percentageWaiting;
  double averageServiceTime;
  double averageWaitingTime;
  // calculate What percentage of time the server was busy helping customers
  percentageService = static_cast< double >(totalServiceTime)/
                      static_cast< double >(departureTime)*100;
  // What percentage of customers had to wait in line
  percentageWaiting = static_cast< double >(customerNumNeedWaiting)/
                      static_cast< double >(customerNum)*100;
  // Average service time over the whole simulation
  averageServiceTime = static_cast< double >(totalServiceTime)/
                       static_cast< double >(customerNum);
  // Average waiting time
  averageWaitingTime = static_cast< double >(totalWaitingTime)/
                       static_cast< double >(customerNum);
  // show statistic data
  cout << "Below are the statistics from this simulation: " << endl;
  cout << "Total number of customers simulated: " << customerNum << endl;
  cout << "What percentage of time the server was busy helping customers: "
       << percentageService << "%" << endl;
  cout << "What percentage of customers had to wait in line: "
       << percentageWaiting << "%" << endl;
  cout << "The longest the line was throughout the simulation: "
       << longestLineNum << endl;
  cout << "Average service time over the whole simulation: "
       << averageServiceTime << endl;
  cout << "Waiting time of the customer who wait for the longest time: "
       << longestCustomerWatingTime << endl;
  cout << "Average waiting time over the whole simulation: "
       << averageWaitingTime << endl;
}
