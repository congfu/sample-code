#ifndef _SIMULATIONCLASS_H_
#define _SIMULATIONCLASS_H_

#include "SimulationClass.h"
#include "FIFOQueueClass.h"
#include "SortedListClass.h"
#include "EventClass.h"
#include "CustomerClass.h"

class SimulationClass
{
  private:
    int closeTime; // restaurant close time
    int arrivalTime; // used to store arrival time
    int departureTime; // used to store departure time
    int customerNum; // total costumer number
    int customerNumNeedWaiting; // total number of customers who need to wait
    int customerNumInline; // number of customers waiting in line 
    int longestLineNum; // longest line customer number
    // Waiting time of the customer who wait for the longest time
    int longestCustomerWatingTime;
    int totalWaitingTime; // total waiting time for all customer
    int min; // uniform distribution min value
    int max; // uniform distribution max value
    double mean; // normal distribution mean value
    double stdDev; // normal distribution standard deviation value
    bool queueHasCustomer;
    int arrivalTimeInverval; // arrived time interval between two customer
    int serviceTime; // serivce time for a customer
    int totalServiceTime;
    // creat a sortedlist to store event
    SortedListClass< EventClass > eventList;
    // creat a FIFOqueue to store customer
    FIFOQueueClass< CustomerClass > queue;
  public:
    // constructor, set stop time of the simulation
    SimulationClass(int inTime,
                    int seed,
                    int inMin,
                    int inMax,
                    double inMean,
                    double inStdDev);

    // This function generate first customer to start off the simulation
    // The first customer arrival event will be add into event list 
    void setUpSimulation();

    // Handle first customer arrival event and serve first customer, add 
    // first customer departure event to the event list 
    void startSimulation();

    // This function handle departure event: dequeue next customer from
    // FIFOqueue and serve this customer
    void handleDepartureEvent();

    // This funtion handle arrival event: add arrived customer to the 
    // FIFOqueue and generate next customer arrival event to the event list
    void handleArrivalEvent();

    // Determine how long this customer will be served and
    // generate departure event of this customer to the event list
    void serveCustomer();

    // this function show statistics data
    void showStatistics();

};

#endif