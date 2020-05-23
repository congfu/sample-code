#include <iostream>
using namespace std;

#include "CustomerClass.h"

CustomerClass::CustomerClass()
{
  // default constructor
  waitingTime = 0;
}

CustomerClass::CustomerClass(int inIdx, int inArrivalTime)
{
  idx = inIdx; // set index
  arrivalTime = inArrivalTime;
}

int CustomerClass::getIdx()
{
  return (idx); // get index
}

int CustomerClass::getWaitingTime(int lastCustomerDEPTime)
{
  // waiting time equals to the departure time of the customer before this 
  // customer minus this customer's arrival time
  waitingTime = lastCustomerDEPTime - arrivalTime;
  return (waitingTime);
}
