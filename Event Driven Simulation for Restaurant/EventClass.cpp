#include <iostream>
#include <string>
using namespace std;

#include "EventClass.h"

EventClass::EventClass()
{
  // default constructor: do nothing
  // when creating a tempEvent object, call this constructor
}

EventClass::EventClass(int inTime, string inType)
{
  time = inTime; // set event happen time
  type = inType; // set event type
}

string EventClass::getType() const
{
  return (type);
}

int EventClass::getTime() const
{
  return (time);
}

bool EventClass::operator<=(const EventClass &rhs)
{
  if (this != 0)
  {
    // compare data member "time" of two class
    if (time <= rhs.getTime())
    {
      return (true);
    }
    else
    {
      return (false);
    }
  }
  return (false);
}
