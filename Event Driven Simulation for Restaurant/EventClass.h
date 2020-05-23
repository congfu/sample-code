#ifndef _EVENTCLASS_H_
#define _EVENTCLASS_H_

#include <string>

class EventClass
{
  private:
    int time; // event occured time
    string type; // event type arrival or departure
  public:
    // default constructor
    EventClass();
    // overload constructor: set time that this event will happen
    EventClass(int inTime, string inType);
    // This function get event type
    string getType() const;
    // This function get event happen time
    int getTime() const;

    // overload <= operator to implement class comparation
    bool operator<=(const EventClass &rhs);

};

#endif