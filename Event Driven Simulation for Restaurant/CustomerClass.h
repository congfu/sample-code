#ifndef _CUSTOMERCLASS_H_
#define _CUSTOMERCLASS_H_

class CustomerClass
{
  private:
    int idx; // current customer index
    int arrivalTime;
    int waitingTime;
  public:
    // default constructor
    CustomerClass();
    // overload constructor, used to creat new customer and set index to 
    // this customer
    CustomerClass(int inIdx, int inArrivalTime);
    // get index of this customer
    int getIdx();
    // This function get waiting time for this costomer
    int getWaitingTime(int lastCustomerDEPTime);
};

#endif