#include <iostream>
using namespace std;
#include "FIFOQueueClass.h"

template< class T >
FIFOQueueClass< T >::FIFOQueueClass()
{
  // constructor
  head = 0;
  tail = 0;
}

template< class T >
void FIFOQueueClass< T >::enqueue(const T &newItem)
{
  if (head == 0) // empty list
  {    
    LinkedNodeClass< T > *newHead = new LinkedNodeClass< T >(0, newItem, 0);
    head = newHead;
    tail = newHead;
  }
  else
  {
    // add item to the tail of the list
    LinkedNodeClass< T > *temp;
    temp = new LinkedNodeClass< T >(tail, newItem, 0); 
    temp->setBeforeAndAfterPointers();
    tail = temp;
  }
}

template< class T >
bool FIFOQueueClass< T >::dequeue(T &outItem)
{
  LinkedNodeClass< T > *temp;
  if (head == 0)
  {
    return (false); // empty list
  }
  else
  {
    outItem = head->getValue();
    // let head point to new head
    temp = head;
    head = temp->getNext();
    if (head == 0) // empty list
    {
      tail = 0;
    }
    else
    {
      head->setPreviousPointerToNull(); // set head node's prev to null
    }
    // free memory pointed by temp
    delete temp;
    return (true);
  }
}

template< class T >
void FIFOQueueClass< T >::print() const
{
  LinkedNodeClass< T > *temp = head; // temp linked node

  while (temp != 0)
  {
    // print one node value
    cout << temp->getValue() << " ";
    // move temp pointer to next node
    temp = temp->getNext();
  }
  cout << endl;

}

template<class T >
FIFOQueueClass< T >::~FIFOQueueClass()
{
  LinkedNodeClass< T > *temp = head;
  while (temp != 0)
  {
    head = temp->getNext(); // head point to next node
    delete temp; // delete previous head node
    temp = head; // move temp to new head node
  }
  tail = 0;
}
