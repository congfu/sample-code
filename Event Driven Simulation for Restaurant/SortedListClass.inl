#include <iostream>
using namespace std;

#include "SortedListClass.h"
#include "LinkedNodeClass.h"

template < class T >
SortedListClass< T >::SortedListClass()
{
  // constructor
  head = 0;
  tail = 0;
}

template < class T >
SortedListClass< T >::SortedListClass(const SortedListClass< T > &rhs)
{
  // deep copy every node in list
  LinkedNodeClass< T > *temp = rhs.head; // used to traverse the old list
  head = new LinkedNodeClass< T >(0, temp->getValue(), 0); // creat new head
  LinkedNodeClass< T > *tempPrev = head; // point to node before new node
  LinkedNodeClass< T > *tempNext = head; // used to created new node
  T tempVal; // used to store value of node in old list
  temp = temp->getNext(); // move to next node in old list
  //
  while (temp != 0)
  {
    tempVal = temp->getValue(); // get value in current node in old list
    // creat new node in new list
    tempNext = new LinkedNodeClass< T >(tempPrev, tempVal, 0);
    tempNext->setBeforeAndAfterPointers();
    // move tempPrev to new node in new list
    tempPrev = tempPrev->getNext();
    temp = temp->getNext(); // move temp to next node in old list
  }
  tail = tempPrev;
}

template< class T >
void SortedListClass< T >::operator=(const SortedListClass< T > &rhs)
{
 // deep copy every node in list
 LinkedNodeClass< T > *temp = rhs.head; // used to traverse the old list
 head = new LinkedNodeClass< T >(0, temp->getValue(), 0); // creat new head
 LinkedNodeClass< T > *tempPrev = head; // point to node before new node
 LinkedNodeClass< T > *tempNext = head; // used to created new node
 T tempVal; // used to store value of node in old list
 temp = temp->getNext(); // move to next node in old list
                         //
 while (temp != 0)
 {
   tempVal = temp->getValue(); // get value in current node in old list
                               // creat new node in new list
   tempNext = new LinkedNodeClass< T >(tempPrev, tempVal, 0);
   tempNext->setBeforeAndAfterPointers();
   // move tempPrev to new node in new list
   tempPrev = tempPrev->getNext();
   temp = temp->getNext(); // move temp to next node in old list
 }
 tail = tempPrev;
}

template < class T >
void SortedListClass< T >::clear()
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

template < class T >
void SortedListClass< T >::insertValue(const T &valToInsert)
{
  if (head == 0) // empty list
  {
    // add first node in the list
    LinkedNodeClass< T > *newHead = new LinkedNodeClass< T >(0, valToInsert, 0);
    head = newHead;
    tail = newHead;
  }
  else
  {
    LinkedNodeClass< T > *cur = head; // point to current node
    LinkedNodeClass< T > *prev = 0; // point to previous node
    LinkedNodeClass< T > *temp; // used to creat node to be inserted
    while (cur->getValue() <= valToInsert && cur->getNext() != 0)
    {
      // if current node value less than or equal to insert value
      // and current pointer not equals to null
      // move prev and cur pointers to next node
      prev = cur;
      cur = cur->getNext();
    }
    if (cur->getNext() == 0 && cur->getValue() <= valToInsert)
    {
      // creat a new node and insert between prev and cur
      temp = new LinkedNodeClass< T >(cur, valToInsert, 0);
      temp->setBeforeAndAfterPointers();
      tail = temp;
    }
    else
    {
      // creat a new node and insert between prev and cur
      temp = new LinkedNodeClass< T >(prev, valToInsert, cur);
      temp->setBeforeAndAfterPointers();
      // check if temp is the first node
      if (temp->getPrev() == 0)
      {
        head = temp;
      }
    }
  }
      
}

template < class T >
void SortedListClass< T >::printForward() const
{
  LinkedNodeClass< T > *temp = head; // temp linked node
  cout << "Forward List Contents Follow:" << endl;
  while (temp != 0)
  {
    // print one node value
    cout << "  " << temp->getValue() << endl;
    // move temp pointer to next node
    temp = temp->getNext();
  }
  cout << "End Of List Contents" << endl;
}

template < class T >
void SortedListClass< T >::printBackward() const
{
  LinkedNodeClass< T > *temp = tail; // temp linked node
  cout << "Backward List Contents Follow:" << endl;
  while (temp != 0)
  {
    // print one node value
    cout << "  " << temp->getValue() << endl;
    // move temp pointer to next node
    temp = temp->getPrev();
  }
  cout << "End Of List Contents" << endl;
}

template < class T >
bool SortedListClass< T >::removeFront(T &theVal)
{
  LinkedNodeClass< T > *temp;
  if (head == 0)
  {
    return (false); // empty list
  }
  else
  {
    theVal = head->getValue();
    // let head point to new head
    temp = head;
    head = temp->getNext();
    if (head == 0)
    {
      tail = 0;
    }
    else
    {
      head->setPreviousPointerToNull();
    }
    // free memory pointed by temp
    delete temp;
    return (true);
  }
}

template < class T >
bool SortedListClass< T >::removeLast(T &theVal)
{
  LinkedNodeClass< T > *temp;
  if (tail == 0)
  {
    return (false); // empty list
  }
  else
  {
    theVal = tail->getValue();
    // let tail point to new tail
    temp = tail;
    tail = temp->getPrev();
    if (tail == 0)
    {
      head = 0;
    }
    else
    {
      tail->setNextPointerToNull();
    }
    // free memory pointed by temp
    delete temp;
    return (true);
  }
}

template < class T >
int SortedListClass< T >::getNumElems() const
{
  int count = 0;
  LinkedNodeClass< T > *temp = head;
  while (temp != 0)
  {
    count += 1; // detect new node, count plus one
    temp = temp->getNext(); // move to next node
  }
  return (count);
}

template < class T >
bool SortedListClass< T >::getElemAtIndex(const int index, T &outVal)
{
  int nodeNum = getNumElems(); // get total node number
  int iterateNum = 0;
  LinkedNodeClass< T > *temp = head;
  // corner case
  if (index + 1 > nodeNum || index < 0)
  {
    return (false);
  }
  // find node at index number
  else
  {
    while (iterateNum < index)
    {
      temp = temp->getNext(); // move to next node
      iterateNum++; // move to next index number
    }
    outVal = temp->getValue();
  }
  return (true);
}

template< class T >
SortedListClass< T >::~SortedListClass()
{
  clear();
}
