#include <iostream>
using namespace std;
#include "LinkedNodeClass.h"

template < class T >
LinkedNodeClass< T >::LinkedNodeClass(LinkedNodeClass *inPrev,
                                      const T &inVal,
                                      LinkedNodeClass *inNext)
{
  prevNode = inPrev; // set prevNode
  nodeVal = inVal; // set node value
  nextNode = inNext; // set nextNode
}

template < class T >
T LinkedNodeClass< T >::getValue() const
{
  if (this != 0)
  {
    return nodeVal; // return this node value
  }
}

template < class T >
LinkedNodeClass< T >* LinkedNodeClass< T >::getNext() const
{
  if (this != 0)
  {
    return nextNode; // return next node pointer
  }
}

template < class T >
LinkedNodeClass< T >* LinkedNodeClass< T >::getPrev() const
{
  if (this != 0)
  {
    return prevNode; // return prev node pointer
  }
}

template < class T >
void LinkedNodeClass< T >::setNextPointerToNull()
{
  if (this != 0)
  {
    nextNode = 0; // set next node to null
  }
}

template < class T >
void LinkedNodeClass< T >::setPreviousPointerToNull()
{
  if (this != 0)
  {
    prevNode = 0; // set previous node to null
  }
}

template < class T >
void LinkedNodeClass< T >::setBeforeAndAfterPointers()
{
  // set previous node point to this node
  if (prevNode != 0)
  {
    prevNode->nextNode = this;
  }
  // set next node points to this node
  if (nextNode != 0)
  {
    nextNode->prevNode = this;
  }
}

