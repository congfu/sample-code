#include <iostream>
using namespace std;

template < class T >
T getUserInput()
{
  T inVal;
  bool validInputFound;
  // Get user input choice
  validInputFound = false;
  while (!validInputFound)
  {
    cin >> inVal;
    if (cin.fail() || inVal < 0)
    {
      cin.clear();
      cin.ignore(200, '\n');
      cout << "Input not valid, try another value!: ";
    }
    else
    {
      validInputFound = true;
    }
  }
  return (inVal);
}

