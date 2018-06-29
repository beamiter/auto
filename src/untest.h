#ifndef _UNTEST_
#define _UNTEST_


#include "reflect.h"

//声明再继承类
class yinjian : public CDerived
{
  DECLEAR_DYNCRT_CLASS(yinjian, CBase)
public:
  virtual void Print()
  {
    cout << "This is yinjian!" << endl;
    a = 1;
    b = 2; 
    c = 3;
    cout << a << "; " << b << "; " << c << endl;
  }
};
IMPLEMENT_DYNCRT_CLASS(yinjian)



#endif