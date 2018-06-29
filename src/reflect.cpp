#include "reflect.h"
#include "untest.h"

int main(int argc, char *argv[])
{
  CBase *base = CBase::Create("CBase");
  if (base)
  {
    base->Print();
  }
  CBase *base2 = CBase::Create("CDerived");
  if (base2)
  {
    base2->Print();
  }
  CBase *base3 = CBase::Create("ExCDerived");
  if (base3)
  {
    base3->Print();
  }

  CBase *base4 = CBase::Create("yinjian");
  if (base4)
  {
    base4->Print();
  }
  return 0;
}