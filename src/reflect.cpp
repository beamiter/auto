#include "reflect.h"
#include "untest.h"


int main(int argc, char *argv[])
{
  vector<shared_ptr<CBase> > storage;
  shared_ptr<CBase> base = CBase::Create("CBase");
  if (base)
  {
    storage.push_back(base);
  }
  shared_ptr<CBase> base2 = CBase::Create("CDerived");
  if (base2)
  {
    storage.push_back(base2);
  }
  shared_ptr<CBase> base3 = CBase::Create("ExCDerived");
  if (base3)
  {
    storage.push_back(base3);
  }
  shared_ptr<CBase> base4 = CBase::Create("yinjian");
  if (base4)
  {
    storage.push_back(base4);
  }
  for (auto a : storage)
  {
    a->Print();
  }
  return 0;
}