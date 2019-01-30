#ifndef __HEADER_LOOKUP_TABLE__
#define __HEADER_LOOKUP_TABLE__

#include <vector>

class LookupTable
{
public:
  LookupTable(size_t size = 0);
  void arrange();
  void clear();
  int lookup(int source);
  void resize(size_t size);
  size_t size();
  void update(int source, int destination);
private:
  std::vector<int> table_;
};

#endif