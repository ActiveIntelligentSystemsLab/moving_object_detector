#ifndef __HEADER_LOOKUP_TABLE__
#define __HEADER_LOOKUP_TABLE__

#include <vector>

class LookupTable
{
public:
  LookupTable(size_t size = 0);
  inline int addLabel()
  {
    max_label_++;
    table_.at(max_label_) = max_label_;
    return max_label_;
  };
  inline void link(int label1, int label2)
  {
    int label1_destination = lookup(label1);
    int label2_destination = lookup(label2);

    if (label1_destination > label2_destination)
      table_.at(label1_destination) = label2_destination;
    else
      table_.at(label2_destination) = label1_destination;
  };
  inline int lookup(int source){
    while (source != table_.at(source))
    {
      table_.at(source) = table_.at(table_.at(source));
      source = table_.at(source);
    }
    return source;
  };
  void reset();
  void resize(size_t size);
  size_t size();

private:
  const int NO_LABEL_ = -1;
  std::vector<int> table_;
  int max_label_;
};

#endif