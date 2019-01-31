#include "lookup_table.h"

LookupTable::LookupTable(size_t size)
{
  if (size > 0)
    resize(size);
  
  max_label_ = NO_LABEL_;
}

void LookupTable::reset()
{
  max_label_ = NO_LABEL_;
}

size_t LookupTable::size()
{
  return table_.size();
}

void LookupTable::resize(size_t new_size)
{
  table_.resize(new_size);
}