#include "lookup_table.h"

LookupTable::LookupTable(size_t size)
{
  if (size > 0)
    resize(size);
}

void LookupTable::arrange()
{
  for (int i = 1; i < size(); i++)
    table_.at(i) = lookup(i);
}

void LookupTable::clear()
{
  table_.clear();
}

int LookupTable::lookup(int source)
{
  if (source < 0)
    return -1;

  int temporary_destination = table_.at(source);
  if (temporary_destination == source)
    return temporary_destination;

  return lookup(temporary_destination);
}

size_t LookupTable::size()
{
  return table_.size();
}

void LookupTable::resize(size_t new_size)
{
  if (new_size <= size())
    return;

  size_t old_size = size();
  table_.resize(new_size);

  // サイズが大きくなった場合，増えた分の要素をsource == destinationとして初期化
  for (int i = old_size; i < new_size; i++)
    table_.at(i) = i;
}

void LookupTable::update(int source, int destination)
{
  if (source < destination)
    return;
  
  if (source < 0)
    return;
  
  if (source > size() - 1)
    resize(source + 1);

  table_.at(source) = lookup(destination);
}