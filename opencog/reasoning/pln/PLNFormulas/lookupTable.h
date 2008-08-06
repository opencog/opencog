#ifndef LOOKUPTABLE_
#define LOOKUPTABLE_

#include <map.h>
#include <vector.h>

using namespace std;

template<class KeyType, class ComparatorType>
class LookupTable 
{

  public:
    void add(const KeyType key, const float value);
    float lookup(const KeyType &key);
		map<const KeyType, float, ComparatorType> getTable();				
  private:
    map<const KeyType, float, ComparatorType> table;
};

template<class KeyType, class ComparatorType>
map<const KeyType, float, ComparatorType> LookupTable<KeyType, ComparatorType>::getTable()
{
	return table;	
};

template<class KeyType, class ComparatorType>
void LookupTable<KeyType, ComparatorType>::add(const KeyType key, const float value)
{
  table[key]=value;
};

template<class KeyType, class ComparatorType>
float LookupTable<KeyType, ComparatorType>::lookup(const KeyType &key)
{
  //return table[key];//inserts the key if it does not exists in table
  return table.find(key)->second;
};

#endif /*LOOKUPTABLE_*/
