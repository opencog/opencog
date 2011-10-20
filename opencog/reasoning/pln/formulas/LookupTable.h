/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef LOOKUPTABLE_
#define LOOKUPTABLE_

#include <map.h>
#include <vector.h>

namespace opencog {
namespace pln {

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
  return table.find(key)->second;
};

}} // ~namespace opencog::pln

#endif /*LOOKUPTABLE_*/
