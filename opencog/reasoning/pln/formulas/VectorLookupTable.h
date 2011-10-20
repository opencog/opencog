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

#ifndef VECTORLOOKUPTABLE_H_
#define VECTORLOOKUPTABLE_H_

#include <vector>

namespace opencog {
namespace pln {

class VectorLookupTable
{
	public:
		VectorLookupTable(){};
		VectorLookupTable(int size){
			table = new float[size];
			this->size=size;
		};
		void add(int index, float value){table[index]=value;};
		float lookup(int index){return table[index];};
		float* getTable(){return table;};
		int getSize(){return size;};
	private:
		float* table;
		int size;	
};

}} // ~namespace opencog::pln

#endif /*VECTORLOOKUPTABLE_H_ */
