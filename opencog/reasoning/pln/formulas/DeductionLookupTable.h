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

#ifndef DEDUCTIONLOOKUPTABLE_H_
#define DEDUCTIONLOOKUPTABLE_H_

#include "VectorLookupTable.h"

#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <string>

/**
 * This file (and deductionLookupTable.cc) deal with the lookup table for deduction
 * rule. The description on how should it be implemented is in the twiki page.
 * The lookup table is stored as a csv file, in the format 
 * "s_A;s_B;s_C;s_AB;S_BC;s_AC;"
 * The memory lookup table is the VectorLookupTable data structure (table variable)
 * 
 */


namespace opencog {
namespace pln {
//the MAX_x must be the same as the "precision" of the table.
//If each variable (A,B...) vary from 10 values, MAX_x must be 10
const int MAX_A=10;
const int MAX_B=MAX_A;
const int MAX_C=MAX_A;
const int MAX_AB=MAX_A;
const int MAX_BC=MAX_A;

class DeductionLookupTable 
{
	public:
		static DeductionLookupTable* getInstance();
		~DeductionLookupTable();
		
		void readTable();
		float lookup(float A, float B, float C, float AB, float BC);
		void add(float A, float B, float C, float AB, float BC, float result);
		int getLookupTableSize(){return table->getSize();};
	    void closeFile();	
	private:
		DeductionLookupTable();
		static DeductionLookupTable* instance;
		static bool instanceFlag;
		
		static bool CREATE_COMPACT_FILE;
		
		int count;//counts the number of tubles saves in the table
		
		std::ofstream lookupTableFile;
		std::string filePath;
		char *fname;
		std::ofstream mycompactfile;
		int tableSize;
		
		VectorLookupTable* table;
		float lookup(int index);
		int getIndex(float A, float B, float C, float AB, float BC);
        		
		void printEmptyValues();
};

}}

#endif /*DEDUCTIONLOOKUPTABLE_H_*/
