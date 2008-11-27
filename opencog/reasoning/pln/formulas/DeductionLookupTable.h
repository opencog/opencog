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

using std::string;

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
		
		ofstream lookupTableFile;
		string filePath;
		char *fname;
		ofstream mycompactfile;
		int tableSize;
		
		VectorLookupTable* table;
		float lookup(int index);
		int getIndex(float A, float B, float C, float AB, float BC);
        		
		void printEmptyValues();
};

#endif /*DEDUCTIONLOOKUPTABLE_H_*/
