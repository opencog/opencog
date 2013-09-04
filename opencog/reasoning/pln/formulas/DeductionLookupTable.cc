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

#include "DeductionLookupTable.h"

using std::cout;
using std::endl;
using std::ios;
using std::string;
using std::vector;

namespace opencog {
namespace pln {

bool DeductionLookupTable::instanceFlag = false;
DeductionLookupTable* DeductionLookupTable::instance = NULL;
bool DeductionLookupTable::CREATE_COMPACT_FILE = true;

DeductionLookupTable::DeductionLookupTable()
{
	//the size of the lookup table
	tableSize=(int)(MAX_A*MAX_B*MAX_C*MAX_AB*MAX_BC);
	table = new VectorLookupTable(tableSize);

	count = 0;
	
		
	//filePath="/fabricio/novamente/deductionLookupTableK2N100-v5.csv";
	filePath="/fabricio/novamente/compactDeductionLookupTable-precision10-v20071219.csv";
	fname=new char[filePath.size()+1];
	filePath.copy(fname,filePath.size(), 0);
	fname[filePath.size()]='\0';
	cout << "fname is: " << fname << endl;
	lookupTableFile.open (fname, ios::app);
	
	if(CREATE_COMPACT_FILE){
		mycompactfile.open (fname, ios::app);	  		
	}	
}

DeductionLookupTable::~DeductionLookupTable(){
	lookupTableFile.close();
	if(CREATE_COMPACT_FILE){
		mycompactfile.close();			  		
	}
}

DeductionLookupTable* DeductionLookupTable::getInstance()
{
	if(instanceFlag == false)
	{
		instance=new DeductionLookupTable();
		instanceFlag=true;
	}
	return instance;
}

void DeductionLookupTable::closeFile(){
    lookupTableFile.close();
	if(CREATE_COMPACT_FILE){
        //printf("Fecnahdo arquivo\n");
        mycompactfile << std::flush;
		mycompactfile.close();			  		
	}
}


//add a tuple on the lookup table
//if CREATE_COMPACT_FILE is true, add also on the file
void DeductionLookupTable::add(float A,
                               float B,
						       float C,
							   float AB,
							   float BC,
							   float result)
{
	int key=getIndex(A, B, C, AB, BC);
	//printf("<A, B, C, AB, BC>=<result> -- <%.10f, %.10f, %.10f, %.10f, %.10f>=<%.3f>\n",A, B,C,AB,BC,result);
	//printf("Index: %d\n",key);
    
    // Used to be NULL, but comparing against a float!
	if (lookup(key) == 0.0f) {
		count++;
		table->add(key, result);
		
		if(CREATE_COMPACT_FILE){
			mycompactfile << A << ";" << B << ";" << C << ";" << AB << ";" << BC << ";" << result << ";\n";
            mycompactfile << std::flush;
		}
		if(count >= tableSize){
			cout << "Count = " << count << " -- Tabela completa\n\n";
			//exit(1);
		}
	}
}

int DeductionLookupTable::getIndex(float A,
                                   float B,
						           float C,
								   float AB,
								   float BC)
{
    //adds 0.05 to make sure that floor will return the correct int, since
    //in some cases, for instance, 0.700 are seem as 0.699...
    int xA=floor((A+0.05)*(float)MAX_A);
	int xB=floor((B+0.05)*(float)MAX_B);
	int xC=floor((C+0.05)*(float)MAX_C);
	int xAB=floor((AB+0.05)*(float)MAX_AB);
	int xBC=floor((BC+0.05)*(float)MAX_BC);
					
	int index= xA + MAX_A * (xB + MAX_B * (xC + MAX_C * (xAB + MAX_AB * xBC)));
	return index;
}

float DeductionLookupTable::lookup(float A,
                                   float B,
            					   float C,
	    						   float AB,
								   float BC)
{
	int key=getIndex(A, B, C, AB, BC);
	float result=table->lookup(key);
//	printf("<%.6f, %.6f, %.6f, %.6f, %.	6f> --- <%d>=<%.6f>\n",A,B,C,AB,BC,key, result);
	return result;										
}

float DeductionLookupTable::lookup(int index)
{
	return table->lookup(index);
}

void DeductionLookupTable::readTable()
{
	cout << "Reading lookup table\n";
	
	std::ifstream in(fname);
	vector < vector <string> > data;
  string element, delimiters = ";";
  int row = 0;
  char ch;
  
  //FABRICIO
  int column = 0;
  float A = 0.0f, B = 0.0f, C = 0.0f, AB = 0.0f, BC = 0.0f, result = 0.0f;

  //data.push_back( vector <string>() );
  while( in.read( (char*)&ch, 1 ) )
  {
    if( ch == '\n' || ch == '\r' )
    {
      row++;
      column=0;
    }
    else
    { 
    	if( delimiters.find_first_of(ch) == delimiters.npos )
    	{
      	element += ch;
    	}else
    	{

				switch(column) {
					case 0:
						A = atof(element.c_str());
						break;
					case 1:
						B = atof(element.c_str());
						break;
					case 2:
						C = atof(element.c_str());
						break;
					case 3:
						AB = atof(element.c_str());
						break;
					case 4:
						BC = atof(element.c_str());
						break;
                    case 5:
                        result = atof(element.c_str());
						add(A, B, C, AB, BC, result);
						break;
					default:
						cout << "Invalid switch case.....\n";						
				}
				column++;
      	element = "";
    	}
    }
	}
  in.close();
}

void DeductionLookupTable::printEmptyValues()
{
	cout << "Reading Lookup Table\n";
	int size=(int)(MAX_A*MAX_B*MAX_C*MAX_AB*MAX_BC);	
	float* lookupTable=table->getTable();
	for(int i=0; i<size; i++){
		if(lookupTable[i] == 0.0f){ // Used to be NULL, but comparing against a float!
			cout << "Index " << i << " NULL \n";
		}
	}
}

}} // ~namespace opencog::pln
