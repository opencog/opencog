#ifndef VECTORLOOKUPTABLE_H_
#define VECTORLOOKUPTABLE_H_

#include <vector>

using namespace std;

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

#endif /*VECTORLOOKUPTABLE_H_ */
