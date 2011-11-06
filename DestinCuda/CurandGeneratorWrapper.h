/*
 * CurandGeneratorWrapper.h
 *
 *  Created on: Nov 4, 2011
 *      Author: ted
 */

#ifndef CURANDGENERATORWRAPPER_H_
#define CURANDGENERATORWRAPPER_H_

#include <curand.h>

class CurandGeneratorWrapper {
	curandGenerator_t gen;
public:
	CurandGeneratorWrapper(){
		 curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
	}
	~CurandGeneratorWrapper(){
		curandDestroyGenerator(gen);
	}

	curandGenerator_t& reference(){
		return gen;
	}
};

#endif /* CURANDGENERATORWRAPPER_H_ */
