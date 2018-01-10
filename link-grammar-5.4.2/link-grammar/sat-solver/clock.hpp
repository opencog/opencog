#ifndef __CLOCK_H__
#define __CLOCK_H__

/**
*   Time measurment functions
*/

#include <iostream>
#include <ctime>

class Clock {
private:
	clock_t  start;
public:
	Clock() {
		reset();
	}
	
	void reset()
	{
		start = clock();
	}

public:	
	double elapsed()
	{
		clock_t stop = clock();
		return ((double)stop-(double)start)/CLOCKS_PER_SEC;
	
	}
};

#endif
