#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include "Temporal.h"

using namespace opencog;

/**
 * This class is just an extension of Temporal class for indicating 
 * that its attribute values are actually related to a time stamp.
 */
class TimeStamp : public Temporal {

friend class TimeServer; 

public:
    TimeStamp(bool, unsigned long);
	virtual ~TimeStamp();
    unsigned long getValue();

protected:    
    TimeStamp(bool, unsigned long, unsigned long);
    
};

#endif //TIMESTAMP_H
