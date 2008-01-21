#ifndef HANDLETEMPORALPAIR_H
#define HANDLETEMPORALPAIR_H

#include "types.h"
#include "Temporal.h"

class HandleTemporalPair
{
public:
	HandleTemporalPair();
	HandleTemporalPair(Handle, Temporal*);
	virtual ~HandleTemporalPair();
    
    Handle getHandle() const;
    Temporal* getTemporal() const;
    std::string toString() const;
    HandleTemporalPair clone();
    
private:
    Handle handle;
    Temporal* time;
     
};

#endif //HANDLETEMPORALPAIR_H
