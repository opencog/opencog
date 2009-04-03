#ifndef _REAL_TIME_WORLD_PROVIDER_H_
#define _REAL_TIME_WORLD_PROVIDER_H_
#include "WorldProvider.h"
#include "PAITestUtil.h"

class RealTimeWorldProvider : public WorldProvider
{
    AtomSpace* atomSpace;
public:
    RealTimeWorldProvider(AtomSpace* _atomSpace) : atomSpace(_atomSpace) {}
    unsigned long getLatestSimWorldTimestamp() const {
        return PerceptionActionInterface::PAITestUtil::getCurrentTimestamp();
    }
    AtomSpace* getAtomSpace() const {
        return atomSpace;
    }
};

#endif // _REAL_TIME_WORLD_PROVIDER_H_

