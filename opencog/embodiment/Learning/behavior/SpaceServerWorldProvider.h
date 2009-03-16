#ifndef _SPACESERVERWORLDPROVIDER_H
#define _SPECESERVERWORLDPROVIDER_H

#include "WorldProvider.h"
#include "SpaceServer.h"

//That WorldProvider is used when one would need a simple
//WorldProvider implementation, used for UTest for instance
class SpaceServerWorldProvider : public WorldProvider
{
  SpaceServer& _ss;
  unsigned long _latestSimWorldTimestamp;
public:
  SpaceServerWorldProvider(SpaceServer& ss,
			   unsigned long latestSimWorldTimestamp = 0);
  unsigned long getLatestSimWorldTimestamp() const;
  void setLatestSimWorldTimestamp(unsigned long t);
  SpaceServer& getSpaceServer() const;
};

#endif
