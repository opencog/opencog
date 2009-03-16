#ifndef _WORLD_PROVIDER_H_
#define _WORLD_PROVIDER_H_

#include "SpaceServer.h"

class WorldProvider
{
 public:
  virtual unsigned long getLatestSimWorldTimestamp() const=0;
  virtual SpaceServer& getSpaceServer() const = 0;
  virtual ~WorldProvider() { }
};

#endif // _WORLD_PROVIDER_H_

