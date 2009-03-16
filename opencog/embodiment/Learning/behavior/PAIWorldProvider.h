#ifndef _PAIWORLDPROVIDER_H
#define _PAIWORLDPROVIDER_H

#include "WorldProvider.h"
#include "PAI.h"

class PAIWorldProvider : public WorldProvider
{
  PerceptionActionInterface::PAI* pai;
public:
  PAIWorldProvider(PerceptionActionInterface::PAI* _pai);
  unsigned long getLatestSimWorldTimestamp() const;
  SpaceServer& getSpaceServer() const;
};

#endif
