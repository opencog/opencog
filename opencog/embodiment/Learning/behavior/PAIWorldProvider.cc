#include "PAIWorldProvider.h"

PAIWorldProvider::PAIWorldProvider(PerceptionActionInterface::PAI* _pai) : pai(_pai)
{}

unsigned long PAIWorldProvider::getLatestSimWorldTimestamp() const
{
    return pai->getLatestSimWorldTimestamp();
}

SpaceServer& PAIWorldProvider::getSpaceServer() const
{
  return pai->getSpaceServer();
}
