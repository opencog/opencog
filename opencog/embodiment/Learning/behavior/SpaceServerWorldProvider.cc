#include "SpaceServerWorldProvider.h"

SpaceServerWorldProvider::SpaceServerWorldProvider(SpaceServer& ss,
						   unsigned long
						   latestSimWorldTimestamp)
  : _ss(ss), _latestSimWorldTimestamp(latestSimWorldTimestamp) {}

unsigned long SpaceServerWorldProvider::getLatestSimWorldTimestamp() const
{
    return _latestSimWorldTimestamp;
}

void SpaceServerWorldProvider::setLatestSimWorldTimestamp(unsigned long t) {
  _latestSimWorldTimestamp = t;
}

SpaceServer& SpaceServerWorldProvider::getSpaceServer() const
{
  return _ss;
}
