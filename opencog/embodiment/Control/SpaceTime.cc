
#include "SpaceTime.h"

namespace opencog {

SpaceServer* SpaceTimeCogServer::spacer = NULL;
TimeServer* SpaceTimeCogServer::timeser = NULL;

SpaceServer& SpaceTimeCogServer::getSpaceServer()
{
    return *spacer;
}

TimeServer& SpaceTimeCogServer::getTimeServer()
{
    return *timeser;
}

SpaceTimeCogServer::SpaceTimeCogServer()
{
    spacer = new SpaceServer(getAtomSpace());
    timeser = new TimeServer(getAtomSpace(), spacer);
}

SpaceServer& spaceServer()
{
   return dynamic_cast<SpaceTimeCogServer&>(server()).getSpaceServer();
}

TimeServer& timeServer()
{
   return dynamic_cast<SpaceTimeCogServer&>(server()).getTimeServer();
}

} // namespace opencog
