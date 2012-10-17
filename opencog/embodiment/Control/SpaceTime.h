
#ifndef _EMBODIMENT_SPACETIME_H
#define _EMBODIMENT_SPACETIME_H

#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/embodiment/Control/MessagingSystem/EmbodimentCogServer.h>

namespace opencog {

class SpaceTimeCogServer : public messaging::EmbodimentCogServer
{

protected:
    static SpaceServer* spacer;
    static TimeServer* timeser;
public:
    SpaceTimeCogServer();
    SpaceServer& getSpaceServer();
    TimeServer& getTimeServer();
};

SpaceServer& spaceServer();
TimeServer& timeServer();

} // namepsace opencog

#endif // _EMBODIMENT_SPACETIME_H

