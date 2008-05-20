/**
 * STIDecayingAgent.h
 *
 * Author: Gustavo Gama
 * Creation: Thu May 15 12:34:19 BRT 2008
 */

/* Short description: this agent implements another forgetting mechanism, way
 * simpler than Joel's forgetting agent. It mimmics the forgetting mechanism
 * used by Petaverse */

#ifndef _STI_DECAYING_AGENT_H
#define _STI_DECAYING_AGENT_H

#include "Logger.h"
#include "AtomSpace.h"
#include "MindAgent.h"

namespace opencog {

class CogServer;

class STIDecayingAgent : public MindAgent {

public:
	STIDecayingAgent();
	virtual ~STIDecayingAgent();
	virtual void run(CogServer *server);

}; // class

}; // namespace
#endif // _STI_DECAYING_AGENT_H

