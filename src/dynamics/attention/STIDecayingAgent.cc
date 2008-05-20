#include "STIDecayingAgent.h"
#include "CogServer.h"

using namespace opencog;

STIDecayingAgent::STIDecayingAgent() {
}

STIDecayingAgent::~STIDecayingAgent() {
}

void STIDecayingAgent::run(CogServer *cogserver) {
    cogserver->getAtomSpace()->decayShortTermImportance();;
}
