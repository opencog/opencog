#include "ForgettingAgent.h"

#include <CogServer.h>

namespace opencog {

ForgettingAgent::ForgettingAgent()
{
    // forget 0.1% of atoms
    forgetPercentage = 0.001;
    // No limit to lti of removed atoms
    forgettingThreshold = AttentionValue::MAXLTI;
}

ForgettingAgent::~ForgettingAgent()
{

}

void ForgettingAgent::run(CogServer *c)
{
    a = c->getAtomSpace();
    forget(forgetPercentage);

}

void ForgettingAgent::forget(float proportion=0.10f)
{
    HandleEntry *atoms;
    std::vector<Handle> atomsVector;
    int count = 0;
    int removalAmount;

    atoms = a->getAtomTable().getHandleSet(ATOM,true);
    // Sort atoms by lti, remove the lowest unless vlti is NONDISPOSABLE
    atoms->toHandleVector(atomsVector);
    std::sort(atomsVector.begin(), atomsVector.end(), ForgettingLTIThenTVAscendingSort());
    delete atoms;

    removalAmount = (int) (atomsVector.size() * proportion);

    for (unsigned int i = 0; i < atomsVector.size() ; i++) {
	if (a->getLTI(atomsVector[i]) <= forgettingThreshold
		&& count < removalAmount) {
	    if (a->getVLTI(atomsVector[i]) != AttentionValue::NONDISPOSABLE ) {
		//cout << "Removing atom " <<  TLB::getAtom(atomsVector[i])->toString().c_str() << endl;
		MAIN_LOGGER.log(Util::Logger::FINE,"Removing atom %s", TLB::getAtom(atomsVector[i])->toString().c_str());
		if (!a->removeAtom(atomsVector[i])) {
		    MAIN_LOGGER.log(Util::Logger::ERROR,"Couldn't remove atom %s", TLB::getAtom(atomsVector[i])->toString().c_str());
		    MAIN_LOGGER.log(Util::Logger::ERROR,"Aborting forget process");
		    return;
		}
		count++;
	    }
	} else {
	    i = atomsVector.size();
	}
    }

}

}; //namespace
