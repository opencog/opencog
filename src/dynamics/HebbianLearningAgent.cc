#include "HebbianLearningAgent.h"

#include <AtomSpace.h>
#include <CogServer.h>
#include <SimpleTruthValue.h>

namespace opencog {

HebbianLearningAgent::HebbianLearningAgent()
{
    convertLinks = false;
    conversionThreshold = 15;

}

HebbianLearningAgent::~HebbianLearningAgent()
{

}

void HebbianLearningAgent::run(CogServer *server)
{
    a = server->getAtomSpace();
    hebbianLearningUpdate();
}

void HebbianLearningAgent::hebbianLearningUpdate()
{
    HandleEntry *links, *current_l;

    // tc affects the truthvalue
    float tc, old_tc, new_tc;
    float tcDecayRate = 0.1;

    MAIN_LOGGER.log(Util::Logger::DEBUG,"----------- Hebbian Learning update");

    // get links again to include the new ones
    links = a->getAtomTable().getHandleSet(HEBBIAN_LINK, true);

    for (current_l = links; current_l; current_l = current_l->next) {
	// for each hebbian link, find targets, work out conjunction and convert
	// that into truthvalue change. the change should be based on existing TV.
	Handle h;
	std::vector<Handle> outgoing;

	h = current_l->handle;
	
	// get out going set
	outgoing = TLB::getAtom(h)->getOutgoingSet();
	new_tc = targetConjunction(outgoing);
	// old link strength decays 
	old_tc = a->getTV(h).getMean();

	if (convertLinks && a->getLTI(h) < conversionThreshold)
	{
	    float temp_tc;
	    // If mind agent is set to convert hebbian links then
	    // inverse and symmetric links will convert between one
	    // another when conjunction between sti values is right
	    // (initially for hopfield emulation, but could
	    // be useful in other cases)
	    if (TLB::getAtom(h)->getType() == INVERSE_HEBBIAN_LINK) {
		temp_tc = (tcDecayRate * -new_tc) + ( (1.0-tcDecayRate) * old_tc);
		if (temp_tc < 0) {
		    // Inverse link no longer representative
		    // change to symmetric hebbian link
		    a->removeAtom(h);
		    h = a->addLink(SYMMETRIC_HEBBIAN_LINK, outgoing, SimpleTruthValue(-temp_tc, 1));
		} else {
		    a->setMean(h,temp_tc);
		}
	    } else {
		temp_tc = (tcDecayRate * new_tc) + ( (1.0-tcDecayRate) * old_tc);
		if (temp_tc < 0) {
		    // link no longer representative
		    // change to inverse hebbian link
		    a->removeAtom(h);
		    outgoing = moveSourceToFront(outgoing);
		    h = a->addLink(INVERSE_HEBBIAN_LINK, outgoing, SimpleTruthValue(-temp_tc, 1));
		} else {
		    a->setMean(h,temp_tc);
		}
	    }

	} else { 
	    // otherwise just update link weights
	    tc = (tcDecayRate * new_tc) + ( (1.0-tcDecayRate) * old_tc);
	    if (tc < 0.0f) tc = 0.0f;
	    a->setMean(h,tc);
	}
	MAIN_LOGGER.log(Util::Logger::FINE,"HebLearn: %s old tv %f", TLB::getAtom(h)->toString().c_str(), old_tc);
	
    }
    // if not enough links, try and create some more either randomly
    // or by looking at what nodes are in attentional focus

    delete links;

}


std::vector<Handle>& HebbianLearningAgent::moveSourceToFront(std::vector<Handle> &outgoing)
{
    // find source, atom with positive norm_sti, and make first in outgoing
    std::vector<Handle>::iterator outgoing_i;
    Handle theSource;
    bool foundTheSource = false;
    for (outgoing_i = outgoing.begin(); outgoing_i < outgoing.end();) {
	float normsti;
	Handle oh = *outgoing_i;
	normsti = getNormSTI(a->getSTI(oh));
	if (normsti > 0.0f) {
	    theSource = oh;
	    foundTheSource = true;
	    outgoing_i = outgoing.erase(outgoing_i);
	} else
	    outgoing_i++;
    }
    if (foundTheSource) {
	outgoing.insert(outgoing.begin(),theSource);
    } else {
	MAIN_LOGGER.log(Util::Logger::ERROR,"Can't find source atom for new Asymmetric Hebbian Link");
    }
    return outgoing;

}

float HebbianLearningAgent::getNormSTI(AttentionValue::sti_t s)
{
    // get normalizer (maxSTI - attention boundary)
    int normaliser = (int) a->getMaxSTI().recent - a->getAttentionalFocusBoundary();
    return (s - a->getAttentionalFocusBoundary()) / (float) normaliser;
}

float HebbianLearningAgent::targetConjunction(std::vector<Handle> handles)
{
    // TODO: this won't work for Hebbian Links with arity > 2

    // indicates whether at least one of the source/target are
    // in the attentional focus
    bool inAttention = false;
    std::vector<Handle>::iterator h_i;
    Handle h;
    AtomSpace *a;
    AttentionValue::sti_t sti;
    float tc = 0.0f, normsti;
    std::vector<float> normsti_v;
    bool tcInit = true;


    for (h_i = handles.begin();
	    h_i != handles.end();
	    h_i++) {
	h = *h_i;
	sti = a->getSTI(h); 

	// if none in attention return 0 at end
	if (sti > a->getAttentionalFocusBoundary()) inAttention = true;

	// normalise each sti and multiple each
	normsti = getNormSTI(sti);

	// For debugging:
	normsti_v.push_back(normsti);

	if (tcInit) {
	    tc = normsti;
	    tcInit = false;
	} else tc *= normsti;

    }

    MAIN_LOGGER.log(Util::Logger::FINE,"TC: normstis [%f,%f]", normsti_v[0],normsti_v[1]);

    if (!inAttention) return 0.0f;
    
    // cap conjunction to range [-1,1]
    if (tc > 1.0f) return 1.0f;
    if (tc < -1.0f) return -1.0f;
    return tc;
	
}

};
