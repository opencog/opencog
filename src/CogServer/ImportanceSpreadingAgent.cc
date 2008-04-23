#include "ImportanceSpreadingAgent.h"

#include <CogServer.h>
#include <Link.h>

namespace opencog {

ImportanceSpreadingAgent::ImportanceSpreadingAgent()
{
    spreadThreshold = MA_DEFAULT_SPREAD_THRESHOLD;
    importanceSpreadingMultiplier = MA_DEFAULT_SPREAD_MULTIPLIER;

}

ImportanceSpreadingAgent::~ImportanceSpreadingAgent()
{

}

void ImportanceSpreadingAgent::run(CogServer* server)
{
    a = server->getAtomSpace();
    spreadImportance();

}


void ImportanceSpreadingAgent::spreadImportance()
{

    AttentionValue::sti_t current;

    std::vector<Handle> atoms;
    std::vector<Handle>::iterator hi;
    std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);
    
    a->getHandleSet(out_hi,NODE,true);
    MAIN_LOGGER.log(Util::Logger::FINE, "---------- Spreading importance for atoms with threshold above %d", spreadThreshold);

    hi = atoms.begin();
    while (hi != atoms.end()) {
	Handle h = *hi;

	current = a->getSTI(h);
	/* spread if STI > spread threshold */
	if (current > spreadThreshold )
	    // spread fraction of importance to nodes it's linked to
	    spreadAtomImportance(h);
	
	hi++;
    }

    

}

void ImportanceSpreadingAgent::spreadAtomImportance(Handle h)
{
    HandleEntry *links, *he;
    float maxTransferAmount,totalRelatedness;
    int totalTransferred;
    float importanceSpreadingFactor = 0.4;
    AttentionValue::sti_t minStealingBoundary;

    totalRelatedness = 0.0f;
    totalTransferred = 0;

    MAIN_LOGGER.log(Util::Logger::FINE, "+Spreading importance for atom %s", TLB::getAtom(h)->toString().c_str());

    links = TLB::getAtom(h)->getIncomingSet()->clone();
    links = HandleEntry::filterSet(links, HEBBIAN_LINK, true);
    MAIN_LOGGER.log(Util::Logger::FINE, "  +Hebbian links found %d", links->getSize());
    
    maxTransferAmount = a->getSTI(h) * importanceSpreadingFactor;
    minStealingBoundary = a->getAttentionalFocusBoundary() - (2 * abs(a->getAttentionalFocusBoundary()));

    // sum total relatedness
    he = links;
    while (he) {
	if (((Link*)TLB::getAtom(he->handle))->isSource(h)) {
	    float val;
	    val = fabs(a->getTV(he->handle).toFloat());
	    totalRelatedness += val;
	}
	he = he->next;
    }

    if (totalRelatedness > 0.0f) {
	std::vector<Handle> linksVector;
	std::vector<Handle>::iterator linksVector_i;

	// Find order of links based on their STI
	// I.e. links with higher STI are more likely to get sti passed along
	// them before available sti for spreading runs out.
	links->toHandleVector(linksVector);
	std::sort(linksVector.begin(), linksVector.end(), ImportanceSpreadSTISort());

	for (linksVector_i = linksVector.begin();
		linksVector_i != linksVector.end() &&
		totalTransferred <= maxTransferAmount; linksVector_i++) {
	    double transferWeight, transferAmount;
	    std::vector<Handle> targets;
	    std::vector<Handle>::iterator t;
	    Handle lh = *linksVector_i;
	    const TruthValue &linkTV = a->getTV(lh);

	    if (!((Link*)TLB::getAtom(lh))->isSource(h)) {
		//MAIN_LOGGER.log(Util::Logger::FINE, "Link %s does not have this atom as a source.", TLB::getAtom(lh)->toString().c_str() );
		continue; 
	    }

	    targets = TLB::getAtom(lh)->getOutgoingSet();
	    transferWeight = linkTV.toFloat();

	    // amount to spread dependent on weight and multiplier - needs to be
	    // divided by (targets->size - 1)
	    if (importanceSpreadingMultiplier == 0.0f) {
		transferAmount = transferWeight * (maxTransferAmount / totalRelatedness);
	    } else
		transferAmount = transferWeight * importanceSpreadingMultiplier;

	    // Allow at least one hebbian link to spread importance, even if it's
	    // less than the amount the weight would indicate
	    if (transferAmount > maxTransferAmount)
		transferAmount = maxTransferAmount;
	    
	    if (targets.size() != 2)
		transferAmount = transferAmount / (targets.size()-1.0f);
	    if (transferAmount == 0.0f) continue;

	    MAIN_LOGGER.log(Util::Logger::FINE, "  +Link %s", TLB::getAtom(lh)->toString().c_str() );
	    MAIN_LOGGER.log(Util::Logger::FINE, "    |weight %f, quanta %.2f, size %d, Transfer amount %f, maxTransfer %f", transferWeight, importanceSpreadingMultiplier, targets.size(), transferAmount, maxTransferAmount);

	    for (t = targets.begin();
		    t != targets.end() &&
		    totalTransferred + transferAmount <= maxTransferAmount;
		    t++) {
		Handle target_h = *t;

		// Then for each target of link (except source)...
		if ( TLB::getAtom(target_h) == TLB::getAtom(h) )
		    continue;

		// Check removing STI doesn't take node out of attentional
		// focus...
		// TODO: precalculate this in loop conditional
		if (a->getSTI(h) >= a->getAttentionalFocusBoundary() && \
		    a->getSTI(h) - transferAmount < a->getAttentionalFocusBoundary())
		    break;

		// Check that if the amount is negative, it doesn't steal too
		// much from the atom
		if (transferAmount < 0.0f) {
		    if (a->getSTI(target_h) - (AttentionValue::sti_t) transferAmount \
			    < minStealingBoundary)
			continue;
		}
		totalTransferred += (int) transferAmount;
		a->setSTI( h, a->getSTI(h) - (AttentionValue::sti_t) transferAmount );
		a->setSTI( target_h, a->getSTI(target_h) + (AttentionValue::sti_t) transferAmount );
		MAIN_LOGGER.log(Util::Logger::FINE, "    |%d sti from %s to %s", (int) transferAmount, TLB::getAtom(h)->toString().c_str(), TLB::getAtom(target_h)->toString().c_str() );
		
		// stimulate link 
		// Doesn't make sense to stimulate the links just for spread.
		// Maybe stimulate links that are helpful in getting the right
		// pattern.
		//if ( agent->getUpdateLinksFlag() )
		//    a->stimulateAtom(lh,stimForSpread);
	    }


	}
    }
    else {
	MAIN_LOGGER.log(Util::Logger::FINE, "  |Total relatedness = 0, spreading nothing");
    }
    delete links;

}

};

