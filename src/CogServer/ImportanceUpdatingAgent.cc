#include "ImportanceUpdatingAgent.h"
#include <mt19937ar.h>

namespace opencog {

ImportanceUpdatingAgent::ImportanceUpdatingAgent()
{
    /* init starting wages/rents. these should quickly change and reach
     * stable cycles */
    STIAtomRent = DEFAULT_ATOM_STI_RENT;
    LTIAtomRent = DEFAULT_ATOM_LTI_RENT;
    STIAtomWage = DEFAULT_ATOM_STI_WAGE;
    LTIAtomWage = DEFAULT_ATOM_LTI_WAGE;

    updateLinks = true;

    noiseOn = false;
    noiseOdds = 0.20;
    noiseUnit = 10;

    recentTotalStimulusPerCycle = 0;
    recentTotalStimulusSinceReset = 0;
    recentTotalStimulusDecay = 0.3;

    attentionalFocusSize = 0;
    recentAttentionalFocusSize = 0;
    recentAttentionalFocusNodesSize = 0;
    attentionalFocusSizeDecay = 0.3;

    maxSTIDecayRate = 0.8;
    recentMaxSTI = 0;

    targetLobeSTI = 1000;
    acceptableLobeSTIRange[0] = 800;
    acceptableLobeSTIRange[1] = 1200;
    targetLobeLTI = 1000;
    acceptableLobeLTIRange[0] = 800;
    acceptableLobeLTIRange[1] = 1200;

    lobeSTIOutOfBounds = false;

    initialEstimateMade = false;
}

ImportanceUpdatingAgent::~ImportanceUpdatingAgent() {};
		
void ImportanceUpdatingAgent::init(CogServer *server)
{
    /* Not sure exactly what initial estimates should be made... */

}

void ImportanceUpdatingAgent::run(CogServer *server)
{
    AtomSpace* a = server->getAtomSpace();
    HandleEntry *h, *q;

    /* init iterative variables, that can't be calculated in
     * (no pointer to CogServer there) */
    if (!initialEstimateMade) init(server);

    /* Calculate attentional focus sizes */
    updateAttentionalFocusSizes(a);

    /* Check AtomSpace funds are within bounds */
    checkAtomSpaceFunds(a);

    /* Random stimulation if on */
    if (noiseOn) randomStimulation(a);

    /* Update atoms: Collect rent, pay wages */
    h = a->getAtomTable().getHandleSet(ATOM, true);
    q=h;
    while (q) {
	updateAtomSTI(a, q->handle);
	updateAtomLTI(a, q->handle);
    
	/* Enfore sti and lti caps */
	enforceSTICap(a, q->handle);
	enforceLTICap(a, q->handle);

	q = q->next;
    }
    delete h;

    if (lobeSTIOutOfBounds) updateSTIRent(a);

}

bool ImportanceUpdatingAgent::inRange(long val, long range[2]) const
{
    if (val <= range[1] && val >= range[0])
	return true;
    return false;
}

void ImportanceUpdatingAgent::checkAtomSpaceFunds(AtomSpace* a)
{
    if (!inRange(a->getTotalSTI(),acceptableLobeSTIRange))
	lobeSTIOutOfBounds = true;
	adjustSTIFunds(a);
    if (!inRange(a->getTotalLTI(),acceptableLobeLTIRange))
	adjustLTIFunds(a);
}

void ImportanceUpdatingAgent::randomStimulation(AtomSpace* a)
{
    int expectedNum, actualNum;    
    HandleEntry *h, *q;
    Util::RandGen *rng;

    // TODO: Use time or something
    rng = new Util::MT19937RandGen(32423423);

    expectedNum = (int) (noiseOdds * a->getAtomTable().getSize());

    // TODO: use util::lazy_random_selector and a binomial dist
    // to get actualNum
    actualNum = 0;
    h = a->getAtomTable().getHandleSet(ATOM, true);
    q=h;
    while (q) {
	if (rng->randdouble() < noiseOdds)
	    a->stimulateAtom(q, noiseUnit);
	    actualNum++;
	q = q->next;
    }

    delete h;
    delete rng;

}

void ImportanceUpdatingAgent::adjustSTIFunds(AtomSpace* a)
{
    long diff, oldTotal, newTotal;
    AttentionValue::sti_t taxAmount, afterTax;
    HandleEntry* h;
    HandleEntry* q;

    oldTotal = a->getTotalSTI();
    diff = targetLobeSTI - oldTotal;
    h = a->getAtomTable().getHandleSet(ATOM, true);
    taxAmount = diff / a->getAtomTable().getSize();

    newTotal = 0;
    q=h;
    while (q) {
	afterTax = a->getSTI(q->handle) - taxAmount;
	a->setSTI(q->handle, afterTax);
	newTotal += afterTax;
	q = q->next;
    }
    delete h;
    
}

void ImportanceUpdatingAgent::adjustLTIFunds(AtomSpace* a)
{
    long diff, oldTotal, newTotal;
    AttentionValue::lti_t taxAmount, afterTax;
    HandleEntry* h;
    HandleEntry* q;

    oldTotal = a->getTotalLTI();
    diff = targetLobeLTI - oldTotal;
    h = a->getAtomTable().getHandleSet(ATOM, true);
    taxAmount = diff / a->getAtomTable().getSize();

    newTotal = 0;
    q=h;
    while (q) {
	afterTax = a->getLTI(q->handle) - taxAmount;
	a->setSTI(q->handle, afterTax);
	newTotal += afterTax;
	q = q->next;
    }
    delete h;
    
}

void ImportanceUpdatingAgent::updateSTIRent(AtomSpace* a)
{
   lobeSTIOutOfBounds = false; 
}
    

void ImportanceUpdatingAgent::updateAttentionalFocusSizes(AtomSpace* a)
{
    float r = attentionalFocusSizeDecay;
    int n = 0;
    HandleEntry* inFocus;
    HandleEntry* h;

    AtomTable at = a->getAtomTable(); 
    // TODO: implement max and get method of next line
    inFocus = at.getHandleSet(a->getAttentionalFocusBoundary(),AttentionValue::MAXSTI);
    attentionalFocusSize = inFocus->getSize();

    recentAttentionalFocusSize = (long) ( (r * attentionalFocusSize) + \
				 ((1.0-r) * recentAttentionalFocusSize) );
  
    h = inFocus;
    while (h) {
	if (a->isNode(h->getAtom()->getType()))
	    n += 1;
	h = h->next;
    }
    attentionalFocusNodesSize = n;
    recentAttentionalFocusNodesSize = (long) ( (r * attentionalFocusNodesSize) + \
				 ((1.0-r) * recentAttentionalFocusNodesSize) );

}

void ImportanceUpdatingAgent::updateAtomSTI(AtomSpace* a, Handle h)
{
    AttentionValue::sti_t current, stiRentCharged, exchangeAmount;

    current = a->getSTI(h);
    /* collect if STI > a->attentionalFocusBoundary */
    if (current > a->getAttentionalFocusBoundary())
	stiRentCharged = STIAtomRent;
    else
	stiRentCharged = 0;

    exchangeAmount = - stiRentCharged + (STIAtomWage * a->getAtomStimulus(h));
    a->setSTI(h, current + exchangeAmount);

}

void ImportanceUpdatingAgent::updateAtomLTI(AtomSpace* a, Handle h)
{
    /* collect LTI */
    AttentionValue::lti_t current, exchangeAmount;

    current = a->getLTI(h);
    exchangeAmount = - LTIAtomRent + (LTIAtomWage * a->getAtomStimulus(h));
    a->setLTI(h, current + exchangeAmount);

}

bool ImportanceUpdatingAgent::enforceSTICap(AtomSpace* a, Handle h)
{
    AttentionValue::sti_t current, diff;

    current = a->getSTI(h);
    if (current > STICap) {
	diff = current - STICap;
	a->setSTI(h, STICap);
	return true;
    } else if (current < -STICap) {
	diff = -STICap + current;
	a->setSTI(h, -STICap);
	return true;
    }
    return false;
}

bool ImportanceUpdatingAgent::enforceLTICap(AtomSpace* a, Handle h)
{
    AttentionValue::lti_t current, diff;

    current = a->getLTI(h);
    if (current > LTICap) {
	diff = current - LTICap;
	a->setLTI(h, LTICap);
	return true;
    } else if (current < -LTICap) {
	diff = -LTICap + current;
	a->setLTI(h, -LTICap);
	return true;
    }
    return false;
}

string ImportanceUpdatingAgent::toString()
{
    ostringstream s;

    s << "Importance Updating Mind Agent\n";
    s << "STIAtomRent: " << STIAtomRent << "\n";
    s << "STIAtomWage: " << STIAtomWage << "\n";
    s << "LTIAtomRent: " << LTIAtomRent << "\n";
    s << "LTIAtomWage: " << LTIAtomWage << "\n";
    s << "AV Caps (STI/LTI): " << STICap << "/" << LTICap << "\n";
    s << "Updating Links: ";
    if (updateLinks) s <<  "Yes";
    else s << "No";
    s << "\n";
    if (noiseOn)
	s << "Random stimulation on. Chance: " << noiseOdds << \
	    " Amount: " << noiseUnit << "\n";
    s << "Recent Total Stim, per cycle: " << recentTotalStimulusPerCycle \
	<< ", since reset: " << recentTotalStimulusSinceReset \
	<< ", decay: " << recentTotalStimulusDecay << "\n";
    s << "Att. focus. Size: " << attentionalFocusSize << ", recent: " \
	<< recentAttentionalFocusSize << ", recentForNodes: " \
	<< recentAttentionalFocusNodesSize << ", decay: " \
	<< attentionalFocusSizeDecay << "\n"; 
    s << "target (range) STI: " << targetLobeSTI << \
	"(" << acceptableLobeSTIRange[0] << "-" << acceptableLobeSTIRange[1] << \
	") LTI: " << targetLobeLTI << \
	"(" << acceptableLobeLTIRange[0] << "-" << acceptableLobeLTIRange[1] << \
	")\n";

    s.put(0); //null terminate the string cout
    return s.str();

}

} // Namespace opencog
