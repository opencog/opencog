#include "ImportanceUpdatingAgent.h"

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

    /* init iterative variables, that can't be calculated in
     * (no pointer to CogServer there) */
    if (!initialEstimateMade) init(server);

    /* Calculate attentional focus sizes */
    updateAttentionalFocusSizes(a);

    /* Collect rent */
    collectSTIRent(a);
    collectLTIRent(a);

    /* Random stimulation if on */
    if (noiseOn) randomStimulation(a);

    /* Check AtomSpace funds are within bounds */
    checkAtomSpaceFunds(a);

    /* Pay wages based on stimulus */
    paySTIWages(a);
    payLTIWages(a);
    
    /* Enfore sti and lti caps */
    enforceSTICap(a);
    enforceLTICap(a);
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
	fixSTIDynamics(a);
    if (!inRange(a->getTotalLTI(),acceptableLobeLTIRange))
	fixLTIDynamics(a);
}

void ImportanceUpdatingAgent::fixSTIDynamics(AtomSpace* a)
{
    long diff, oldTotal, newTotal;
    AttentionValue::sti_t taxAmount, afterTax;
    HandleEntry* h;
    HandleEntry* q;

    oldTotal = a->getTotalSTI();
    diff = targetLobeSTI - oldTotal;
    h = a->getAtomTable().getHandleSet(ATOM, true);
    taxAmount = diff / h->getSize();

    newTotal = 0;
    q=h;
    while (q) {
	afterTax = a->getSTI(q->handle) - taxAmount;
	a->setSTI(q->handle, afterTax);
	newTotal += afterTax;
	q = q->next;
    }
    delete h;
    
    // TODO: adjust sti rent
    // adjustSTIRent(a);
}

void ImportanceUpdatingAgent::fixLTIDynamics(AtomSpace* a)
{
    long diff, oldTotal, newTotal;
    AttentionValue::lti_t taxAmount, afterTax;
    HandleEntry* h;
    HandleEntry* q;

    oldTotal = a->getTotalLTI();
    diff = targetLobeLTI - oldTotal;
    h = a->getAtomTable().getHandleSet(ATOM, true);
    taxAmount = diff / h->getSize();

    newTotal = 0;
    q=h;
    while (q) {
	afterTax = a->getLTI(q->handle) - taxAmount;
	a->setSTI(q->handle, afterTax);
	newTotal += afterTax;
	q = q->next;
    }
    delete h;
    
    // TODO: adjust sti rent
    // adjustLTIRent(a);
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

void ImportanceUpdatingAgent::collectSTIRent(AtomSpace* a)
{
    /* Iterate through atoms w STI > a->attentionalFocusBoundary */

}

void ImportanceUpdatingAgent::collectLTIRent(AtomSpace* a)
{
    /* Iterate through all atoms and collect LTI */
}

void ImportanceUpdatingAgent::paySTIWages(AtomSpace* a)
{

}

void ImportanceUpdatingAgent::payLTIWages(AtomSpace* a)
{

}

bool ImportanceUpdatingAgent::enforceSTICap(AtomSpace* a)
{
    return false;
}

bool ImportanceUpdatingAgent::enforceLTICap(AtomSpace* a)
{
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
