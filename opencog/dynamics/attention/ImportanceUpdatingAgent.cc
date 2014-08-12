/*
 * opencog/dynamics/attention/ImportanceUpdatingAgent.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * Written by Joel Pitt <joel@fruitionnz.com>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "ImportanceUpdatingAgent.h"

#include <algorithm>
#include <math.h>
#include <time.h>

#include <opencog/util/Config.h>
#include <opencog/util/foreach.h>
#include <opencog/util/mt19937ar.h>

#define DEBUG

using namespace opencog;

ImportanceUpdatingAgent::ImportanceUpdatingAgent(CogServer& cs) :
    Agent(cs)
{
    // Starting values for rent and wage
    static const std::string defaultConfig[] = {
        "ECAN_STARTING_ATOM_STI_RENT", "10",
        "ECAN_STARTING_ATOM_LTI_RENT", "0", // TEMPORARILY SET TO 0
        "ECAN_STARTING_ATOM_STI_WAGE", "2",
        "ECAN_STARTING_ATOM_LTI_WAGE", "2",
        "ECAN_RENT_TYPE","2", // RENT_LOG
        "ECAN_RENT_AMNESTY","5",
        "ECAN_RENT_EQUATION_PARAMETER_0","0.05",
        "ECAN_RENT_EQUATION_PARAMETER_1","0.0",
        "", ""
    };
    setParameters(defaultConfig);

    // init starting wages/rents. these should quickly change and reach
    // stable values, which adapt to the system dynamics
    STIAtomRent = config().get_int("ECAN_STARTING_ATOM_STI_RENT");
    STITransitionalAtomRent = STIAtomRent;
    LTIAtomRent = config().get_int("ECAN_STARTING_ATOM_LTI_RENT");
    STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_WAGE");
    LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_WAGE");

	setRentType( (rentType_t) config().get_int("ECAN_RENT_TYPE") );
    // For RENT_LOG:
    // percentAmnesty = rentFunctionParams[0];
    // multiplier = rentFunctionParams[1];
    // For RENT_EXP:
    // y = rentFunctionParams[0];
    // multiplier = rentFunctionParams[1];
    // Parameters not used by RENT_FLAT
    rentFunctionParams.push_back(config().get_double("ECAN_RENT_EQUATION_PARAMETER_0"));
    rentFunctionParams.push_back(config().get_double("ECAN_RENT_EQUATION_PARAMETER_1"));

	setAmnesty( (AttentionValue::sti_t) config().get_int("ECAN_RENT_AMNESTY") );

    updateLinks = true;

    noiseOn = false;
    noiseOdds = 0.20f;
    noiseUnit = 10;

    // set decay rates for dampened values
    STITransitionalAtomRent.decay = 0.5f;
    totalStimulusSinceReset.decay = 0.5f;
    attentionalFocusSize.decay = 0.8f;
    attentionalFocusNodesSize.decay = 0.8f;

    targetLobeSTI = config().get_int("STARTING_STI_FUNDS");
    acceptableLobeSTIRange[0] = targetLobeSTI - config().get_int("STI_FUNDS_BUFFER");
    acceptableLobeSTIRange[1] = targetLobeSTI + config().get_int("STI_FUNDS_BUFFER");
    targetLobeLTI = config().get_int("STARTING_LTI_FUNDS");
    acceptableLobeLTIRange[0] = targetLobeLTI - config().get_int("LTI_FUNDS_BUFFER");
    acceptableLobeLTIRange[1] = targetLobeLTI + config().get_int("LTI_FUNDS_BUFFER");

    lobeSTIOutOfBounds = false;
    lobeLTIOutOfBounds = false;

    STICap = AttentionValue::MAXSTI / 2;
    LTICap = AttentionValue::MAXLTI / 2;

    initialEstimateMade = false;

    rng = NULL;

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("ImportanceUpdatingAgent.log", Logger::FINE, true));
}

ImportanceUpdatingAgent::~ImportanceUpdatingAgent()
{
    if (log) delete log;
    if (rng) delete rng;
}

void ImportanceUpdatingAgent::init()
{
    // Not sure exactly what initial estimates should be made...
    log->fine("ImportanceUpdatingAgent::init");
    initialEstimateMade = true;
	// Perhaps initiate recent_val members to initial
	// size before the mind process begins.
}

void ImportanceUpdatingAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* ImportanceUpdatingAgent::getLogger()
{
    return log;
}

void ImportanceUpdatingAgent::getHandlesToUpdate(AtomSpace *a, HandleSeq &hs)
{
    if (updateLinks)
        a->getHandlesByType(back_inserter(hs), ATOM, true);
    else
        a->getHandlesByType(back_inserter(hs), NODE, true);
}

void ImportanceUpdatingAgent::calculateAtomWages(AtomSpace *a, const AgentSeq &agents)
{
    STIAtomWageForAgent.clear();
    LTIAtomWageForAgent.clear();
    for (size_t n = 0; n < agents.size(); n++) {
        // cout << agents[n]->to_string() << ": ";
        updateAgentSTI(a, agents[n]);
        updateAgentLTI(a, agents[n]);

        if (agents[n]->getTotalStimulus() == 0)
        {
           // cout << "no stimulus" << endl;

            STIAtomWageForAgent.push_back(0.0f);
            LTIAtomWageForAgent.push_back(0.0f);
            continue;
        }
        //cout << "STI " << (float) a->getSTI(agents[n]);
        //cout << " stim " << agents[n]->getTotalStimulus() << endl;

        STIAtomWageForAgent.push_back((float)
            agents[n]->getAV()->getSTI() /
                                      agents[n]->getTotalStimulus());
        LTIAtomWageForAgent.push_back((float)
            agents[n]->getAV()->getLTI() /
                                      agents[n]->getTotalStimulus());
    }
}

void ImportanceUpdatingAgent::run()
{
    AgentSeq agents = _cogserver.runningAgents();
    AtomSpace* a = &_cogserver.getAtomSpace();
    HandleSeq hs;
    AttentionValue::sti_t maxSTISeen = AttentionValue::MINSTI;
    AttentionValue::sti_t minSTISeen = AttentionValue::MAXSTI;

    log->fine("=========== ImportanceUpdatingAgent::run =======");
    /* init iterative variables, that can't be calculated in
     * (no pointer to CogServer there) */
    if (!initialEstimateMade) init();

    /* Calculate attentional focus sizes */
    updateAttentionalFocusSizes(a);

    /* Random stimulation if on */
    if (noiseOn) {
        log->debug("Random stimulation on, stimulating atoms");
        for (size_t n = 0; n < agents.size(); n++)
            randomStimulation(a, agents[n]);
    }

    /* Update stimulus totals */
    updateTotalStimulus(agents);

    /* Update atoms: Collect rent, pay wages */
    log->info("Collecting rent and paying wages");

    getHandlesToUpdate(a,hs);

    /* Calculate STI/LTI atom wages for each agent */
    calculateAtomWages(a, agents);

    foreach(Handle handle, hs) {
        updateAtomSTI(a, agents, handle);
        updateAtomLTI(a, agents, handle);

        /* Enfore sti and lti caps */
        enforceSTICap(a, handle);
        enforceLTICap(a, handle);

        // Greater than max sti seen?
        if (a->getSTI(handle) > maxSTISeen) {
            maxSTISeen = a->getSTI(handle);
		} else if (a->getSTI(handle) < minSTISeen) {
            minSTISeen = a->getSTI(handle);
		}
    }

    // Update AtomSpace recent maxSTI and recent minSTI
	if (minSTISeen > maxSTISeen) {
		// if all Atoms have the same STI this will occur
		minSTISeen = maxSTISeen;
	}

    a->updateMaxSTI(maxSTISeen);
    a->updateMinSTI(minSTISeen);
    log->debug("Max STI seen is %d, recentMaxSTI is now %d", maxSTISeen, a->getMaxSTI());
    log->debug("Min STI seen is %d, recentMinSTI is now %d", minSTISeen, a->getMinSTI());

    /* Check AtomSpace funds are within bounds */
    checkAtomSpaceFunds(a);

    if (lobeSTIOutOfBounds) {
        log->debug("Lobe STI was out of bounds, updating STI rent");
        updateSTIRent(a);
    }
    /* Not sure whether LTI rent should be updated */
    //if (lobeLTIOutOfBounds) {
    //    log->debug("Lobe LTI was out of bounds, updating LTI rent");
    //    updateLTIRent(a);
    //}

    /* Reset Stimulus */
    for (size_t n = 0; n < agents.size(); n++)
        agents[n]->resetStimulus();
}

void ImportanceUpdatingAgent::updateTotalStimulus(const AgentSeq &agents)
{
    int total = 0;
    for (size_t n = 0; n < agents.size(); n++)
        total += agents[n]->getTotalStimulus();
    totalStimulusSinceReset.update(total);
}

void ImportanceUpdatingAgent::setNoiseFlag(bool newVal)
{
    noiseOn = newVal;
}

bool ImportanceUpdatingAgent::inRange(long val, long range[2]) const
{
    if (val <= range[1] && val >= range[0])
        return true;
    return false;
}

bool ImportanceUpdatingAgent::checkAtomSpaceFunds(AtomSpace* a)
{
	bool adjustmentMade = false;

    log->debug("Checking STI funds = %d, range=[%d,%d]", a->getAttentionBank().getSTIFunds(),
               acceptableLobeSTIRange[0], acceptableLobeSTIRange[1]);
    if (!inRange(a->getAttentionBank().getSTIFunds(), acceptableLobeSTIRange)) {
        log->debug("Lobe STI funds out of bounds, re-adjusting.");
        lobeSTIOutOfBounds = true;
        adjustSTIFunds(a);
		adjustmentMade = true;
    }

    log->debug("Checking LTI funds = %d, range=[%d,%d]", a->getAttentionBank().getLTIFunds(),
               acceptableLobeLTIRange[0], acceptableLobeLTIRange[1]);
    if (!inRange(a->getAttentionBank().getLTIFunds(), acceptableLobeLTIRange)) {
        log->debug("Lobe LTI funds out of bounds, re-adjusting.");
        lobeLTIOutOfBounds = true;
        adjustLTIFunds(a);
		adjustmentMade = true;
    }
	return adjustmentMade;
}

opencog::RandGen* ImportanceUpdatingAgent::getRandGen()
{
    if (!rng) {
        rng = new opencog::MT19937RandGen((unsigned long) time(NULL));
    }
    return rng;
}

void ImportanceUpdatingAgent::randomStimulation(AtomSpace *a, AgentPtr agent)
{
    int expectedNum, actualNum;
    HandleSeq hs;
    opencog::RandGen *rng;

    rng = getRandGen();

    // TODO: use util::lazy_random_selector and a binomial dist
    // to get actualNum
    actualNum = 0;

    getHandlesToUpdate(a,hs);
    expectedNum = (int) (noiseOdds * hs.size());

    foreach( Handle h, hs) {
        double r;
        r = rng->randdouble();
        if (r < noiseOdds) {
            agent->stimulateAtom(h, noiseUnit);
            actualNum++;
        }
    }

    log->info("Applied stimulation randomly to %d " \
              "atoms, expected about %d.", actualNum, expectedNum);

}

void ImportanceUpdatingAgent::adjustSTIFunds(AtomSpace* a)
{
    long diff, oldTotal;
    AttentionValue::sti_t afterTax, beforeTax;
    double taxAmount;
    HandleSeq hs;

    oldTotal = a->getAttentionBank().getSTIFunds();
    diff = targetLobeSTI - oldTotal;
    getHandlesToUpdate(a,hs);
    taxAmount = (double) diff / (double) hs.size();

    foreach(Handle handle, hs) {
        int actualTax;
        actualTax = getTaxAmount(taxAmount);
        beforeTax = a->getSTI(handle);
        afterTax = beforeTax - actualTax;
               
        a->setSTI(handle, afterTax);
        log->fine("sti %d. Actual tax %d. after tax %d.", beforeTax, actualTax, afterTax);

#ifdef DEBUG        
        std::cout << "Atom " << handle.value() << " STI " << beforeTax << 
                     ". Tax " << actualTax << ". After tax " << afterTax << 
                     "." << std::endl;
#endif
    }

    log->info("AtomSpace STI Funds were %d, now %d. All atoms taxed %f.", \
              oldTotal, a->getAttentionBank().getSTIFunds(), taxAmount);
    
#ifdef DEBUG
    std::cout << "AtomSpace STI Funds were " << oldTotal << ", now " <<
                 a->getAttentionBank().getSTIFunds() << ". All atoms taxed " <<
                 taxAmount << "." << std::endl;
#endif
}

void ImportanceUpdatingAgent::adjustLTIFunds(AtomSpace* a)
{
    long diff, oldTotal;
    AttentionValue::lti_t afterTax;
    double taxAmount;
    HandleSeq hs;

    oldTotal = a->getAttentionBank().getLTIFunds();
    diff = targetLobeLTI - oldTotal;
    getHandlesToUpdate(a,hs);

    taxAmount = (double) diff / (double) hs.size();

    foreach(Handle handle, hs) {
        afterTax = a->getLTI(handle) - getTaxAmount(taxAmount);
        a->setLTI(handle, afterTax);
    }

    log->info("AtomSpace LTI Funds were %d, now %d. All atoms taxed %.2f.", \
              oldTotal, a->getAttentionBank().getLTIFunds(), taxAmount);
}

int ImportanceUpdatingAgent::getTaxAmount(double mean)
{
    double sum, prob, p;
    int count = 0;
    int base;
    bool negative = false;

    if (mean < 0.0) {
        negative = true;
        mean = -mean;
    }
    base = (int) mean;
    mean = mean - base;
    // Calculates tax amount by sampling a Poisson distribution
    p = getRandGen()->randDoubleOneExcluded();
    prob = sum = exp(-mean);

    while (p > sum) {
        count++;
        prob = (prob * mean) / count;
        sum += prob;
    }
    count = count + base;

    if (negative) count = -count;

    return count;
}

void ImportanceUpdatingAgent::updateSTIRent(AtomSpace* a, bool gradual)
{
    AttentionValue::sti_t oldSTIAtomRent;
    float focusSize = 0;

    // STIAtomRent must be adapted based on attentional focus size, or else balance btw
    // lobe STI wealth and node/link STI wealth may not be maintained

    oldSTIAtomRent = STIAtomRent;

    if (!updateLinks) {
        if (attentionalFocusNodesSize.recent > 0)
            STITransitionalAtomRent.update((AttentionValue::sti_t) ceil((float) STIAtomWage * (float) totalStimulusSinceReset.recent \
                          / (float) attentionalFocusNodesSize.recent));
        //else
        //    STIAtomRent = (AttentionValue::sti_t)ceil((float) STIAtomWage * (float) totalStimulusSinceReset.recent);

        focusSize = attentionalFocusNodesSize.recent;

    } else {
        if (attentionalFocusSize.recent > 0)
            STITransitionalAtomRent.update((AttentionValue::sti_t)ceil((float) STIAtomWage * (float) totalStimulusSinceReset.recent \
                          / (float) attentionalFocusSize.recent));
        //else
        //    STIAtomRent = (AttentionValue::sti_t)ceil((float) STIAtomWage * (float) totalStimulusSinceReset.recent);

        focusSize = attentionalFocusSize.recent;
    }

    if(gradual)
        STIAtomRent = STITransitionalAtomRent.recent;
    else
        STIAtomRent = STITransitionalAtomRent.val;

    log->fine("STIAtomRent was %d, now %d. Focus size was %.2f. Wage is %d. Total stim was %.2f.", oldSTIAtomRent, STIAtomRent, focusSize, STIAtomWage, totalStimulusSinceReset.recent);

    lobeSTIOutOfBounds = inRange(a->getAttentionBank().getSTIFunds(), acceptableLobeSTIRange);
}

void ImportanceUpdatingAgent::updateLTIRent(AtomSpace* a)
{
    AttentionValue::lti_t oldLTIAtomRent;

    float focusSize = a->getNumNodes();

    // LTIAtomRent must be adapted based on total AtomSpace size, or else balance btw
    // lobe LTI wealth and node/link LTI wealth will not be maintained

    oldLTIAtomRent = LTIAtomRent;

    if (!updateLinks) {
        if (focusSize > 0)
            LTIAtomRent = (AttentionValue::sti_t) ceil((float) LTIAtomWage \
                    * (float) totalStimulusSinceReset.recent \
                    / (float) focusSize);
    } else {
        focusSize += a->getNumLinks();
        if (focusSize > 0)
            LTIAtomRent = (AttentionValue::sti_t)ceil((float) LTIAtomWage \
                    * (float) totalStimulusSinceReset.recent \
                    / (float) focusSize);
    }

    log->fine("LTIAtomRent was %d, now %d. Focus size was %.2f. Wage is %d. Total stim was %.2f.", oldLTIAtomRent, LTIAtomRent, focusSize, LTIAtomWage, totalStimulusSinceReset.recent);

    lobeLTIOutOfBounds = false;
}


void ImportanceUpdatingAgent::updateAttentionalFocusSizes(AtomSpace* a)
{
    int n = 0;
    HandleSeq inFocus;

    AttentionValue::sti_t threshold = a->getAttentionalFocusBoundary() + amnesty;
    a->getHandlesByAV(back_inserter(inFocus), threshold);

    attentionalFocusSize.update(inFocus.size());

    log->fine("attentionalFocusSize = %d, recent = %f",
              attentionalFocusSize.val, attentionalFocusSize.recent);

    foreach(Handle h, inFocus) {
        if (a->isNode(h)) n += 1;
    }
    attentionalFocusNodesSize.update(n);

    log->fine("attentionalFocusNodesSize = %d, recent = %f",
              attentionalFocusNodesSize.val, attentionalFocusNodesSize.recent);
}

void ImportanceUpdatingAgent::updateAgentSTI(AtomSpace* a, AgentPtr agent)
{
    AttentionValue::sti_t current = agent->getAV()->getSTI();

    /* TODO
     *
     * The wikibook says:
     *
     *    Currency flows from Units to MindAgents when Units reward MindAgents
     *    for helping achieve system goals.
     *
     *    Currency flows from MindAgents to Units when MindAgents pay Units
     *    rent for the processor time they utilize.
     */
    AttentionValue::sti_t exchangeAmount = 0;

    // just top up to a fixed amount for now
    if (current < STIAtomWage * 100)
        exchangeAmount = STIAtomWage * 100 - current;

    a->getAttentionBank().updateSTIFunds(-exchangeAmount);

    AttentionValuePtr old_av = agent->getAV();
    AttentionValuePtr new_av = createAV(current + exchangeAmount,
                                        old_av->getLTI(),
                                        old_av->getVLTI());
    agent->setAV(new_av);
}

void ImportanceUpdatingAgent::updateAgentLTI(AtomSpace* a, AgentPtr agent)
{
    AttentionValue::lti_t current = agent->getAV()->getLTI();

    // TODO: see above
    AttentionValue::lti_t exchangeAmount = 0;

    // just top up to a fixed amount for now
    if (current < LTIAtomWage * 100)
        exchangeAmount = LTIAtomWage * 100 - current;

    a->getAttentionBank().updateLTIFunds(-exchangeAmount);

    AttentionValuePtr old_av = agent->getAV();
    AttentionValuePtr new_av = createAV(old_av->getSTI(),
                                        current + exchangeAmount,
                                        old_av->getVLTI());
    agent->setAV(new_av);
}

void ImportanceUpdatingAgent::updateAtomSTI(AtomSpace* a, const AgentSeq &agents, Handle h)
{
    // Check for changes to the rent and wage parameters
    updateRentAndWages(a);
    
    AttentionValue::sti_t current = a->getSTI(h);
    AttentionValue::sti_t stiRentCharged = calculateSTIRent(a, current);

    int total_stim = 0;
    AttentionValue::sti_t exchangeAmount = -stiRentCharged;

    for (size_t n = 0; n < agents.size(); n++) {
        if (agents[n]->getTotalStimulus() == 0)
        {
            continue;
        }
        stim_t s = agents[n]->getAtomStimulus(h);
        total_stim += s;
        float wage = STIAtomWageForAgent[n];
        if (wage > STIAtomWage)
            wage = (float) STIAtomWage;
        exchangeAmount += (AttentionValue::sti_t) wage * s;
        
        a->getAttentionBank().updateSTIFunds(exchangeAmount);

        AttentionValuePtr old_av = agents[n]->getAV();
        AttentionValuePtr new_av = createAV(current - exchangeAmount,
                                            old_av->getLTI(),
                                            old_av->getVLTI());
        agents[n]->setAV(new_av);
    }
    a->setSTI(h, current + exchangeAmount);

    log->fine("Atom %s total stim = %d, STI old = %d, new = %d, rent = %d", a->getName(h).c_str(), total_stim, current, a->getSTI(h), stiRentCharged);
    
#ifdef DEBUG
    if (stiRentCharged != 0 || exchangeAmount != 0 || total_stim != 0)
    {
        std::cout << "Atom " << h.value() << " total stimulus = " << total_stim << 
                     " STI old = " << current << ", new = " << a->getSTI(h) << 
                     ", rent = " << stiRentCharged << 
                     ", exchangeAmount = " << exchangeAmount << std::endl;
    }
#endif
}

AttentionValue::sti_t ImportanceUpdatingAgent::calculateSTIRent(AtomSpace* a, AttentionValue::sti_t c)
{
    AttentionValue::sti_t stiRentCharged = 0;

	switch (rentType) {
		case RENT_FLAT:
			// Charge a flat rent to all atoms with
			// STI > AF boundary + amnesty
			if (c > a->getAttentionalFocusBoundary() + amnesty)
				stiRentCharged = STIAtomRent;
			break;
		case RENT_EXP:
			// ipython: plot(x, [max(0,i) for i in (exp(x)-(1-y))/(1+y)])
			if (c > a->getAttentionalFocusBoundary() + amnesty) {
				double y = rentFunctionParams[0];
				double multiplier = rentFunctionParams[1];
				double x;
				x = c - a->getAttentionalFocusBoundary();
				x = x / (a->getMaxSTI() - a->getAttentionalFocusBoundary());
				multiplier = std::max(0.0, (exp(x) - (1.0 - y))/(1.0 + y));
				stiRentCharged = (AttentionValue::sti_t) (multiplier * STIAtomRent);
			}
			break;
		case RENT_LOG:
			// max(0,i) where i = log((x-(amnesty%-0.05))*20)/2])
			// ipython: plot(x, [max(0,i) for i in log((x)*20)/2])
			if (c > a->getAttentionalFocusBoundary()) {
				double percentAmnesty = rentFunctionParams[0];
				double multiplier = rentFunctionParams[1];
				double x;
				percentAmnesty = percentAmnesty-0.05;
				x = c - a->getAttentionalFocusBoundary();
				x = x / (a->getMaxSTI() - a->getAttentionalFocusBoundary());
				if (percentAmnesty < 0) percentAmnesty = 0;
				multiplier = std::max(0.0, ::log((x - percentAmnesty) * 20) / 2.0);
				stiRentCharged = (AttentionValue::sti_t) (multiplier * STIAtomRent);
			}
			break;
	}
    
    // Do not charge rent in excess of an atom's STI, so that STI does not go
    // below zero
    if (stiRentCharged > c) {
        stiRentCharged = c;
    }
    
	return stiRentCharged;

}

void ImportanceUpdatingAgent::updateAtomLTI(AtomSpace* a, const AgentSeq &agents, Handle h)
{
    /* collect LTI */
    AttentionValue::lti_t current = a->getLTI(h);
    AttentionValue::lti_t exchangeAmount = -LTIAtomRent;
    for (size_t n = 0; n < agents.size(); n++) {
        if (agents[n]->getTotalStimulus() == 0)
            continue;
        stim_t s = agents[n]->getAtomStimulus(h);
        float wage = LTIAtomWageForAgent[n];
        if (wage > LTIAtomWage)
            wage = (float) LTIAtomWage;
        exchangeAmount += (AttentionValue::lti_t) (wage * s);

        a->getAttentionBank().updateLTIFunds(exchangeAmount);

        AttentionValuePtr old_av = agents[n]->getAV();
        AttentionValuePtr new_av = createAV(old_av->getSTI(),
                                            current - exchangeAmount,
                                            old_av->getVLTI());
        agents[n]->setAV(new_av);
    }
    a->setLTI(h, current + exchangeAmount);

    log->fine("Atom %s LTI old = %d, new = %d", a->getName(h).c_str(), current, a->getLTI(h));
}

bool ImportanceUpdatingAgent::enforceSTICap(AtomSpace* a, Handle h)
{
    AttentionValue::sti_t current;

    current = a->getSTI(h);
    if (current > STICap) {
        a->setSTI(h, STICap);
        log->fine("Atom STI too high - old = %d, new = %d", current, a->getSTI(h));
        return true;
    } else if (current < -STICap) {
        a->setSTI(h, -STICap);
        log->fine("Atom STI too low - old = %d, new = %d", current, a->getSTI(h));
        return true;
    }
    return false;
}

bool ImportanceUpdatingAgent::enforceLTICap(AtomSpace* a, Handle h)
{
    AttentionValue::lti_t current;

    current = a->getLTI(h);
    if (current > LTICap) {
        a->setLTI(h, LTICap);
        log->fine("Atom LTI too high - old = %d, new = %d", current, a->getSTI(h));
        return true;
    } else if (current < -LTICap) {
        a->setLTI(h, -LTICap);
        log->fine("Atom LTI too low - old = %d, new = %d", current, a->getSTI(h));
        return true;
    }
    return false;
}

void ImportanceUpdatingAgent::setUpdateLinksFlag(bool f)
{
    updateLinks = f;
}

bool ImportanceUpdatingAgent::getUpdateLinksFlag() const
{
    return updateLinks;
}

std::string ImportanceUpdatingAgent::toString()
{
    std::ostringstream s;

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
    s << "Recent Total Stim since reset: " << totalStimulusSinceReset.recent \
    << ", decay: " << totalStimulusSinceReset.decay << "\n";
    s << "Att. focus. Size: " << attentionalFocusSize.val << ", recent: " \
    << attentionalFocusSize.recent << ", recentForNodes: " \
    << attentionalFocusNodesSize.val << ", decay: " \
    << attentionalFocusSize.decay << "\n";
    s << "target (range) STI: " << targetLobeSTI << \
    "(" << acceptableLobeSTIRange[0] << "-" << acceptableLobeSTIRange[1] << \
    ") LTI: " << targetLobeLTI << \
    "(" << acceptableLobeLTIRange[0] << "-" << acceptableLobeLTIRange[1] << \
    ")\n";

    s.put(0); //null terminate the string cout
    return s.str();
}

/*
 * Allow the atom rent and wage parameters to be varied dynamically by modifying
 * a configuration atom in the atomspace. This method checks for the existence
 * of the configuration atom, and if it exists, updates the parameter to its
 * current value. The value should be an integer between 0 and MAXSTI.
 */
void ImportanceUpdatingAgent::updateRentAndWages(AtomSpace* a) {
    // Update rent
    HandleSeq wage;
    a->getHandlesByName(back_inserter(wage), "CONFIG-Rent");
    if (wage.size() > 0) {
        // Given the PredicateNode, walk to the NumberNode
        Handle h = wage.front();
        wage = a->getIncoming(h);
        h = wage.front();
        wage = a->getOutgoing(h);
        h = wage.back();
        wage = a->getOutgoing(h);
        h = wage.front();
        int value = std::stoi(a->getName(h));
        
        if (STIAtomRent != value) {
#ifdef DEBUG
            std::cout << "Rent parameter set to: " << value << 
                         " [previous value: " << STIAtomRent << 
                         "]" << std::endl;
#endif
            STIAtomRent = value;
        }
    }
    
    // Update wages
    HandleSeq rent;
    a->getHandlesByName(back_inserter(rent), "CONFIG-Wages");
    if (rent.size() > 0) {
        // Given the PredicateNode, walk to the NumberNode
        Handle h = rent.front();
        rent = a->getIncoming(h);
        h = rent.front();
        rent = a->getOutgoing(h);
        h = rent.back();
        rent = a->getOutgoing(h);
        h = rent.front();
        int value = std::stoi(a->getName(h));

        if (STIAtomWage != value) {      
#ifdef DEBUG
        std::cout << "Wage parameter set to: " << value << 
                     " [previous value: " << STIAtomRent << 
                     "]" << std::endl;
#endif
            STIAtomWage = value;
        }
    }
}

//AttentionValue::sti_t ImportanceUpdatingAgent::getSTIAtomWage()
//{
//	return STIAtomWage;
//}

//AttentionValue::lti_t ImportanceUpdatingAgent::getLTIAtomWage()
//{
//	return LTIAtomWage;
//}
