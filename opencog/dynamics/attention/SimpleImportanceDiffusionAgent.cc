/*
 * opencog/dynamics/attention/SimpleImportanceDiffusionAgent.cc
 *
 * Copyright (C) 2014 Cosmo Harrigan
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
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

#include "SimpleImportanceDiffusionAgent.h"
#include "SpreadDecider.h"
#include <time.h>
#include <math.h>
#include <opencog/atomspace/Link.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/platform.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/algorithm.h>
#include <opencog/atomspace/FollowLink.h>

#define DEBUG
#define _unused(x) ((void)x)

namespace opencog
{

SimpleImportanceDiffusionAgent::SimpleImportanceDiffusionAgent(CogServer& cs) :
    Agent(cs)
{
    /* ======================================================== */
    // Default configuration settings
    
    // ECAN_MAX_SPREAD_PERCENTAGE
    // Maximum percentage of STI that is spread from an atom
    
    // ECAN_SPREAD_HEBBIAN_ONLY
    // If false, will diffuse along hebbian links only. If true,
    // will also diffuse to all non-hebbian incident atoms in the
    // incoming and outgoing sets
    
    // HEBBIAN_MAX_ALLOCATION_PERCENTAGE
    // Maximum percentage that will be available for diffusion to hebbian links
    
    static const std::string defaultConfig[] = {
        "ECAN_MAX_SPREAD_PERCENTAGE", "0.6",
        "ECAN_SPREAD_HEBBIAN_ONLY", "false",
        "HEBBIAN_MAX_ALLOCATION_PERCENTAGE", "0.5",
        "",""
    };
    
    setParameters(defaultConfig);
    /* ======================================================== */
    
    spreadDecider = NULL;
    setSpreadDecider(STEP);
    setMaxSpreadPercentage(
                (float) (config().get_double("ECAN_MAX_SPREAD_PERCENTAGE")));
    setSpreadHebbianOnly(config().get_bool("ECAN_SPREAD_HEBBIAN_ONLY"));
    setHebbianMaxAllocationPercentage(
                config().get_double("HEBBIAN_MAX_ALLOCATION_PERCENTAGE"));

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("SimpleImportanceDiffusionAgent.log", 
                                  Logger::FINE, true));
}

void SimpleImportanceDiffusionAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* SimpleImportanceDiffusionAgent::getLogger()
{
    return log;
}

/*
 * Set the value of the parameter that determines how much of an atom's STI
 * is available for diffusion at each time step. An atom will not diffuse more
 * than this percentage.
 */
void SimpleImportanceDiffusionAgent::setMaxSpreadPercentage(float percent)
{
    maxSpreadPercentage = percent;
}

/*
 * Allow the maximum diffusion percentage parameter to be varied dynamically by
 * modifying a configuration atom in the atomspace. This method checks for the
 * existence of the configuration atom, and if it exists, updates the parameter
 * to its current value. The value should be a probability between 0 and 1.
 */
void SimpleImportanceDiffusionAgent::updateMaxSpreadPercentage() {
    HandleSeq resultSet;
    as->getHandlesByName(back_inserter(resultSet), "CONFIG-DiffusionPercent");
    if (resultSet.size() > 0) {
        // Given the PredicateNode, walk to the NumberNode
        Handle h = resultSet.front();
        resultSet = as->getIncoming(h);
        h = resultSet.front();
        resultSet = as->getOutgoing(h);
        h = resultSet.back();
        resultSet = as->getOutgoing(h);
        h = resultSet.front();
        float value = std::atof(as->getName(h).c_str());
        setMaxSpreadPercentage(value);

#ifdef DEBUG
        std::cout << "Diffusion percentage set to: " <<
                     maxSpreadPercentage << std::endl;
#endif
    }
}

/*
 * Set the value of the parameter that determines how much of an atom's STI 
 * will be allocated for potential diffusion to hebbian links. Note that any 
 * unused STI that was available but unused for diffusion to hebbian links
 * will then be reallocated for diffusion to incident non-hebbian atoms, if
 * that option is enabled.
 */
void SimpleImportanceDiffusionAgent::setHebbianMaxAllocationPercentage(
        float percent)
{
    hebbianMaxAllocationPercentage = percent;
}

/*
 * Set the value of the parameter that determines whether diffusion will occur
 * only along hebbian links (when the parameter is True) or if diffusion will
 * occur both along hebbian links and to non-hebbian incident atoms (when the
 * parameter is False), meaning that all atoms that are in the incoming or
 * outgoing sets will be diffusion targets.
 */
void SimpleImportanceDiffusionAgent::setSpreadHebbianOnly(bool option)
{
    spreadHebbianOnly = option;
}

void SimpleImportanceDiffusionAgent::setSpreadDecider(int type, float shape)
{
    if (spreadDecider) {
        delete spreadDecider;
        spreadDecider = NULL;
    }
    switch (type) {
    case HYPERBOLIC:
        spreadDecider = 
                (SpreadDecider*) new HyperbolicDecider(_cogserver, shape);
        break;
    case STEP:
        spreadDecider = (SpreadDecider*) new StepDecider(_cogserver);
        break;
    }
    
}

SimpleImportanceDiffusionAgent::~SimpleImportanceDiffusionAgent()
{
    if (spreadDecider) {
        delete spreadDecider;
        spreadDecider = NULL;
    }
}

void SimpleImportanceDiffusionAgent::run()
{
    as = &_cogserver.getAtomSpace();
    spreadDecider->setFocusBoundary(0);
    spreadImportance();
}

/*
 * Carries out the importance diffusion process, spreading STI along the
 * graph according to the configuration settings
 */
void SimpleImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq diffusionSourceVector = 
            SimpleImportanceDiffusionAgent::diffusionSourceVector();
    
    // Calculate the diffusion for each source atom, and store the diffusion
    // event in a stack
    foreach (Handle atomSource, diffusionSourceVector) 
    {
        // Check the decision function to determine if spreading will occur
        if (spreadDecider->spreadDecision(as->getSTI(atomSource))) {
#ifdef DEBUG
            std::cout << "Calling diffuseAtom." << std::endl;
#endif
            diffuseAtom(atomSource);
        }
#ifdef DEBUG
        else
        {
            std::cout << "Did not call diffuseAtom." << std::endl;
        }
#endif
    }
   
    // Now, process all of the outstanding diffusion events in the diffusion 
    // stack
    SimpleImportanceDiffusionAgent::processDiffusionStack();
}

/*
 * Diffuses importance from one atom to its non-hebbian incident atoms
 * and hebbian adjacent atoms
 */
void SimpleImportanceDiffusionAgent::diffuseAtom(Handle source)
{
    // (1) Find the incident atoms that will be diffused to
    HandleSeq incidentAtoms = 
            SimpleImportanceDiffusionAgent::incidentAtoms(source);        
    
    // (2) Find the hebbian adjacent atoms that will be diffused to
    HandleSeq hebbianAdjacentAtoms = 
            SimpleImportanceDiffusionAgent::hebbianAdjacentAtoms(source);
        
    // (3) Calculate the probability vector that determines what proportion to 
    //     diffuse to each incident atom
    std::map<Handle, double> probabilityVectorIncident = 
            SimpleImportanceDiffusionAgent::probabilityVectorIncident(
                incidentAtoms);

#ifdef DEBUG
    std::cout << "Calculating diffusion for handle # " << source.value() <<
                 std::endl;
    std::cout << "Incident probability vector contains " << 
                 probabilityVectorIncident.size() << " atoms." << std::endl;
#endif
    
    // (4) Calculate the probability vector that determines what proportion to 
    //     diffuse to each hebbian adjacent atom
    std::map<Handle, double> probabilityVectorHebbianAdjacent = 
            SimpleImportanceDiffusionAgent::probabilityVectorHebbianAdjacent(
                source, hebbianAdjacentAtoms);

#ifdef DEBUG
    std::cout << "Hebbian adjacent probability vector contains " << 
                 probabilityVectorHebbianAdjacent.size() << " atoms." << 
                 std::endl;
#endif
    
    // (5) Combine the two probability vectors into one according to the
    //     configuration parameters
    std::map<Handle, double> probabilityVector = combineIncidentAdjacentVectors(
                probabilityVectorIncident, probabilityVectorHebbianAdjacent);
    
#ifdef DEBUG
    std::cout << "Probability vector contains " << probabilityVector.size() << 
                 " atoms." << std::endl;
#endif
    
    // (6) Calculate the total amount that will be diffused
    AttentionValue::sti_t totalDiffusionAmount = 
            calculateDiffusionAmount(source);
    
#ifdef DEBUG
    std::cout << "Total diffusion amount: " << totalDiffusionAmount << std::endl;
#endif
    
    /* ===================================================================== */
    
    // If there is nothing to diffuse, finish
    if (totalDiffusionAmount == 0)
    {
        return;
    }

    // Perform diffusion from the source to each atom target
    typedef std::map<Handle, double>::iterator it_type;
    for(it_type iterator = probabilityVector.begin(); 
        iterator != probabilityVector.end(); iterator++)
    {
        DiffusionEventType diffusionEvent;
        
        // Calculate the diffusion amount using the entry in the probability 
        // vector for this particular target (stored in iterator->second)
        diffusionEvent.amount = (AttentionValue::sti_t) 
                floor(totalDiffusionAmount * iterator->second);
        
        diffusionEvent.source = source;
        diffusionEvent.target = iterator->first;
        
        // Add the diffusion event to a stack. The diffusion is stored in a 
        // stack, so that all the diffusion events can be processed after the 
        // diffusion calculations are complete. Otherwise, the diffusion 
        // amounts will be calculated in a different way than expected.
        diffusionStack.push(diffusionEvent);
    }

    /* ===================================================================== */
    
    // TODO: What if the STI values of the atoms change during these updates?
    // This could go wrong if there were simultaneous updates in other threads.
    
    // TODO: Support inverse hebbian links
}

/*
 * Trades STI between a source atom and a target atom
 */
void SimpleImportanceDiffusionAgent::tradeSTI(DiffusionEventType event)
{
    // Trade STI between the source and target atoms
    as->setSTI(event.source, as->getSTI(event.source) - event.amount);
    as->setSTI(event.target, as->getSTI(event.target) + event.amount);
    
#ifdef DEBUG
    std::cout << "tradeSTI: " << event.amount << " from " << event.source 
              << " to " << event.target << "." << std::endl;
#endif
    
    // TODO: How to make this a transaction? This could go wrong if there
    // were simultaneous updates in other threads.
    
    // TODO: Using integers for STI values can cause strange consequences.
    // Rounding to an integer is required so that only whole STI amounts
    // are exchanged; due to flooring after multiplying the probability 
    // vector by the total diffusion amount, the amount diffused by this 
    // routine may not exactly match the totalDiffusionAmount, which could 
    // be a problem. Floor is used instead of round, so that an atom cannot
    // diffuse more STI than it has. This also can cause an atom to not
    // diffuse any STI when the amount to be diffused is less than 1.
    //   * See: https://github.com/opencog/opencog/issues/676    
}

/*
 * Returns a vector of atom handles that will diffuse STI
 * 
 * Calculated as all atoms in the attentional focus (nodes and links)
 * excluding any hebbian links
 */
HandleSeq SimpleImportanceDiffusionAgent::diffusionSourceVector()
{
    // Retrieve the atoms in the AttentionalFocus
    HandleSeq resultSet;
    as->getHandleSetInAttentionalFocus(back_inserter(resultSet));

#ifdef DEBUG      
    std::cout << "Calculating diffusionSourceVector." << std::endl;
    std::cout << "AF Size before removing hebbian links: " << 
                 resultSet.size() << "\n";
#endif
    
    // Remove the hebbian links
    auto it_end =
        std::remove_if(resultSet.begin(), resultSet.end(),
                       [=](const Handle& h)
                       { 
                           Type type = as->getType(h);

#ifdef DEBUG                           
                           std::cout << "Checking atom of type: " << 
                                        classserver().getTypeName(type) << "\n";
#endif
                           
                           if (type == ASYMMETRIC_HEBBIAN_LINK ||
                               type == HEBBIAN_LINK ||
                               type == SYMMETRIC_HEBBIAN_LINK ||
                               type == SYMMETRIC_HEBBIAN_LINK ||
                               type == INVERSE_HEBBIAN_LINK ||
                               type == SYMMETRIC_INVERSE_HEBBIAN_LINK)
                           {
#ifdef DEBUG
                               std::cout << "Atom is hebbian" << "\n";
#endif
                               return true;
                           }
                           else
                           {
#ifdef DEBUG
                               std::cout << "Atom is not hebbian" << "\n";
#endif
                               return false;
                           }
                       });
    resultSet.erase(it_end, resultSet.end());
    
#ifdef DEBUG      
    std::cout << "AF Size after removing hebbian links: " << 
    resultSet.size() << "\n";
#endif

    return resultSet;
}

/*
 * Returns a vector of atom handles that are incident to a given atom
 * 
 * Calculated as the set union of an atom's incoming and outgoing set,
 * excluding hebbian links
 */
HandleSeq SimpleImportanceDiffusionAgent::incidentAtoms(Handle h)
{
    HandleSeq resultSet;
    
    // Add the incoming set
    resultSet = as->getIncoming(h);
    
    // Calculate and append the outgoing set
    HandleSeq outgoing = as->getOutgoing(h);
    resultSet.insert(resultSet.end(), outgoing.begin(), outgoing.end());
    
    return resultSet;
}

/*
 * Returns a vector of atom handles that are hebbian adjacent to a given atom
 * 
 * Calculated as all atoms that are adjacent to the given atom where the type
 * of the connecting edge is a hebbian link
 */
HandleSeq SimpleImportanceDiffusionAgent::hebbianAdjacentAtoms(Handle h)
{
    // Chase the hebbian links originating at this atom and obtain the 
    // adjacent atoms that are found by traversing those links
    HandleSeq resultSet = 
            as->getNeighbors(h, false, true, ASYMMETRIC_HEBBIAN_LINK, false);
    
    return resultSet;
}

/*
 * Returns a map of atom handles and probability values
 * 
 * Calculated as the portion of the total STI that will be allocated to each
 * of the incident atoms
 * 
 * TODO: The ideal formula to use here is a subject of current research
 */
std::map<Handle, double> 
SimpleImportanceDiffusionAgent::probabilityVectorIncident(HandleSeq handles)
{
    std::map<Handle, double> result;
    
    // Allocate an equal probability to each incident atom
    double diffusionAmount = 1.0f / handles.size();
    
    foreach (Handle target, handles)
    {
        result.insert({target, diffusionAmount});
    }
    
    return result;
}

/*
 * Returns a map of atom handles and probability values
 * 
 * Calculated as the portion of the total STI that will be allocated to each
 * of the hebbian adjacent atoms
 * 
 * The specifics of the algorithm are a subject of current research
 */
std::map<Handle, double> 
SimpleImportanceDiffusionAgent::probabilityVectorHebbianAdjacent(
        Handle source, HandleSeq targets)
{
    std::map<Handle, double> result;
    
    // Start with 100% of possible diffusion, and then allocate it
    double diffusionAvailable = 1.0;
    
    // Calculate out how many hebbian adjacent atoms there are in total
    int atomCount = targets.size();
    
    // Calculate the maximum amount that could be allocated to each adjacent 
    // atom, if each hebbian link divided up the total available amount equally
    double maxAllocation = diffusionAvailable / atomCount;
    
    // For each hebbian link that will be spread across, discount the 
    // amount that is actually allocated to it, based on certain attributes 
    // of the link
    foreach (Handle target, targets)
    {
        // Find the hebbian link that connects the source atom to this target
        HandleSeq searchAtoms(2);
        searchAtoms[0] = source;
        searchAtoms[1] = target;
        HandleSeq resultList;
        as->getHandlesByOutgoing(back_inserter(resultList), searchAtoms, NULL, 
                                 NULL, 2, ASYMMETRIC_HEBBIAN_LINK, false);
        Handle link = resultList.front();
        
        // Calculate the discounted diffusion amount based on the link
        // attributes
        double diffusionAmount = 
                maxAllocation * calculateHebbianDiffusionPercentage(link);    
    
        // Insert the diffusionAmount into the map
        result.insert({target, diffusionAmount});
    }
    
    return result;
}

/*
 * Returns a map of atom handles and probability values
 * 
 * Calculated as the portion of the total STI that will be allocated to each
 * of the atoms.
 * 
 * Two probability vectors are combined in this function in order to return
 * a single, unified probability vector. This allocates a portion of the 
 * available STI to the vector containing incident atoms (excluding hebbian
 * links), and a portion of the STI to the vector containing hebbian adjacent
 * atoms.
 * 
 * The specifics of the algorithm are a subject of current research
 */
std::map<Handle, double> 
SimpleImportanceDiffusionAgent::combineIncidentAdjacentVectors(
        std::map<Handle, double> incidentVector, 
        std::map<Handle, double> adjacentVector)
{
    std::map<Handle, double> result;
    
    // Start with 100% of possible diffusion, and then allocate it
    double diffusionAvailable = 1.0;
    
    // Calculate the maximum proportion that could be used for diffusion to
    // all hebbian adjacent atoms
    double hebbianDiffusionAvailable = 
            hebbianMaxAllocationPercentage * diffusionAvailable;
    
    // Calculate the maximum proportion that could be allocated to any
    // particular hebbian adjacent atom, as the total amount available for
    // allocation to all hebbian adjacent atoms, divided by the count of
    // hebbian adjacent atoms
    double hebbianMaximumLinkAllocation = 
            hebbianDiffusionAvailable / adjacentVector.size();
    
    // Keep track of how much diffusion has been allocated to hebbian adjacent
    // atoms
    double hebbianDiffusionUsed = 0.0;
    
    // For each hebbian adjacent target, allocate a proportion of STI according
    // to the probability vector for the target, and the proportion that is
    // available for allocation to any particular individual atom
    typedef std::map<Handle, double>::iterator it_type;
    for(it_type iterator = adjacentVector.begin(); 
        iterator != adjacentVector.end(); iterator++)
    {
        // iterator->second is the entry in the probability vector for this
        // handle
        double diffusionAmount = 
                hebbianMaximumLinkAllocation * iterator->second;
        
        // iterator->first is the handle
        result.insert({iterator->first, diffusionAmount});
        
        // Decrement the amount of remaining diffusion that is available
        hebbianDiffusionUsed += diffusionAmount;
    }
    
    // There is likely unused diffusion remaining from the hebbian diffusion 
    // process, if some of the links did not diffuse fully due to their 
    // attributes. Subtract what diffusion was used to determine how much is
    // still available.
    diffusionAvailable -= hebbianDiffusionUsed;
    
    // Keep track of how much diffusion has been allocated to incident atoms
    double incidentDiffusionUsed = 0.0;
    
    // Allocate the remaining diffusion amount to the incident atoms according
    // to the probability vector for the targets
    for(it_type iterator = incidentVector.begin(); 
        iterator != incidentVector.end(); iterator++)
    {
        double diffusionAmount = 
                diffusionAvailable * iterator->second;
        result.insert({iterator->first, diffusionAmount});
        
        incidentDiffusionUsed += diffusionAmount;
    }
    
#ifdef DEBUG
    // Confirm that the probability vector sums to 1.0
    double totalDiffused = incidentDiffusionUsed + hebbianDiffusionUsed;
    double tolerance = 0.001;
    assert (totalDiffused > 1 - tolerance && totalDiffused < 1 + tolerance);
    _unused(totalDiffused);
    _unused(tolerance);
#endif
    
    return result;
}

/*
 * Returns the total amount of STI that the atom will diffuse
 * 
 * Calculated as the maximum spread percentage multiplied by the atom's STI
 */
AttentionValue::sti_t SimpleImportanceDiffusionAgent::calculateDiffusionAmount(
        Handle h)
{
    updateMaxSpreadPercentage();

    return (AttentionValue::sti_t) round(as->getSTI(h) * maxSpreadPercentage);
    
    // TODO: Using integers for STI values can cause strange consequences.
    // For example, if the amount to diffuse is 0.4, it will become 0, causing
    // no diffusion to occur.
    //   * See: https://github.com/opencog/opencog/issues/676
}

/*
 * Returns a percentage, which should be multiplied by the amount of STI
 * that is available to spread across a given hebbian link, to determine the 
 * amount of STI that should actually be spread across the given hebbian link
 * 
 * For example, this allows a calculation that uses the strength or confidence
 * of a TruthValue to make a hebbian link less "conductive".
 * 
 * TODO: The ideal formula to use here is a subject of current research
 * 
 * In the current implementation, by returning the strength times the 
 * confidence, if either value is far from 1.0, then the link will allow less 
 * diffusion to occur across the hebbian link
 */
float SimpleImportanceDiffusionAgent::calculateHebbianDiffusionPercentage(
        Handle h)
{
    strength_t strength = as->getMean(h);
    confidence_t confidence = as->getConfidence(h);
    
    return strength * confidence;
}

/*
 * Processes all of the diffusion events that have accumulated in the stack
 * 
 * This is where the atom STI updates actually take place.
 */
void SimpleImportanceDiffusionAgent::processDiffusionStack()
{
#ifdef DEBUG
    // Keep track of the total amount of STI traded for debugging
    AttentionValue::sti_t totalAmountTraded = 0;
#endif
    
    while (!diffusionStack.empty())
    {
        DiffusionEventType event = diffusionStack.top();
        SimpleImportanceDiffusionAgent::tradeSTI(event);
        diffusionStack.pop();
        
#ifdef DEBUG
        totalAmountTraded += event.amount;
#endif
    }
    
#ifdef DEBUG    
    // Each trade occurs bidirectionally. Therefore, if you add up all the
    // trades, it should be equal to twice the amount that was diffused
    std::cout << "Total STI traded: " << totalAmountTraded << std::endl;
#endif
}

} // namespace
