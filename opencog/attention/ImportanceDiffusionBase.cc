/*
 * attention/ImportanceDiffusionBase.h
 *
 * Copyright (C) 2014-2016 Cosmo Harrigan
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
 * written by Misgana Bayetta
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
#include <time.h>
#include <math.h>

#include <opencog/util/algorithm.h>
#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/platform.h>

#include <opencog/atomutils/FollowLink.h>
#include <opencog/atomutils/Neighbors.h>
#include <opencog/atoms/base/Link.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/attention/atom_types.h>
#include <opencog/attentionbank/AttentionBank.h>

#include "ImportanceDiffusionBase.h"
#include "AttentionStat.h"
#include "AttentionUtils.h"

#define DEBUG
#define _unused(x) ((void)x)
#ifdef DEBUG
#undef DEBUG
#endif

using namespace opencog;


ImportanceDiffusionBase::ImportanceDiffusionBase(CogServer& cs) : Agent(cs)
                         ,_atq(&cs.getAtomSpace())
{
    _bank = &attentionbank(_as);

    // Provide a logger
    setLogger(new opencog::Logger("ImportanceDiffusionBase.log",
                                  Logger::FINE, true));
}

/*
 * Allow the maximum diffusion percentage parameter to be varied dynamically by
 * modifying a configuration atom in the atomspace. This method checks for the
 * existence of the configuration atom, and if it exists, updates the parameter
 * to its current value. The value should be a probability between 0 and 1.
 */
void ImportanceDiffusionBase::updateMaxSpreadPercentage()
{
    maxSpreadPercentage = std::stod(_atq.get_param_value(
                AttentionParamQuery::dif_spread_percentage));
}

ImportanceDiffusionBase::~ImportanceDiffusionBase()
{
}

/*
 * Diffuses importance from one atom to its non-hebbian incident atoms
 * and hebbian adjacent atoms
 */
void ImportanceDiffusionBase::diffuseAtom(Handle source)
{
    // (1) Find the incident atoms that will be diffused to
    HandleSeq incidentAtoms =
            ImportanceDiffusionBase::incidentAtoms(source);

    // (2) Find the hebbian adjacent atoms that will be diffused to
    HandleSeq hebbianAdjacentAtoms =
            ImportanceDiffusionBase::hebbianAdjacentAtoms(source);

    // (3) Calculate the probability vector that determines what proportion to
    //     diffuse to each incident atom
    std::map<Handle, double> probabilityVectorIncident =
            ImportanceDiffusionBase::probabilityVectorIncident(
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
            ImportanceDiffusionBase::probabilityVectorHebbianAdjacent(
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

#ifdef LOG_AV_STAT
    // Log sti gain from spreading via  non-hebbian links
    for(const auto& kv : probabilityVectorIncident){
        if(atom_avstat.find(kv.first) == atom_avstat.end()){
            AVStat avstat;
            avstat.link_sti_gain = kv.second;
            atom_avstat[kv.first] = avstat;
        }
        atom_avstat[kv.first].link_sti_gain += kv.second;
    }

    // Log sti gain from spreading via hebbian links
    for(const auto& kv : probabilityVectorHebbianAdjacent){
        if(atom_avstat.find(kv.first) == atom_avstat.end()){
            AVStat avstat;
            avstat.heblink_sti_gain = kv.second;
            atom_avstat[kv.first] = avstat;
        }
        atom_avstat[kv.first].heblink_sti_gain += kv.second;
    }

    // Log amount of sti spread from
    if(atom_avstat.find(source) == atom_avstat.end()){
        AVStat avstat;
        avstat.spreading = totalDiffusionAmount;
        atom_avstat[source] = avstat;
    }
    atom_avstat[source].spreading += totalDiffusionAmount;
#endif

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
    for( const auto& p : probabilityVector)
    {
        DiffusionEventType diffusionEvent;

        // Calculate the diffusion amount using the entry in the probability
        // vector for this particular target (stored in iterator->second)
        diffusionEvent.amount = (AttentionValue::sti_t)
                (totalDiffusionAmount * p.second);

        diffusionEvent.source = source;
        diffusionEvent.target = p.first;

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
void ImportanceDiffusionBase::tradeSTI(DiffusionEventType event)
{
    // Trade STI between the source and target atoms
    _bank->set_sti(event.source, get_sti(event.source) - event.amount);
    _bank->set_sti(event.target, get_sti(event.target) + event.amount);

#ifdef DEBUG
    std::cout << "tradeSTI: " << event.amount << " from " << event.source
              << " to " << event.target << "." << std::endl;
#endif

    // TODO: How to make this a transaction? This could go wrong if there
    // were simultaneous updates in other threads.
}

/*
 * Returns a vector of atom handles that will diffuse STI
 *
 * Calculated as all atoms in the attentional focus (nodes and links)
 * excluding any hebbian links
 */
HandleSeq ImportanceDiffusionBase::diffusionSourceVector(void)
{
    HandleSeq resultSet;
    _bank->get_handle_set_in_attentional_focus(back_inserter(resultSet));

#ifdef DEBUG
    std::cout << "Calculating diffusionSourceVector." << std::endl;
    std::cout << "Source Size before removing hebbian links: " <<
        resultSet.size() << "\n";
#endif

    removeHebbianLinks(resultSet);

#ifdef DEBUG
    std::cout << "Sources Size after removing hebbian links: " <<
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
HandleSeq ImportanceDiffusionBase::incidentAtoms(Handle h)
{
    HandleSeq resultSet;

    // Add the incoming set only found in the present atomspace, because
    // if on another thread the incoming-set is being modified, for example
    // a query that uses transient atomspaces is being processed, we don't
    // want to diffuse to transient atoms created.
    // TODO: How to handle cases when the other atomspaces are not transient
    // but are a child or parent of the present atomspace?
    IncomingSet hIncomingSet = h->getIncomingSet(_as);
    for (const auto& i : hIncomingSet)
    {
        resultSet.push_back(i->get_handle());
    }

    // Calculate and append the outgoing set
    if (h->is_link()) {
        HandleSeq outgoing = h->getOutgoingSet();
        resultSet.insert(resultSet.end(), outgoing.begin(), outgoing.end());
    }

    removeHebbianLinks(resultSet);

    return resultSet;
}

/*
 * Returns a vector of atom handles that are hebbian adjacent to a given atom
 *
 * Calculated as all atoms that are adjacent to the given atom where the type
 * of the connecting edge is a hebbian link
 */
HandleSeq ImportanceDiffusionBase::hebbianAdjacentAtoms(Handle h)
{
    // Chase the hebbian links originating at this atom and obtain the
    // adjacent atoms that are found by traversing those links
    HandleSeq resultSet =
            get_target_neighbors(h, ASYMMETRIC_HEBBIAN_LINK);

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
ImportanceDiffusionBase::probabilityVectorIncident(HandleSeq handles)
{
    std::map<Handle, double> result;

    // Allocate an equal probability to each incident atom
    double diffusionAmount = 1.0 / handles.size();

    for (Handle target : handles)
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
ImportanceDiffusionBase::probabilityVectorHebbianAdjacent(
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
    for (Handle target : targets)
    {
        // Find the hebbian link that connects the source atom to this target
        Handle link = _as->get_handle(ASYMMETRIC_HEBBIAN_LINK, source, target);

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
ImportanceDiffusionBase::combineIncidentAdjacentVectors(
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
    for(const auto& p : adjacentVector)
    {
        // iterator->second is the entry in the probability vector for this
        // handle
        double diffusionAmount =
                hebbianMaximumLinkAllocation * p.second;

        // iterator->first is the handle
        result.insert({p.first, diffusionAmount});

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
    for(const auto& p : incidentVector)
    {
        double diffusionAmount =
                diffusionAvailable * p.second;
        result.insert({p.first, diffusionAmount});

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
double ImportanceDiffusionBase::calculateHebbianDiffusionPercentage(
        Handle h)
{
    strength_t strength = h->getTruthValue()->get_mean();
    confidence_t confidence = h->getTruthValue()->get_confidence();

    return strength * confidence;
}

/*
 * Processes all of the diffusion events that have accumulated in the stack
 *
 * This is where the atom STI updates actually take place.
 */
void ImportanceDiffusionBase::processDiffusionStack()
{

#ifdef DEBUG
    // Keep track of the total amount of STI traded for debugging
    AttentionValue::sti_t totalAmountTraded = 0;
#endif

    while (!diffusionStack.empty())
    {
        DiffusionEventType event = diffusionStack.top();
        ImportanceDiffusionBase::tradeSTI(event);
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

