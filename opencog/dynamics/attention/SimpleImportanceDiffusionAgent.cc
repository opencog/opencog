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

#define DEBUG
namespace opencog
{

SimpleImportanceDiffusionAgent::SimpleImportanceDiffusionAgent(CogServer& cs) :
    Agent(cs)
{
    static const std::string defaultConfig[] = {
        //! Maximum percentage of STI that is spread from an atom
        "ECAN_MAX_SPREAD_PERCENTAGE","0.5",
        "ECAN_SPREAD_HEBBIAN_ONLY","false",
        "",""
    };
    
    setParameters(defaultConfig);
    spreadDecider = NULL;
    maxSpreadPercentage = 
            (float) (config().get_double("ECAN_MAX_SPREAD_PERCENTAGE"));
    setSpreadDecider(STEP);
    spreadHebbianOnly = config().get_bool("ECAN_SPREAD_HEBBIAN_ONLY");

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

void SimpleImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq diffusionSourceVector = 
            SimpleImportanceDiffusionAgent::diffusionSourceVector();
    
    foreach (Handle atomSource, diffusionSourceVector) 
    {
        // Check the decision function to determine if spreading will occur
        if (spreadDecider->spreadDecision(as->getSTI(atomSource))) {
            diffuseAtom(atomSource);
        }
    }
}

/*
 * Diffuses importance from one atom to its non-hebbian incident atoms
 * and hebbian adjacent atoms
 */
void SimpleImportanceDiffusionAgent::diffuseAtom(Handle atomSource)
{
    // TODO: What if the STI values of the atoms change during these updates?
    // This could go wrong if there were simultaneous updates in other threads.
    
    HandleSeq targetAtoms = 
            SimpleImportanceDiffusionAgent::incidentAtoms(atomSource);        
    
    HandleSeq hebbianAdjacentAtoms = 
            SimpleImportanceDiffusionAgent::hebbianAdjacentAtoms(atomSource);
    
    targetAtoms.insert(targetAtoms.end(), 
                       hebbianAdjacentAtoms.begin(), 
                       hebbianAdjacentAtoms.end());
    
    std::map<Handle, double> probabilityVector = 
            SimpleImportanceDiffusionAgent::probabilityVector(targetAtoms);
    
    AttentionValue::sti_t diffusionAmount = 
            SimpleImportanceDiffusionAgent::calculateDiffusionAmount(atomSource);
    
    if (diffusionAmount == 0)
    {
        return;
    }

    // For each atomTarget in incident atoms
    foreach (Handle atomTarget, targetAtoms)
    {       
        diffusionAmount = diffusionAmount * probabilityVector[atomTarget];   
        
        // Trade STI between the source and target atoms
        as->setSTI(atomSource, as->getSTI(atomSource) - diffusionAmount);
        as->setSTI(atomTarget, as->getSTI(atomTarget) + diffusionAmount);
        
        // TODO: How to make this a transaction? This could go wrong if there
        // were simultaneous updates in other threads.
    }
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
    
    // Remove the hebbian links
    resultSet.erase(
        std::remove_if(resultSet.begin(), resultSet.end(),
                       [=](const Handle& h)
                       { 
                           Type type = as->getType(h);
                           
                           if (type == ASYMMETRIC_HEBBIAN_LINK ||
                               type == HEBBIAN_LINK ||
                               type == SYMMETRIC_HEBBIAN_LINK ||
                               type == SYMMETRIC_HEBBIAN_LINK ||
                               type == INVERSE_HEBBIAN_LINK ||
                               type == SYMMETRIC_INVERSE_HEBBIAN_LINK)
                           {
                               return true;
                           }
                       }));
    
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
    // Use FollowLink::follow_link?   
}

/*
 * Returns a map of atom handles and probability values
 * 
 * Calculated as the portion of the total STI that will be allocated to each
 * of the atoms
 */
std::map<Handle, double> SimpleImportanceDiffusionAgent::probabilityVector(
        HandleSeq handles)
{
    
}

/*
 * Returns the total amount of STI that the atom will diffuse
 * 
 * Calculated as the maximum spread percentage multiplied by the atom's STI
 */
AttentionValue::sti_t SimpleImportanceDiffusionAgent::calculateDiffusionAmount(
        Handle h)
{
    
}

} // namespace
