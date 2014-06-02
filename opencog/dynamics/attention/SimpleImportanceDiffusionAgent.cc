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

#define DEBUG
namespace opencog
{

SimpleImportanceDiffusionAgent::SimpleImportanceDiffusionAgent(CogServer& cs) :
    Agent(cs)
{
    static const std::string defaultConfig[] = {
        //! Default value that normalised STI has to be above before
        //! being spread
        "ECAN_DIFFUSION_THRESHOLD","0.0",
        //! Maximum percentage of STI that is spread from an atom
        "ECAN_MAX_SPREAD_PERCENTAGE","1.0",
        "ECAN_ALL_LINKS_SPREAD","false",
        "",""
    };
    setParameters(defaultConfig);
    spreadDecider = NULL;

    //! @todo won't respond to the parameters being changed later
    //! (not a problem at present, but could get awkward with, for example,
    //! automatic parameter adaptation)
    maxSpreadPercentage = (float) (config().get_double("ECAN_MAX_SPREAD_PERCENTAGE"));

    setSpreadDecider(STEP);
    setDiffusionThreshold((float) (config().get_double("ECAN_DIFFUSION_THRESHOLD")));

    allLinksSpread = config().get_bool("ECAN_ALL_LINKS_SPREAD");

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("SimpleImportanceDiffusionAgent.log", Logger::FINE, true));
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
        spreadDecider = (SpreadDecider*) new HyperbolicDecider(_cogserver, shape);
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

void SimpleImportanceDiffusionAgent::setMaxSpreadPercentage(float p)
{ maxSpreadPercentage = p; }

float SimpleImportanceDiffusionAgent::getMaxSpreadPercentage() const
{ return maxSpreadPercentage; }

void SimpleImportanceDiffusionAgent::setDiffusionThreshold(float p)
{
    diffusionThreshold = p;
}

float SimpleImportanceDiffusionAgent::getDiffusionThreshold() const
{ return diffusionThreshold; }

void SimpleImportanceDiffusionAgent::run()
{
    a = &_cogserver.getAtomSpace();
    spreadDecider->setFocusBoundary(diffusionThreshold);
#ifdef DEBUG
    totalSTI = 0;
#endif
    spreadImportance();
}

#define toFloat getMean

void SimpleImportanceDiffusionAgent::spreadImportance()
{
    // For each atomSource in AttentionalFocus that is not a hebbian link
    
    // Check the decision function to determine if spreading will occur
    // If no, continue (to the next iteration)
    
    // Construct a vector of incident atoms
    // (consisting of the set union of the incoming and outgoing sets)
    
    // Add the hebbian adjacent atoms to the incident atoms vector
    // (atoms that are adjacent, connected by a hebbian link)
    
    // Construct a probability vector containing the incident atoms, which 
    // determines how to allocate the STI that will be diffused 
    // (stored as a map, associating each atom with a probability)
    
    // Find the total amount of STI that atomSource will diffuse
    
    // For each atomTarget in incident atoms
    
    // Set the diffusion amount to the associated entry in the probability 
    // vector multiplied by the total amount to diffuse
    
    // Update the STI of the target atom and the source atom
}

} // namespace
