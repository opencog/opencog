/*
 * opencog/attention/StochasticImportanceDiffusionAgent.cc
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

#include <time.h>
#include <math.h>

#include <opencog/util/Config.h>
#include <opencog/util/platform.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/atoms/base/Link.h>
#include <opencog/atomutils/Neighbors.h>
#include <opencog/attention/atom_types.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cogserver/server/CogServer.h>

#include "StochasticImportanceDiffusionAgent.h"

#define DEBUG
namespace opencog
{

StochasticImportanceDiffusionAgent::StochasticImportanceDiffusionAgent(CogServer& cs) :
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

    //! @todo won't respond to the parameters being changed later
    //! (not a problem at present, but could get awkward with, for example,
    //! automatic parameter adaptation)
    maxSpreadPercentage = (float) (config().get_double("ECAN_MAX_SPREAD_PERCENTAGE"));

    setDiffusionThreshold((float) (config().get_double("ECAN_DIFFUSION_THRESHOLD")));

    allLinksSpread = config().get_bool("ECAN_ALL_LINKS_SPREAD");

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("StochasticImportanceDiffusionAgent.log", Logger::FINE, true));
}

void StochasticImportanceDiffusionAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* StochasticImportanceDiffusionAgent::getLogger()
{
    return log;
}

void StochasticImportanceDiffusionAgent::setMaxSpreadPercentage(float p)
{ maxSpreadPercentage = p; }

float StochasticImportanceDiffusionAgent::getMaxSpreadPercentage() const
{ return maxSpreadPercentage; }

void StochasticImportanceDiffusionAgent::setDiffusionThreshold(float p)
{
    diffusionThreshold = p;
}

float StochasticImportanceDiffusionAgent::getDiffusionThreshold() const
{ return diffusionThreshold; }

void StochasticImportanceDiffusionAgent::run()
{
    a = &_cogserver.getAtomSpace();
#ifdef DEBUG
    totalSTI = 0;
#endif
    spreadImportance();
}

#define toFloat getMean

void StochasticImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq links;
    std::back_insert_iterator< std::vector<Handle> > out_hi(links);

    log->debug("Begin diffusive importance spread.");

    a->get_handles_by_AV(out_hi,diffusionThreshold);

    size_t size = links.size();

    int index = rand() % size;

    Handle source = links[index];

    HandleSeq targetSet =
            get_target_neighbors(source, ASYMMETRIC_HEBBIAN_LINK);

    float totalAvailableDiffusionAmmount = a->get_STI(source) * maxSpreadPercentage;

    float availableDiffusionAmmount = totalAvailableDiffusionAmmount / targetSet.size();

    for (Handle target : targetSet)
    {
        Handle link = a->get_handle(ASYMMETRIC_HEBBIAN_LINK, source, target);

        float diffusionAmmount = availableDiffusionAmmount * link->getTruthValue()->getMean();

        a->set_STI(source, a->get_STI(source) - diffusionAmmount);
        a->set_STI(target, a->get_STI(target) + diffusionAmmount);
    }
}
};
