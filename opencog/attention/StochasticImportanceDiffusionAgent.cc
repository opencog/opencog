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

StochasticImportanceDiffusionAgent::~StochasticImportanceDiffusionAgent()
{
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
  //HandleSeq atoms;
  //std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);

  //log->debug("Begin diffusive importance spread.");
  //a->get_handles_by_AV(out_hi,diffusionThreshold);

    HandleSeq atoms;
    a->get_all_atoms(atoms);

    atoms.erase(remove_if(atoms.begin(),atoms.end(),
                    [](Handle h) -> bool {
                    return classserver().isA(h->getType(),HEBBIAN_LINK);
                    }),atoms.end());

    size_t size = atoms.size();

    if (size == 0)
        return;

    int index = rand() % size;

    Handle source = atoms[index];

    HandleSeq targetSet =
            get_target_neighbors(source, ASYMMETRIC_HEBBIAN_LINK);
  //HandleSeq links;
  //std::back_insert_iterator< std::vector<Handle> > out_hi2(links);
  //source->getIncomingSetByType(out_hi2,ASYMMETRIC_HEBBIAN_LINK);

    if (targetSet.size() == 0)
        return;

    float totalAvailableDiffusionAmmount = a->get_STI(source) * maxSpreadPercentage;

    float availableDiffusionAmmount = totalAvailableDiffusionAmmount / targetSet.size();

    float factor = 1;

    std::map<Handle,float> tvs;

    for (Handle target : targetSet)
    {
        Handle link = a->get_handle(ASYMMETRIC_HEBBIAN_LINK, source, target);

        if (link == Handle::UNDEFINED)
            continue;

        float tv = link->getTruthValue()->getMean();
        factor += tv;
        tvs[target] = tv;
    }

    int totalam = 0;

    for (Handle target : targetSet)
    {
        std::map<Handle,float>::iterator it = tvs.find(target);

        if (it == tvs.end())
            continue;

        float tv = it->second * 2 - 1;
        int ssti = a->get_STI(source);
        int tsti = a->get_STI(target);

        //int diffusionAmmount = floor(((ssti - tsti) * tv) / factor);
        int diffusionAmmount = floor(availableDiffusionAmmount * tv);
        totalam +=diffusionAmmount;


        if (diffusionAmmount > 0) {
            a->set_STI(source, ssti - diffusionAmmount);
            a->set_STI(target, tsti + diffusionAmmount);
        }
    }
    //printf("diffusionAmmount: %d \n",totalam);
}
};
