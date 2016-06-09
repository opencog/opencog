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
#include <thread>
#include <chrono>

#include <opencog/util/Config.h>
#include <opencog/util/platform.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/atoms/base/Link.h>
#include <opencog/atomutils/Neighbors.h>
#include <opencog/attention/atom_types.h>
#include <opencog/truthvalue/SimpleTruthValue.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cogserver/server/CogServer.h>

#include "StochasticImportanceDiffusionAgent.h"

#define DEBUG

using namespace std;
using namespace chrono;

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

    a = &_cogserver.getAtomSpace();
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

#define toFloat getMean

void StochasticImportanceDiffusionAgent::run()
{
    std::thread producer(&StochasticImportanceDiffusionAgent::produce,this);
    consume();
    producer.join(); //Will wait forever as these threads will run indefinitelly
}

void StochasticImportanceDiffusionAgent::produce()
{
    std::default_random_engine generator;

    while (true) {
        count++;
        if (count == 100 || atomcount == 0) {
            count = 0;
            a->get_all_atoms(atoms);

            atoms.erase(remove_if(atoms.begin(),atoms.end(),
                            [](Handle h) -> bool {
                            bool isHebbian = classserver().isA(h->getType(),HEBBIAN_LINK);
                            bool lowSTI = h->getSTI() < 3;
                            return (isHebbian or lowSTI);
                            }),atoms.end());

            atomcount = atoms.size();

            if (atomcount == 0) {
                //printf("no atoms found. \n");
                continue;
            }
        }

        HandleSeq targetSet;
        int idx;

        std::uniform_int_distribution<int> distribution(0,atomcount-1);

        idx = distribution(generator);

        targetSet = get_target_neighbors(atoms[idx], ASYMMETRIC_HEBBIAN_LINK);

        if (targetSet.size() == 0)
            continue;

        short startSTI = a->get_STI(atoms[idx]);
        float totalAvailableDiffusionAmmount = (float)startSTI * maxSpreadPercentage;

        if (totalAvailableDiffusionAmmount < 1.0f) {
            //printf("Total to low.\n");
            continue;
        }

        if (targetSet.size() > floor(totalAvailableDiffusionAmmount)) {
            unsigned int end = targetSet.size() - floor(totalAvailableDiffusionAmmount);

            if (targetSet.size() == end) {
                //printf("Not enought STI.\n");
                continue;
            }

            //printf("targets: %zd end: %d totaladm: %d",targetSet.size(), end, totalAvailableDiffusionAmmount);

            std::sort(targetSet.begin(), targetSet.end(),
                    [](Handle a, Handle b) -> bool {
                        return a->getTruthValue()->getMean() < a->getTruthValue()->getMean();
                    }
                    );
            targetSet.erase(targetSet.begin(), targetSet.begin() + end);
            //printf("new targets: %zd",targetSet.size());
        }

        while (!queue.push(std::pair<Handle,HandleSeq>(atoms[idx],targetSet))) {
            std::this_thread::yield();
        }
    }
}

void StochasticImportanceDiffusionAgent::consume()
{
    std::pair<Handle,HandleSeq> p;
    Handle source;
    HandleSeq targetSet;
    while (true) {
        if (!queue.pop(p)) {
            std::this_thread::yield();
            continue;
        }

        source = p.first;
        targetSet = p.second;

      //printf("Source: %s,targetSet.size(): %d ",source->toString().c_str(),targetSet.size());

        float totalAvailableDiffusionAmmount = source->getSTI();

        float availableDiffusionAmmount = totalAvailableDiffusionAmmount / (float)targetSet.size();

        //Check if there is at least 1 STI for each HebbianLink
        //This might have changed since the producer checked
        if (availableDiffusionAmmount < 1.0f) {
            continue;
        }

        int total = 0;

        for (Handle target : targetSet)
        {
            Handle link = a->get_handle(ASYMMETRIC_HEBBIAN_LINK, source, target);
            if (link == Handle::UNDEFINED) {
                continue;
            }

            //Normalize to [-1,1] so values <0.5 act like a Inverse Link
            float tv = link->getTruthValue()->getMean();
            tv = tv * 2.0f - 1.0f;

            int ssti = a->get_STI(source);
            int tsti = a->get_STI(target);

            float fdiffusionAmmount = availableDiffusionAmmount * tv;
            int diffusionAmmount = 0;

            //If we want to spread less then 1 STI use the value instead
            //as a probability to spread 1 STI
            if (fdiffusionAmmount < 1 && fdiffusionAmmount > 0) {
                if ((rand() % 100) > (100 * fdiffusionAmmount)) {
                    diffusionAmmount = 1;
                }
            } else if (fdiffusionAmmount > -1 && fdiffusionAmmount < 0) {
                if ((rand() % 100) > (100 * fdiffusionAmmount)) {
                    diffusionAmmount = -1;
                }
            } else {
                diffusionAmmount = round(fdiffusionAmmount);
            }

            if (abs(diffusionAmmount) < 1)
                continue;

            //If we have a Inverse Connection
            //Make sure not to take more STI then the Target has
            if (tsti + diffusionAmmount < 0)
                diffusionAmmount = -tsti;

            total += diffusionAmmount;

            source->setSTI(ssti - diffusionAmmount);
            target->setSTI(tsti + diffusionAmmount);
        }

      //printf("Total Available STI: %f Available per Link: %f Diffused a Total of: %d \n",totalAvailableDiffusionAmmount,availableDiffusionAmmount,total);
    }
}

};
