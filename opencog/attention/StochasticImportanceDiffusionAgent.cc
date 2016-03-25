/*
 * opencog/attention/ImportanceDiffusionAgent.cc
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
#include <opencog/attention/atom_types.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cogserver/server/CogServer.h>

#include "ImportanceDiffusionAgent.h"

#define DEBUG
namespace opencog
{

ImportanceDiffusionAgent::ImportanceDiffusionAgent(CogServer& cs) :
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
    setLogger(new opencog::Logger("ImportanceDiffusionAgent.log", Logger::FINE, true));
}

void ImportanceDiffusionAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* ImportanceDiffusionAgent::getLogger()
{
    return log;
}

void ImportanceDiffusionAgent::setSpreadDecider(int type, float shape)
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

ImportanceDiffusionAgent::~ImportanceDiffusionAgent()
{
    if (spreadDecider) {
        delete spreadDecider;
        spreadDecider = NULL;
    }
}

void ImportanceDiffusionAgent::setMaxSpreadPercentage(float p)
{ maxSpreadPercentage = p; }

float ImportanceDiffusionAgent::getMaxSpreadPercentage() const
{ return maxSpreadPercentage; }

void ImportanceDiffusionAgent::setDiffusionThreshold(float p)
{
    diffusionThreshold = p;
}

float ImportanceDiffusionAgent::getDiffusionThreshold() const
{ return diffusionThreshold; }

void ImportanceDiffusionAgent::run()
{
    a = &_cogserver.getAtomSpace();
    spreadDecider->setFocusBoundary(diffusionThreshold);
#ifdef DEBUG
    totalSTI = 0;
#endif
    spreadImportance();
}

#define toFloat getMean

void ImportanceDiffusionAgent::spreadImportance()
{
    std::back_insert_iterator< std::vector<Handle> > out_hi(links);

    log->debug("Begin diffusive importance spread.");

    a.get_handle_set_in_attentional_focus(out_hi);

    std::size_type size = links.size();

    index = rand() % size;

    Handle atom = links[index];

    outset = atom.getIncomingSetByType
}

void ImportanceDiffusionAgent::setScaledSTI(Handle h, float scaledSTI)
{
    AttentionValue::sti_t val;

    val = (AttentionValue::sti_t) (a->get_min_STI(false) + (scaledSTI * ( a->get_max_STI(false) - a->get_min_STI(false) )));
/*
    AtomSpace *a = _cogserver.getAtomSpace();
    float af = a->getAttentionalFocusBoundary();
    scaledSTI = (scaledSTI * 2) - 1;
    if (scaledSTI <= 1.0) {
        val = a->getMinSTI(false) + (scaledSTI * ( a->getMinSTI(false) - af ));
    } else {
        val = af + (scaledSTI * (a->getMaxSTI(false) - af ));
    }
*/
    a->set_STI(h,val);
}


};

