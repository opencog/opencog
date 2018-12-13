/*
 * opencog/attention/ImprintAgent.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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
#include "ImprintAgent.h"

#include <iomanip>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/truthvalue/SimpleTruthValue.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

#include "Pattern.h"
#include "HopfieldServer.h"
#include "HopfieldOptions.h"

#define DEBUG

using namespace opencog;

ImprintAgent::ImprintAgent(CogServer& cs) : Agent(cs), epsilon(Pattern(0,0))
{
    static const std::vector<std::string> defaultConfig = {
//        "ECAN_CONVERT_LINKS","false",
        "",""
    };
    setParameters(defaultConfig);

//    convertLinks = config().get_bool("ECAN_CONVERT_LINKS");

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("ImprintAgent.log", Logger::WARN, true));
}

ImprintAgent::~ImprintAgent()
{
    if (log) delete log;
}

void ImprintAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* ImprintAgent::getLogger()
{
    return log;
}

void ImprintAgent::setPattern(Pattern _epsilon)
{
    epsilon = _epsilon;
}

void ImprintAgent::run()
{
    HopfieldServer* hs = dynamic_cast<HopfieldServer*>(&_cogserver);
    // AtomSpace& a = hs->getAtomSpace();
    //a.getNormalisedSTI((static_cast<HopfieldServer&>(server())).hGrid[j],false);
    int n = hs->width * hs->height;
    assert(n == epsilon.size());
    stim_t stimulusAmount = 1;
    if (epsilon.activity() > 0) {
        stimulusAmount = getAV()->getSTI() / epsilon.activity();
        if (stimulusAmount == 0) stimulusAmount++;
    }
    for (int i = 0; i < n; i++) {
        if (hs->options->keyNodes && hs->hGridKey[i]) continue; // Don't encode onto key nodes
        stimulateAtom( hs->hGrid[i], stimulusAmount * epsilon[i] );
    }
}
