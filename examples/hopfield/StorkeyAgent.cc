/*
 * opencog/dynamics/attention/StorkeyAgent.cc
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
#include "StorkeyAgent.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>

#include "HopfieldServer.h"

#define DEBUG

using namespace opencog;

StorkeyAgent::StorkeyAgent()
{
    static const std::string defaultConfig[] = {
        "ECAN_CONVERT_LINKS","false",
        "",""
    };
    setParameters(defaultConfig);

    convertLinks = config().get_bool("ECAN_CONVERT_LINKS");

    // Provide a logger, but disable it initially
    log = NULL;
    setLogger(new opencog::Logger("StorkeyAgent.log", Logger::WARN, true));
    log->disable();
}

StorkeyAgent::~StorkeyAgent()
{
    if (log) delete log;
}

void StorkeyAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* StorkeyAgent::getLogger()
{
    return log;
}

//void StorkeyAgent::setPattern(Pattern _epsilon)
//{ epsilon = _epsilon; }

void StorkeyAgent::run(CogServer *server)
{
    a = server->getAtomSpace();
    storkeyUpdate();
}

float StorkeyAgent::h(int i, w_t w)
{
    int n = w.size();
    float h_sum = 0.0f;
    for (int k=0; k < n; k++) {
        h_sum += ( w[i][k] * a->getNormalisedSTI(
                    (static_cast<HopfieldServer&>(server())).hGrid[k],false) );
    }
    return h_sum;
}

StorkeyAgent::w_t StorkeyAgent::getCurrentWeights()
{
    int n = (static_cast<HopfieldServer&>(server())).hGrid.size();
    std::vector<std::vector<float> > w(n);
    for (int i=0; i < n; i++) {
        std::vector<float> iRow(n);
        Handle iHandle = (static_cast<HopfieldServer&>(server())).hGrid[i];
        for (int j=0; j < n; j++) {
            if (i==j) iRow.push_back(0.0f);
            else if (j<i) iRow.push_back(w[i][j]);

            Handle jHandle = (static_cast<HopfieldServer&>(server())).hGrid[j];
            HandleSeq outgoing;
            outgoing.push_back(iHandle);
            outgoing.push_back(jHandle);
            Handle heb = a->getHandle(SYMMETRIC_HEBBIAN_LINK,outgoing);
            if (heb != Handle::UNDEFINED) {
                iRow.push_back(a->getTV(heb).getMean());
            } else {
                heb = a->getHandle(SYMMETRIC_INVERSE_HEBBIAN_LINK,outgoing);
                if (heb != Handle::UNDEFINED)
                    iRow.push_back( -(a->getTV(heb).getMean()) );
                else {
                    iRow.push_back(0.0f);
                }
            }
        }
        w.push_back(iRow);
    }
#ifdef DEBUG
    assert (w.size() == (unsigned int) n);
    for (int i=0; i < n; i++)
        assert (w[i].size() == (unsigned int) n);
#endif
    return w;

}

void StorkeyAgent::setCurrentWeights(w_t& w)
{
    int n = (static_cast<HopfieldServer&>(server())).hGrid.size();
    for (int i=0; i < n; i++) {
        Handle iHandle = (static_cast<HopfieldServer&>(server())).hGrid[i];
        for (int j=i+1; j < n; j++) {
            if (i==j) continue;
            Handle jHandle = (static_cast<HopfieldServer&>(server())).hGrid[j];
            HandleSeq outgoing;
            outgoing.push_back(iHandle);
            outgoing.push_back(jHandle);

            Handle heb = a->getHandle(SYMMETRIC_HEBBIAN_LINK,outgoing);
            if (heb != Handle::UNDEFINED) {
                if (w[i][j] < 0.0f) {
                    a->removeAtom(heb, true);
                    a->addLink(SYMMETRIC_INVERSE_HEBBIAN_LINK, outgoing,
                            SimpleTruthValue(w[i][j],100));
                } else {
                    a->setMean(heb,w[i][j]);
                }
            } else {
                heb = a->getHandle(SYMMETRIC_INVERSE_HEBBIAN_LINK,outgoing);
                if (heb != Handle::UNDEFINED) {
                    if (w[i][j] > 0.0f) {
                        a->removeAtom(heb, true);
                        a->addLink(SYMMETRIC_HEBBIAN_LINK, outgoing,
                                SimpleTruthValue(w[i][j],100));
                    } else {
                        a->setMean(heb,w[i][j]);
                    }
                }
                // If both == Handle::UNDEFINED, then don't add weight. Link
                // is probably missing due to density < 1.0
            }
        }
    }

}

bool StorkeyAgent::checkWeightSymmetry(w_t& w) {
    int n = w.size();
    for (int i; i < n; i++) {
        for (int j; j < n; j++) {
            if (w[i][j] != w[j][i]) return false;
        }
    }
    return true;
}

void StorkeyAgent::storkeyUpdate()
{
    w_t currentWeights;
    w_t newWeights;

    currentWeights = getCurrentWeights();
    int n = currentWeights.size();
    for (int i = 0; i < n; i++) {
        std::vector<float> iRow;
        for (int j = 0; j < n; j++) {
            if (i==j) iRow.push_back(0.0f);
            float val = currentWeights[i][j];
            val += (1.0f/n) * a->getNormalisedSTI((static_cast<HopfieldServer&>(server())).hGrid[i],false) *
                a->getNormalisedSTI((static_cast<HopfieldServer&>(server())).hGrid[j],false);
            val -= (1.0f/n) * a->getNormalisedSTI((static_cast<HopfieldServer&>(server())).hGrid[i],false) *
                h(j,currentWeights);
            val -= (1.0f/n) * a->getNormalisedSTI((static_cast<HopfieldServer&>(server())).hGrid[j],false) *
                h(i,currentWeights);
            iRow.push_back(val);
        }
        newWeights.push_back(iRow);
    }
#ifdef DEBUG
    //printWeights(newWeights);
    assert(checkWeightSymmetry(newWeights));
#endif
    setCurrentWeights(newWeights);


}


