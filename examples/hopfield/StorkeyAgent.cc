/*
 * opencog/attention/StorkeyAgent.cc
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

#include <iomanip>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/truthvalue/IndefiniteTruthValue.h>
#include <opencog/atoms/truthvalue/SimpleTruthValue.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

#include <opencog/attention/atom_types.h>

#include "HopfieldServer.h"

#define DEBUG

using namespace opencog;
using namespace std;

StorkeyAgent::StorkeyAgent(CogServer& cs) : Agent(cs)
{
    static const std::vector<std::string> defaultConfig = {
        "ECAN_CONVERT_LINKS","false",
        "",""
    };
    setParameters(defaultConfig);

    convertLinks = config().get_bool("ECAN_CONVERT_LINKS");

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("StorkeyAgent.log", Logger::WARN, true));
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

void StorkeyAgent::printWeights(w_t& w)
{
    int n = w.size();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (w[i][j] != 0.0f)
                printf("%+.3f ", w[i][j]);
            else
                printf(" ----- ");
        }
        cout << endl;
    }
}

void StorkeyAgent::run()
{
    storkeyUpdate();
}

float StorkeyAgent::h(int i, int j, w_t& w)
{
    HopfieldServer& hs = static_cast<HopfieldServer&>(_cogserver);
    AtomSpace& a = hs.getAtomSpace();
    int n = w.size();
    float h_sum = 0.0f;
    for (int k=0; k < n; k++) {
        if (k == i || k == j) continue;
        h_sum += ( w[i][k] * a.get_normalised_STI(hs.hGrid[k],false) );
    }
    return h_sum;
}

float StorkeyAgent::h(int i, w_t& w)
{
    HopfieldServer& hs = static_cast<HopfieldServer&>(_cogserver);
    AtomSpace& a = hs.getAtomSpace();
    int n = w.size();
    float h_sum = 0.0f;
    for (int k=0; k < n; k++) {
        h_sum += ( w[i][k] * a.get_normalised_STI(hs.hGrid[k],false) );
    }
    return h_sum;
}

StorkeyAgent::w_t StorkeyAgent::getCurrentWeights()
{
    HopfieldServer& hs = static_cast<HopfieldServer&>(_cogserver);
    AtomSpace& a = hs.getAtomSpace();
    int n = hs.hGrid.size();
    std::vector<std::vector<float> > w;
    for (int i=0; i < n; i++) {
        std::vector<float> iRow(n);
        Handle iHandle = hs.hGrid[i];
        for (int j=0; j < n; j++) {
            if (i==j) {
                iRow[j] = 0.0f;
                continue;
            } else if (j<i) {
                //cout << "ij=" << i <<"," << j << " w[j][i]=="<<w[j][i]<<endl;
                iRow[j] = w[j][i];
                //cout << "irow[j]" << iRow[j] <<endl;
                continue;
            }

            Handle jHandle = hs.hGrid[j];
            HandleSeq outgoing;
            outgoing.push_back(iHandle);
            outgoing.push_back(jHandle);
            Handle heb = a.get_handle(SYMMETRIC_HEBBIAN_LINK,outgoing);
            if (heb != Handle::UNDEFINED) {
                iRow[j] = heb->getTruthValue()->getMean();
            } else {
                heb = a.get_handle(SYMMETRIC_INVERSE_HEBBIAN_LINK,outgoing);
                if (heb != Handle::UNDEFINED)
                    iRow[j] = -(heb->getTruthValue()->getMean());
                else {
                    iRow[j] = 0.0f;
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

static void setMean(Handle h, float mean)
{
    TruthValuePtr oldtv(h->getTruthValue());
    switch (oldtv->getType())
    {
        case SIMPLE_TRUTH_VALUE: {
            TruthValuePtr newtv(SimpleTruthValue::createTV(mean, oldtv->getCount()));
            h->setTruthValue(newtv);
            break;
        }
        case INDEFINITE_TRUTH_VALUE: {
	        OC_ASSERT(false, "Not implemented");
            // IndefiniteTruthValuePtr newtv(IndefiniteTruthValue::createITV(oldtv));
            // newtv->setMean(mean);
            // h->setTruthValue(newtv);
            break;
        }
        default:
            throw InvalidParamException(TRACE_INFO, "Unsupported TV type");
    }
}

void StorkeyAgent::setCurrentWeights(w_t& w)
{
    HopfieldServer& hs = static_cast<HopfieldServer&>(_cogserver);
    AtomSpace& a = hs.getAtomSpace();
    int n = hs.hGrid.size();
    for (int i=0; i < n; i++) {
        Handle iHandle = hs.hGrid[i];
        for (int j=i+1; j < n; j++) {
            if (i==j) continue;
            Handle jHandle = hs.hGrid[j];
            HandleSeq outgoing;
            outgoing.push_back(iHandle);
            outgoing.push_back(jHandle);

            Handle heb = a.get_handle(SYMMETRIC_HEBBIAN_LINK,outgoing);
            if (heb != Handle::UNDEFINED) {
                if (w[i][j] < 0.0f) {
                    a.remove_atom(heb, true);
                    a.add_link(SYMMETRIC_INVERSE_HEBBIAN_LINK, outgoing)->merge(
                            SimpleTruthValue::createTV(-w[i][j],100));
                } else {
                    setMean(heb, w[i][j]);
                }
            } else {
                heb = a.get_handle(SYMMETRIC_INVERSE_HEBBIAN_LINK,outgoing);
                if (heb != Handle::UNDEFINED) {
                    if (w[i][j] > 0.0f) {
                        a.remove_atom(heb, true);
                        a.add_link(SYMMETRIC_HEBBIAN_LINK, outgoing)->merge(
                                SimpleTruthValue::createTV(w[i][j],100));
                    } else {
                        setMean(heb, -w[i][j]);
                    }
                }
                // If both == Handle::UNDEFINED, then don't add weight. Link
                // is probably missing due to density < 1.0
            }
        }
    }

}

bool StorkeyAgent::checkWeightSymmetry(w_t& w)
{
    int n = w.size();
    float err = 0.001;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (w[i][j] < (w[j][i] - err) ||
                w[i][j] > (w[j][i] + err)) return false;
        }
    }
    return true;
}

void StorkeyAgent::storkeyUpdate()
{
    HopfieldServer& hs = static_cast<HopfieldServer&>(_cogserver);
    AtomSpace& a = hs.getAtomSpace();
    w_t currentWeights;
    w_t newWeights;

    currentWeights = getCurrentWeights();
    int n = currentWeights.size();
    for (int i = 0; i < n; i++) {
        std::vector<float> iRow;
        for (int j = 0; j < n; j++) {
            if (i==j) {
                iRow.push_back(0.0f);
                continue;
            }
            float val = currentWeights[i][j];
/*            cout << " before val = " << val << endl;
            cout << " hGrid[i] = " << a.getNormalisedSTI(hs.hGrid[i],false);
            cout << " hGrid[j] = " << a.getNormalisedSTI(hs.hGrid[j],false);
            cout << " h_i = " << h(i,currentWeights);
            cout << " h_j = " << h(j,currentWeights) << endl; */
            val += (1.0f/n) * a.get_normalised_STI(hs.hGrid[i],false) *
                a.get_normalised_STI(hs.hGrid[j],false);
            val -= (1.0f/n) * a.get_normalised_STI(hs.hGrid[i],false) *
                h(j,currentWeights);
            val -= (1.0f/n) * a.get_normalised_STI(hs.hGrid[j],false) *
                h(i,currentWeights);
//            cout << " after val = " << val << endl;
            iRow.push_back(val);
        }
        newWeights.push_back(iRow);
    }
    if (getLogger()->is_fine_enabled()) {
        getLogger()->fine("Weight matrix after Storkey update rule applied:");
        printWeights(newWeights);
    }
#ifdef DEBUG
    assert(checkWeightSymmetry(newWeights));
#endif
    setCurrentWeights(newWeights);


}
