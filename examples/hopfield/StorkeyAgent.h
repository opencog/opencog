/*
 * examples/hopfield/StorkeyAgent.h
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

#ifndef _OPENCOG_STORKEY_AGENT_H
#define _OPENCOG_STORKEY_AGENT_H

#include <string>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/util/Logger.h>

#include "Pattern.h"

namespace opencog
{

class CogServer;

/** Agent that carries out learning rule of Storkey's 1998 paper.
 *
 * Only supports SYMMETRIC_HEBBIAN_LINKs and SYMMETRIC_INVERSE_HEBBIAN_LINKs.
 * If other Hebbian links exist, they are ignored.
 *
 */
class StorkeyAgent : public Agent
{
public:
    typedef std::vector< std::vector<float> > w_t;
private:
    bool verbose;

    /** Set the agent's logger object
     *
     * Note, this will be deleted when this agent is.
     *
     * @param l The logger to associate with the agent.
     */
    void setLogger(Logger* l);

    Logger *log; //!< Logger object for Agent

    //Pattern epsilon; //!< Pattern for updating network weights with

    void setCurrentWeights(w_t& w);
    bool checkWeightSymmetry(w_t& w);
    void printWeights(w_t& w);

public:

    float h(int i, w_t& w); //!< h function in learning rule from Storkey
    //! h function in old palimpsest learning rule from Storkey, needed for
    //! finding keyNode.
    float h(int i, int j, w_t& w);
    w_t getCurrentWeights();

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::StorkeyAgent");
        return _ci;
    }

    StorkeyAgent(CogServer&);
    virtual ~StorkeyAgent();
    virtual void run();

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

    //! Whether to convert links to/from InverseHebbianLinks as necessary.
    bool convertLinks;

    //! Maximum allowable LTI of a link to be converted.
    AttentionValue::lti_t conversionThreshold;

    //! Set pattern to update weights with
    void setPattern(Pattern _epsilon);

	//! Update the TruthValues of the HebbianLinks in the AtomSpace.
    void storkeyUpdate();

}; // class

typedef std::shared_ptr<StorkeyAgent> StorkeyAgentPtr;

} // namespace

#endif // _OPENCOG_STORKEY_AGENT_H
