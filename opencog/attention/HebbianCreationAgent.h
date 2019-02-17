/*
 * opencog/attention/HebbianCreationAgent.h
 *
 * Written by Roman Treutlein
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

#ifndef _OPENCOG_HEBBIAN_CREATION_AGENT_H
#define _OPENCOG_HEBBIAN_CREATION_AGENT_H

#include <string>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/avalue/AttentionValue.h>
#include <opencog/attentionbank/bank/AttentionBank.h>
#include <opencog/cogserver/server/Agent.h>

#include "AttentionParamQuery.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

extern concurrent_queue<Handle> newAtomsInAV;

/**
 * This agent is resposible for Creating new HebbianLinks and making sure
 * that the maximum number of Links per Atom is not exceeded.
 * If an Atom enters the AttentionalFocus this Agent will create links between
 * it and all other atoms in the AttentionalFocus.
 * If will also create links to Atoms outside the Focus. The localToFarLinks
 * parameter decides how many of these "far" Links should be created.
 *
 * If after creating these Links the Atom has to many outgoing HebbianLinks
 * this agent will delete Links randomly until the number of links is less then
 * the maxLinkNum.
 *
 * This Agents is supposed to run in it's own Thread and gets Atoms that enter
 * the Focus via a shared queue  (newAtomsInAV) from the AttentionModule.
 */
class HebbianCreationAgent : public Agent
{
private:
    AttentionParamQuery _atq;

protected:
    AttentionBank* _bank;

    void addHebbian(Handle atom, Handle source);
    double targetConjunction(Handle handle1, Handle handle2);

    unsigned int maxLinkNum;

    //The Number of Local Links (Links to Atoms in the Focus)
    //for which 1 Far Link (Links to Atoms not in the Focus)
    //should be created
    int localToFarLinks;

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::HebbianCreationAgent");
        return _ci;
    }

    HebbianCreationAgent(CogServer&);
    virtual void run();

}; // class

typedef std::shared_ptr<HebbianCreationAgent> HebbianCreationAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_HEBBIAN_CREATION_AGENT_H
