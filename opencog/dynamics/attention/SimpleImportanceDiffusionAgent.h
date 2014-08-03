/*
 * opencog/dynamics/attention/SimpleImportanceDiffusionAgent.h
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

#ifndef _OPENCOG_SIMPLE_IMPORTANCE_DIFFUSION_AGENT_H
#define _OPENCOG_SIMPLE_IMPORTANCE_DIFFUSION_AGENT_H

#include <string>
#include <stack>
#include <math.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/server/Agent.h>
#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>
#include "SpreadDecider.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class CogServer;

/** Diffuses short term importance between atoms in the attentional focus.
 *
 * Diffusion sources consist of all atoms that are in the attentional focus.
 * 
 * Supports two types of importance diffusion:
 * 
 * (1) Diffusion to atoms that are incident to each atom, consisting of that 
 *     atom's incoming and outgoing sets (excluding hebbian links)
 * 
 * (2) Diffusion to atoms that are adjacent to each atom, where the type of
 *     the connecting edge is a hebbian link
 * 
 * Please refer to the detailed description of this agent in the README file,
 * where an extensive explanation of the algorithm, features and pending
 * work is explained.
 */
class SimpleImportanceDiffusionAgent : public Agent
{

private:
    AtomSpace* as;
    float maxSpreadPercentage;
    float hebbianMaxAllocationPercentage;
    bool spreadHebbianOnly;
    SpreadDecider* spreadDecider;
    void setLogger(Logger* l);
    Logger *log;
    
    typedef struct DiffusionEventType
    {
        Handle source;
        Handle target;
        AttentionValue::sti_t amount;
    } DiffusionEventType;

    std::stack<DiffusionEventType> diffusionStack;
    void processDiffusionStack();
    
    void spreadImportance();
    void diffuseAtom(Handle);
    HandleSeq diffusionSourceVector();
    HandleSeq incidentAtoms(Handle);
    HandleSeq hebbianAdjacentAtoms(Handle);
    std::map<Handle, double> probabilityVector(HandleSeq);    
    AttentionValue::sti_t calculateDiffusionAmount(Handle);
    float calculateHebbianDiffusionPercentage(Handle);
    float calculateIncidentDiffusionPercentage(Handle);
    std::map<Handle, double> probabilityVectorIncident(HandleSeq);
    std::map<Handle, double> probabilityVectorHebbianAdjacent(Handle, HandleSeq);
    std::map<Handle, double> combineIncidentAdjacentVectors(
            std::map<Handle, double>, std::map<Handle, double>);
    void tradeSTI(DiffusionEventType);
    void updateMaxSpreadPercentage();
    
public:
    enum { HYPERBOLIC, STEP };
    void setSpreadDecider(int type, float shape = 30);
    void setMaxSpreadPercentage(float);
    void setHebbianMaxAllocationPercentage(float);
    void setSpreadHebbianOnly(bool);
    SimpleImportanceDiffusionAgent(CogServer&);
    virtual ~SimpleImportanceDiffusionAgent();
    virtual void run();
    Logger* getLogger();
    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::SimpleImportanceDiffusionAgent");
        return _ci;
    }
}; // class

typedef std::shared_ptr<SimpleImportanceDiffusionAgent> SimpleImportanceDiffusionAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_SIMPLE_IMPORTANCE_DIFFUSION_AGENT_H
