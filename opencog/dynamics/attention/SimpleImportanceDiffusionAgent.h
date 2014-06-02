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
 */
class SimpleImportanceDiffusionAgent : public Agent
{

private:
    AtomSpace* as;

    // Maximum percentage of importance to spread
    float maxSpreadPercentage;

    // Whether to spread STI along hebbian links only, or to also spread to
    // atoms that are incident to the source
    bool spreadHebbianOnly;

    SpreadDecider* spreadDecider;
    void setLogger(Logger* l);
    Logger *log;
    
    void spreadImportance();
    void diffuseAtom(Handle);
    HandleSeq diffusionSourceVector();
    HandleSeq incidentAtoms(Handle);
    HandleSeq hebbianAdjacentAtoms(Handle);
    std::map<Handle, double> probabilityVector(HandleSeq);    
    AttentionValue::sti_t calculateDiffusionAmount(Handle);

public:
    enum { HYPERBOLIC, STEP };
    void setSpreadDecider(int type, float shape = 30);
    SimpleImportanceDiffusionAgent(CogServer&);
    virtual ~SimpleImportanceDiffusionAgent();
    virtual void run();
    Logger* getLogger();
}; // class

typedef std::shared_ptr<SimpleImportanceDiffusionAgent> SimpleImportanceDiffusionAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_SIMPLE_IMPORTANCE_DIFFUSION_AGENT_H
