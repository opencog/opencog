/*
 * ImportanceDiffusionBase.cc
 *
 * Copyright (C) 2014-2016 Cosmo Harrigan
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
 * written by Misgana Bayetta
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

#ifndef IMPORTANCEDIFFUSIONBASE_H
#define IMPORTANCEDIFFUSIONBASE_H

#include <string>
#include <stack>
#include <math.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>

#include "AttentionParamQuery.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */
class AttentionBank;
/**
 * Common methods and variables used for Importance diffusion.
 */
class ImportanceDiffusionBase : public Agent
{
protected:
    AttentionBank* _bank;
    double maxSpreadPercentage;
    double hebbianMaxAllocationPercentage;
    bool spreadHebbianOnly;
    AttentionParamQuery _atq;

    typedef struct DiffusionEventType
    {
        Handle source;
        Handle target;
        AttentionValue::sti_t amount;
    } DiffusionEventType;

    std::stack<DiffusionEventType> diffusionStack;
    void processDiffusionStack();

    HandleSeq diffusionSourceVector(void);

    HandleSeq incidentAtoms(Handle);
    HandleSeq hebbianAdjacentAtoms(Handle);

    std::map<Handle, double> probabilityVector(HandleSeq);
    std::map<Handle, double> probabilityVectorIncident(HandleSeq);
    std::map<Handle, double> probabilityVectorHebbianAdjacent(Handle, HandleSeq);
    std::map<Handle, double> combineIncidentAdjacentVectors(
            std::map<Handle, double>, std::map<Handle, double>);

    double calculateHebbianDiffusionPercentage(Handle);
    double calculateIncidentDiffusionPercentage(Handle);

    void tradeSTI(DiffusionEventType);
    void updateMaxSpreadPercentage();

    void diffuseAtom(Handle);
    virtual void spreadImportance() = 0;
    virtual AttentionValue::sti_t calculateDiffusionAmount(Handle) = 0;

public:
    ImportanceDiffusionBase(CogServer&);
    virtual ~ImportanceDiffusionBase();

}; // class

/** @}*/
} // namespace

#endif /* IMPORTANCEDIFFUSIONBASE_H */

