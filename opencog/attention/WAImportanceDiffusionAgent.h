/*
 * AFImportanceDiffusionAgent.h
 *
 * Copyright (C) 2016 Opencog Foundation
 *
 * All Rights Reserved
 *
 * Written by Cosmo   Harrigan
 * Written by Misgana Bayetta <misgana.bayetta@gmail.com>
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


#ifndef WAIMPORTANCEDIFFUSIONAGENT_H
#define WAIMPORTANCEDIFFUSIONAGENT_H

#include <opencog/cogserver/server/CogServer.h>

#include "ImportanceDiffusionBase.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

/** Diffuses short term importance between atoms in the AtomSpace.
 *
 * Diffusion sources consist of STI proportionate based randomly selected set
 * of atoms.
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
class AtomSpace;
class WAImportanceDiffusionAgent : public ImportanceDiffusionBase
{
private:
    void spreadImportance();
    AttentionValue::sti_t calculateDiffusionAmount(Handle);

public:
    WAImportanceDiffusionAgent(CogServer&);
   ~WAImportanceDiffusionAgent();

    virtual void run();
    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
    static const ClassInfo _ci("opencog::WAImportanceDiffusionAgent");
        return _ci;
    }

    HandleSeq diffusionSourceVector(void);
};

/** @}*/
} // namespace


#endif /* WAIMPORTANCEDIFFUSIONAGENT_H */
