/*
 * opencog/dynamics/attention/AttentionModule.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_ATTENTION_MODULE_H
#define _OPENCOG_ATTENTION_MODULE_H

#include <opencog/dynamics/attention/ForgettingAgent.h>
#include <opencog/dynamics/attention/HebbianUpdatingAgent.h>
#include <opencog/dynamics/attention/ImportanceSpreadingAgent.h>
#include <opencog/dynamics/attention/ImportanceDiffusionAgent.h>
#include <opencog/dynamics/attention/ImportanceUpdatingAgent.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class AttentionModule : public Module
{

private:
    Factory<ForgettingAgent, Agent>          forgettingFactory;
    Factory<HebbianUpdatingAgent, Agent>     hebbianFactory;
    Factory<ImportanceSpreadingAgent, Agent> spreadingFactory;
#ifdef HAVE_GSL
    Factory<ImportanceDiffusionAgent, Agent> diffusionFactory;
#endif
    Factory<ImportanceUpdatingAgent, Agent>  updatingFactory;

public:

    static inline const char* id();

    AttentionModule(CogServer&);
    virtual ~AttentionModule();
    virtual void init();

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_ATTENTION_MODULE_H
