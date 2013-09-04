/*
 * examples/modules/SingleAgentModule.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifndef _OPENCOG_SINGLE_AGENT_MODULE_H
#define _OPENCOG_SINGLE_AGENT_MODULE_H

#include <string>

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>

namespace opencog
{

class CogServer;

class SingleAgentModule : public Agent, public Module
{
public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::SingleAgentModule");
        return _ci;
    }

    static inline AbstractFactory<Agent>& factory() {
        static Factory<SingleAgentModule, Agent> _factory;
        return _factory;
    }

    SingleAgentModule(CogServer&);
    virtual ~SingleAgentModule();
    virtual void run();
    virtual void init();

    void stopAgent();

    std::string name;

}; // class

} // namespace opencog

#endif // _OPENCOG_SINGLE_AGENT_MODULE_H
