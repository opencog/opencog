/*
 * opencog/learning/PatternMiner/PatternMinerAgent.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
 * April 17 2014
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

#ifndef PatternMinerAgent_H
#define PatternMinerAgent_H

#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/cogserver/server/Module.h>
#include <opencog/learning/PatternMiner/PatternMiner.h>

using namespace opencog::PatternMining;

namespace opencog
{

class CogServer;

class PatternMinerAgent : public Agent
{
public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::PatternMinerAgent");
        return _ci;
    }

    PatternMinerAgent(CogServer&);
    virtual ~PatternMinerAgent();
    virtual void run();

private:
    PatternMiner* patternMiner;

    void selectSubSetFromCorpus();

}; // class

class PatternMinerModule : public Module
{
private:

    Factory<PatternMinerAgent, Agent> factory;

public:

    PatternMinerModule(CogServer&);
    virtual ~PatternMinerModule();
    virtual void init();
    virtual const char* id();
};

} // namespace opencog

#endif // PatternMinerAgent_H
