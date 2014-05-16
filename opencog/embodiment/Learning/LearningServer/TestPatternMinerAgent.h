/*
 * opencog/learning/PatternMiner/TestPatternMinerAgent.h
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

#ifndef TESTPATTERNMINERAGENT_H
#define TESTPATTERNMINERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/learning/PatternMiner/PatternMiner.h>

using namespace opencog::PatternMining;

namespace opencog
{

class CogServer;

class TestPatternMinerAgent : public Agent
{
public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::TestPatternMinerAgent");
        return _ci;
    }

    TestPatternMinerAgent(CogServer&);
    virtual ~TestPatternMinerAgent();
    virtual void run();

private:
    PatternMiner* patternMiner;

}; // class

class TestPatternMinerModule : public Module
{
private:

    Factory<TestPatternMinerAgent, Agent> factory;

public:

    TestPatternMinerModule(CogServer&);
    virtual ~TestPatternMinerModule();
    virtual void init();
    virtual const char* id();
};

} // namespace opencog

#endif // TESTPATTERNMINERAGENT_H
