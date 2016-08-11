/*
 * StimulationAgent.cc
 * 
 * Author: Misgana Bayetta <misgana.bayettta@gmail.com>
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include "StimulationAgent.h"

using namespace opencog;

StimulationAgent::~StimulationAgent() { }

StimulationAgent::StimulationAgent(CogServer& cs) :
Agent(cs) { }

const ClassInfo& StimulationAgent::classinfo() const
{
    return info();
}

const ClassInfo& StimulationAgent::info()
{
    static const ClassInfo _ci("opencog::StimulationAgent");
    return _ci;
}

void StimulationAgent::run(void) {
 }
