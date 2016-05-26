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
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/CogServer.h>

#ifndef _STIMULATION_AGENT_H_
#define _STIMULATION_AGENT_H_

namespace opencog {

    /**
     * A dummy agent whose soul purpose is exposing the stimulate_atom function
     * to other non agent codes with a pointer reference to this classe's object.
     */
    class StimulationAgent : public Agent {
    private:

    public:
        virtual ~StimulationAgent();
        StimulationAgent(CogServer& cs);

        virtual const ClassInfo& classinfo() const;
        static const ClassInfo& info();
        virtual void run();
    };

}
#endif /* _SENTENCEGENAGENT_H_ */
