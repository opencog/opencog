/*
 * opencog/embodiment/Control/OperationalAvatarController/SleepAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef SLEEPAGENT_H
#define SLEEPAGENT_H

#include <opencog/server/Agent.h>

namespace opencog { namespace oac {

using namespace opencog;

class SleepAgent : public Agent
{

private:

public:

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::SleepAgent");
        return _ci;
    }

    virtual ~SleepAgent();
    SleepAgent(CogServer&);

    virtual void run();

}; // class

} } // namespace opencog::oac

#endif
