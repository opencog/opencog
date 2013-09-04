/*
 * opencog/embodiment/Control/OperationalAvatarController/ProcedureInterpreterAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


#ifndef PROCEDUREINTERPRETERAGENT_H
#define PROCEDUREINTERPRETERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/embodiment/Control/Procedure/ProcedureInterpreter.h>

namespace opencog { namespace oac {

class ProcedureInterpreterAgent : public opencog::Agent
{

private:

    Procedure::ProcedureInterpreter* interpreter;

public:

    ProcedureInterpreterAgent(CogServer&);
    virtual ~ProcedureInterpreterAgent();
    void setInterpreter(Procedure::ProcedureInterpreter*);

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::ProcedureInterpreterAgent");
        return _ci;
    }

    virtual void run();

}; // class

} } // namespace opencog::oac

#endif
