/*
 * opencog/embodiment/Control/Procedure/RunningComboSelectProcedure.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef _RUNNING_COMBO_SELECT_PROCEDURE_H
#define _RUNNING_COMBO_SELECT_PROCEDURE_H

#include "ComboProcedure.h"
#include "ComboInterpreter.h"

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/variable_unifier.h>

namespace opencog { namespace Procedure {

class RunningComboSelectProcedure
{

public:

    RunningComboSelectProcedure(ComboInterpreter& interpreter,
                                const ComboProcedure& f,
                                const ComboProcedure& s,
                                const std::vector<combo::vertex> args,
                                combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    void cycle();

    bool isFinished() const;

    bool isFailed() const;

    combo::vertex getResult();

    combo::variable_unifier& getUnifierResult();

private:
    ComboInterpreter& interpreter;

    ComboProcedure firstScript;
    ComboProcedure secondScript;

    std::vector<combo::vertex> arguments;

    // the results of
    combo::variable_unifier unifier;
    combo::vertex result;

    bool firstScriptFinished;
    bool firstScriptFailed;

    bool secondScriptFinished;
    bool secondScriptFailed;

}; // class

}} // ~namespace opencog::Procedure

#endif
