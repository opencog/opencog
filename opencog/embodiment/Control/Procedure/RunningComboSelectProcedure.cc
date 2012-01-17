/*
 * opencog/embodiment/Control/Procedure/RunningComboSelectProcedure.cc
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

#include "RunningComboSelectProcedure.h"

namespace opencog { namespace Procedure {

RunningComboSelectProcedure::RunningComboSelectProcedure(ComboInterpreter& i,
        const ComboProcedure& f,
        const ComboProcedure& s,
        const std::vector<combo::vertex> args,
        combo::variable_unifier& vu)
        : interpreter(i), firstScript(f), secondScript(s), unifier(vu)
{
    this->firstScriptFinished = false;
    this->firstScriptFailed = false;

    this->secondScriptFinished = false;
    this->secondScriptFailed = false;

    this->result = combo::id::logical_false;

    this->arguments.assign(args.begin(), args.end());
}

void RunningComboSelectProcedure::cycle()
{
    // for now the first script wont support parameters, only wild-cards
    std::vector<combo::vertex> empty_args;
    Procedure::RunningProcedureId firstScriptId = interpreter.runProcedure(firstScript.getComboTree(), empty_args, unifier);

    // execute the first script until it's end
    while (!interpreter.isFinished(firstScriptId)) {
        interpreter.run(NULL);
    }

    this->firstScriptFinished = true;

    if (interpreter.isFailed(firstScriptId)) {
        this->firstScriptFailed = true;

        // mark second script as finished and failed too, since they wont be
        // executed
        this->secondScriptFinished = true;
        this->secondScriptFailed = true;

        this->result = combo::id::logical_false;
        this->unifier = combo::variable_unifier::DEFAULT_VU();
        this->unifier.setUpdated(true);

        // end procedure exec
        return;
    }

    combo::vertex firstScriptResult = interpreter.getResult(firstScriptId);
    this->unifier = interpreter.getUnifierResult(firstScriptId);
    std::stringstream ss;

    ss << firstScriptResult;
    OC_ASSERT(is_builtin(firstScriptResult),
             "RunningComboSelectProcedure - First script result should be a buit-in. Got '%s'",
             ss.str().c_str());

    // only execute the second script if the first one evaluates true.
    if (get_builtin(firstScriptResult) == combo::id::logical_true) {

        // no need for unification, normal execution
        if (this->secondScript.getArity() == 0) {
            OC_ASSERT(this->arguments.size() == 0,
                    "RunningComboSelectProcedure - args should be empty.");

            Procedure::RunningProcedureId id = interpreter.runProcedure(
                    secondScript.getComboTree(), empty_args);
            while (!interpreter.isFinished(id)) {
                interpreter.run(NULL);
            }

            this->secondScriptFinished = true;

            if (!interpreter.isFailed(id)) {
                this->result = interpreter.getResult(id);
                this->secondScriptFailed = false;
            } else {
                this->secondScriptFailed = true;
            }

            // one or more parameters, if unifier isn't empty, the last parameter MUST be an
            // unifier result
        } else {
            OC_ASSERT(this->arguments.size() == 0,
                    "RunningComboSelectProcedure - args should be empty.");

            // variable unifier - use it
            if (!unifier.empty()) {

                combo::UnifierIt it;
                this->secondScriptFailed = true; // if any candidate do not fail -> change to false
                this->unifier.setOneVariableActive(false);

                for (it = unifier.begin(); it != unifier.end(); it++) {
                    if (it->second) {
                        logger().debug("RunningComboSelect - unified var '%s'.",
                                     it->first.c_str());

                        this->arguments.push_back(it->first);
                        Procedure::RunningProcedureId id = interpreter.runProcedure(
                                secondScript.getComboTree(), arguments);

                        // execute the first script until it's end
                        while (!interpreter.isFinished(id)) {
                            interpreter.run(NULL);
                        }

                        if (!interpreter.isFailed(id)) {
                            this->secondScriptFailed = false;
                            combo::vertex res = interpreter.getResult(id);

                            if ( (is_builtin(res) &&
                                 get_builtin(res) == combo::id::logical_true) ||
                                 (is_action_result(res) &&
                                 get_action(res) == combo::id::action_success)) {

                                this->result = res;

                                // no need to reset the unifier variable true
                                // since it is already true.
                                this->unifier.setOneVariableActive(true);

                            } else {
                                std::string candidate(it->first);
                                unifier.setVariableState(candidate, false);
                            }

                        } else {
                            std::string candidate(it->first);
                            unifier.setVariableState(candidate, false);
                        }

                        this->arguments.pop_back();
                    }
                }
                this->secondScriptFinished = true;
                this->unifier.setUpdated(true);

                // no unifier - normal case
            } else {
                Procedure::RunningProcedureId id = interpreter.runProcedure(
                        secondScript.getComboTree(), arguments);

                while (!interpreter.isFinished(id)) {
                    interpreter.run(NULL);
                }

                this->secondScriptFinished = true;

                if (!interpreter.isFailed(id)) {
                    this->result = interpreter.getResult(id);
                    this->secondScriptFailed = false;

                } else {
                    this->secondScriptFailed = true;
                }
            }
        }

        // first script evaluated false, result should be a logical false and unifier
        // equals null.
    } else {
        this->secondScriptFailed = false;
        this->secondScriptFinished = true;
        this->result = combo::id::logical_false;
        this->unifier = combo::variable_unifier::DEFAULT_VU();
        this->unifier.setUpdated(true);
    }
}

bool RunningComboSelectProcedure::isFinished() const
{
    if (firstScriptFinished && secondScriptFinished) {
        return true;
    }
    return false;
}

bool RunningComboSelectProcedure::isFailed() const
{
    if (firstScriptFailed || secondScriptFailed) {
        return true;
    }
    return false;
}

combo::vertex RunningComboSelectProcedure::getResult()
{
    OC_ASSERT(isFinished(), "RunningComboSelectProcedure - Procedure isn't finished.");
    if (isFailed()) {
        return combo::id::action_failure;
    }
    return result;
}

combo::variable_unifier& RunningComboSelectProcedure::getUnifierResult()
{
    OC_ASSERT(isFinished(), "RunningComboSelectProcedure - Procedure isn't finished.");
    OC_ASSERT(!isFailed(), "RunningComboSelectProcedure - Procedure failed.");

    return unifier;
}

}} // ~namespace opencog::Procedure
