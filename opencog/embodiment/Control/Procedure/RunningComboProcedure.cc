/*
 * opencog/embodiment/Control/Procedure/RunningComboProcedure.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller, Moshe Looks
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

#include "RunningComboProcedure.h"
#include "ComboProcedure.h"
#include <boost/bind.hpp>
#include <sstream>
#include <map>
#include <opencog/util/functional.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>
#include <opencog/embodiment/AvatarComboVocabulary/avatar_builtin_action.h>
#include <opencog/embodiment/WorldWrapper/WorldWrapperUtil.h>

#include <opencog/comboreduct/interpreter/eval.h>

namespace opencog { namespace Procedure {

using namespace combo;
using namespace spatial;
using namespace pai;

RunningComboProcedure::RunningComboProcedure(WorldWrapperBase& ww,
        const combo::combo_tree& tr,
        const std::vector<combo::vertex>& arguments,
        bool dsdp, combo::variable_unifier& vu)
        : _ww(ww), _tr(tr), _it(_tr.begin()), _hasBegun(false),
        _failed(boost::logic::indeterminate), _inCompound(false),
        _arguments(arguments),
        _doesSendDefinitePlan(dsdp), _vu(vu)
{
    finished = false;
}

RunningComboProcedure::RunningComboProcedure(const RunningComboProcedure& rhs)
        : _ww(rhs._ww), _tr(rhs._tr), _it(_tr.begin()),
        _hasBegun(false), _planSent(false),
        _failed(boost::logic::indeterminate), _inCompound(false),
        _doesSendDefinitePlan(rhs._doesSendDefinitePlan), _vu(rhs._vu)
{
    if (_hasBegun != false || (!rhs._tr.empty() && rhs._it != rhs._tr.begin())) {
        std::stringstream stream (std::stringstream::out);
        stream << "cannot copy a combo procedure (" << _tr
        << ") that has begun running!" << std::endl;

        throw FatalErrorException(TRACE_INFO, "RunningComboProcedure - %s",
                                           stream.str().c_str());
    }

    finished = false;
}

vertex RunningComboProcedure::eval_action(combo_tree::iterator it, combo::variable_unifier& vu)
{
    //TODO
    OC_ASSERT(false, "Not implemented yet");
    return vertex();
}

vertex RunningComboProcedure::eval_procedure(combo::combo_tree::iterator it, combo::variable_unifier& vu)
{
    expand_procedure_call(it);
    *it = eval_throws(it, this, vu);
    _tr.erase_children(it);
    return *it;
}

void RunningComboProcedure::expand_procedure_call(combo::combo_tree::iterator it) throw (ComboException, AssertionException, std::bad_exception)
{

    // sanity checks
    if (!is_procedure_call(*it)) {
        throw ComboException(TRACE_INFO,
                                      "RunningComboProcedure - combo_tree node does not represent a combo procedure call.");
    }
    procedure_call pc = get_procedure_call(*it);

    arity_t ar = pc->arity();
    bool fixed_arity = ar >= 0; //the procedure gets fix number of
    //input arguments
    combo::arity_t exp_arity = combo::abs_min_arity(ar);
    arity_t ap_args = it.number_of_children();
    //OC_ASSERT(ar>=0, "It is assumed that arity is 0 or above, if not in that case the procedure must contain operator with arbitrary arity like and_seq and no variable binding to it, it is probably an error but if you really want to deal with that then ask Nil to add the support of it");
    if (fixed_arity) {
        if (ap_args != ar) {
            throw ComboException(TRACE_INFO,
                                          "RunningComboProcedure - %s arity differs from no. node's children. Arity: %d, number_of_children: %d",
                                          get_procedure_call(*it)->get_name().c_str(), ar, ap_args);
        }
    } else {
        if (ap_args < exp_arity) {
            throw ComboException(TRACE_INFO,
                                          "RunningComboProcedure - %s minimum arity is greater than no. node's children. Minimum arity: %d, number_of_children: %d",
                                          get_procedure_call(*it)->get_name().c_str(), exp_arity, ap_args);
        }
    }

    combo::combo_tree tmp(get_procedure_call(*it)->get_body());
    combo::set_bindings(tmp, it);
    *it = *tmp.begin();
    _tr.erase_children(it);
    _tr.reparent(it, tmp.begin());
}

void RunningComboProcedure::expand_and_evaluate_subtree(combo::combo_tree::iterator it, combo::variable_unifier& vu)
{
    //expand procedure calls and evaluate them
    combo::combo_tree::iterator end = it;
    end.skip_children();
    ++end;
    for (combo::combo_tree::iterator at = it;at != end;++at) {
        if (is_procedure_call(*at)) {
            expand_procedure_call(at);
            *at = eval_throws(at, this, vu);
            _tr.erase_children(at);
        }
    }
}


vertex RunningComboProcedure::eval_percept(combo::combo_tree::iterator it, combo::variable_unifier& vu)
{
    expand_and_evaluate_subtree(it, vu);
    return _ww.evalPerception(it, vu);
}

vertex RunningComboProcedure::eval_indefinite_object(indefinite_object io, combo::variable_unifier& vu)
{
    return _ww.evalIndefiniteObject(io, vu);
}

void RunningComboProcedure::cycle() throw(ActionPlanSendingFailure,
        AssertionException,
        std::bad_exception)
{
    //debug log
    if (logger().isDebugEnabled()) {
        stringstream tr_ss;
        tr_ss << _tr;
        stringstream it_ss;
        it_ss << *_it;
        string message("RunningComboProcedure::cycle() with _tr = ");
        message += tr_ss.str() + string("and _it = ") + it_ss.str();
        //std::cout << message << std::endl;
        logger().debug(message.c_str());
    }
    //~debug log

    if (!isReady())
        return;

    /**
     * First check if we are in the midst of a compound action - there are:
     * - goto_obj(obj speed)
     * - follow(obj duration speed)
     * - heel
     * - go_behind(obj avatar_obj|pet_obj speed)
     * - nudge_to(movable_obj, obj)
     */
    if (_inCompound) {
        if (!continueCompound()) {
            _inCompound = false;
            moveOn();
            if (!_tr.is_valid(_it))
                return;
        } else {
            return;
        }
    }

    //logger().error("pop");
    do {
        sib_it parent = _tr.parent(_it);
        //logger().error("ptp");
        if (*_it == id::action_while) {
            OC_ASSERT(_it.number_of_children() == 1);
            if (isFailed()) {
                _failed = false;
                moveOn();
            } else {
                _it = _it.begin();
            }

        } else if (_tr.is_valid(parent) && *parent == id::sequential_and &&
                   isFailed()) {
            //if the last plan failed and we need to abort a sequence
            logger().warn(
                         "RunningComboProc - Previous plan failed..."
                         " aborting sequence.");
            _it = parent.last_child();
            moveOn();

        } else if (*_it == id::action_action_if) {
            //need to eval an action_action_if's first child
            OC_ASSERT(_it.number_of_children() == 3);
            _it = _it.begin();

        } else if (*_it == id::action_boolean_if) {
            try {
                vertex res = eval_throws(_it.begin(), this, _vu);
                if (res == id::logical_true) {
                    _it = ++_it.begin();
                } else if (res == id::logical_false) {
                    _it = _it.last_child();
                } else {
                    std::stringstream stream (std::stringstream::out);
                    stream << "Conditional should be true or false. Got '"
                    << res << "', failing" << std::endl;

                    throw ComboException(TRACE_INFO,
                                                  "RunningComboProc - %s.",
                                                  stream.str().c_str());
                }

            } catch (...) {
                //some kind of runtime exception - halt execution and
                logger().error(
                             "RunningComboProc - action_boolean_if failure.");
                _failed = true;
                _it = _tr.end();
                return;
            }

        } else if (*_it == id::boolean_while) {
            if (isFailed()) {
                moveOn();
            } else {
                try {
                    vertex res = eval_throws(_it.begin(), this, _vu);
                    if (res == id::logical_true) {
                        _it = ++_it.begin();
                    } else {
                        OC_ASSERT(
                                         res == id::logical_false);
                        moveOn();
                        return;
                    }
                } catch (...) {
                    //some kind of runtime exception - halt execution and
                    logger().error(
                                 "RunningComboProc - boolean_while failure.");
                    _failed = true;
                    _it = _tr.end();
                    return;
                }
            }

        } else if (*_it == id::action_not) {
            _it = _it.begin();
        } else if (*_it == id::action_success) {
            _tr = combo::combo_tree(*_it);
            _it = _tr.begin();
            moveOn();

        } else if (*_it == id::action_failure) {
            _tr = combo::combo_tree(*_it);
            _it = _tr.begin();

            logger().error(
                         "RunningComboProc - Tree node to exec: action_failure.");

            _failed = true;
            moveOn();

        } else if (*_it == id::return_success) {
            OC_ASSERT(_failed == false || boost::logic::indeterminate(_failed));
            _it = _tr.end();
            return;

        } else if (*_it == id::repeat_n) {
            if (_stack.empty() || _stack.top().first != _it) {
                try {
                    vertex res = eval_throws(_it.begin(), this, _vu);
                    OC_ASSERT(is_contin(res));
                    _stack.push(make_pair(_it, get_contin(res)));
                } catch (...) {
                    //some kind of runtime exception - halt execution and
                    logger().error("RunningComboProc - repeat_n failure.");
                    _failed = true;
                    _it = _tr.end();
                    return;
                }
            }
            if (get_contin(_stack.top().second) >= 0.99) {
                _it = _it.last_child();
            } else {
                OC_ASSERT(!_stack.empty());
                _stack.pop();
                moveOn();
            }

        } else if (is_builtin_action(*_it) || *_it == id::sequential_and) {
            //builtin action - can finally do
            //something!
            if (*_it == id::sequential_and) {
                if (_it.is_childless()) {
                    moveOn();
                    //continue;
                    return;
                }
                parent = _it;
                _it = parent.begin();
            }
            if (WorldWrapperUtil::is_builtin_compound_action(*_it)) {
                bool tostop = false;
                for (sib_it sib = _it.begin();sib != _it.end();++sib) {
                    //first evaluate the children
                    *sib = eval_throws(sib, this, _vu);
                    if (*sib == id::null_obj) { //just fail
                        logger().error("RunningComboProc - Sibling is null_obj.");

                        _failed = true;
                        moveOn();
                        tostop = true;
                        break;
                    }
                    _tr.erase_children(sib);
                }
                if (tostop)
                    continue;
                if (beginCompound()) {
                    _inCompound = true;
                    return; //we actually began a compound action
                } else { // no compound action and goto action plan failed
                    if (isFailed()) {
                        return;
                    } else {
                        // compound action and goto action plan sucess
                        moveOn();
                        continue;
                    }
                }
            }
            bool execed =
                ( (_tr.is_valid(parent) && *parent == id::sequential_and) ?
                  execSeq(_it, std::find_if(_it, parent.end(),
                                            !boost::bind(&WorldWrapperUtil::is_builtin_atomic_action, _1)), _vu)
                  : exec(_it, _vu) );
            if (execed) {
                moveOn();
                return;
            }

        } else if (is_procedure_call(*_it)) { //an action procedure
            try {

                expand_procedure_call(_it);
                continue;

            } catch (ComboException& e) {
                logger().error("RunningComboProc - Exception catch when expanding procedure call.");
                // failed, cancel execution
                _failed = true;
                _it = _tr.end();
                return;
            }

        } else if (is_argument(*_it)) {
            OC_ASSERT(false);

        } else {

            if (_hasBegun) {
                std::stringstream stream (std::stringstream::out);
                stream << "Type error at '" << combo::combo_tree(_it) << "', failing" << std::endl;
                logger().error("RunningComboProcedure - %s.",
                             stream.str().c_str());

                _failed = true;
                _it = _tr.end();
                return;

            } else {
                //try passing it off to the regular combo interpreter - evaluation
                //without actions will get done in a single cycle
                _tr = combo::combo_tree(eval_throws(_it, this, _vu));
                _it = _tr.begin();
                moveOn();
            }
        }
    } while (_tr.is_valid(_it));
    //logger().error("popl");
}

bool RunningComboProcedure::execSeq(sib_it from, sib_it to, combo::variable_unifier& vu)
{
    //debug print
    //std::cout << "EXECSEQ FROM : "
    //  << (_tr.is_valid(from)? *from : "invalid")
    //          << " TO : " << (_tr.is_valid(to)? *to : "invalid")
    //   << std::endl;
    //~debug print
    if (from == to) {
        _it = from; //FIXME : not sure it is necessary
        return false;
    }
    _hasBegun = true; //since we're going to send a plan
    _failed = boost::logic::indeterminate;
    std::for_each(make_counting_iterator(from),
                  make_counting_iterator(to),
                  boost::bind(&RunningComboProcedure::expand_and_evaluate_subtree, this, _1, vu));

    if (_doesSendDefinitePlan) {
        for (sib_it sib = from; sib != to; ++sib) {
            for (sib_it arg = sib.begin(); arg != sib.end();++arg) {
                *arg = eval_throws(arg, this, vu);

                if (*arg == id::null_obj) {
                    //will definitely fail at this point..
                    *sib = id::action_failure;
                    _tr.erase_children(from);
                    to = sib;

                    if (from == to) {
                        _it = from;
                        return false;
                    }
                }
            }
        }
    }
    if (from == to) {
        _it = from; //FIXME : not sure it is necessary
        return false;
    }

    try {
        // this function can result in exceptions thrown from other parts
        // specially, PAIWorldProvider and PAI
        _planSent = _ww.sendSequential_and(from, to);

    } catch (...) {
        std::cout << "BEWARE : WorldWrapper has failed handling the plan, please see the application log for more detail."
                  << std::endl;
        _planSent = false;
        _failed = true;
    }

    _it = to;
    --_it;
    return _planSent;
}

bool RunningComboProcedure::beginCompound()
{
    _hasBegun = true; //since we're going to execute a plan

    try {

        // this function can result in exceptions thrown from other parts
        // specially PAIWorldProvider and PAI
        _planSent = _ww.sendSequential_and(_it, _tr.next_sibling(_it));

        // catch all exceptions here since the treatment will be the same for
        // them, i.e., action execution failed.
    } catch (RuntimeException& e) {
        logger().error("RunningComboProc - Runtime exception catch when sendSequential_and.");
        _planSent = false;
        _failed = true;
    }

    return _planSent;
}

bool RunningComboProcedure::continueCompound()
{
    return false; //for now no compound schema require multiple action plans,
    //so we can just return false
}

void RunningComboProcedure::moveOn()
{
    for (sib_it parent = _tr.parent(_it); _tr.is_valid(parent); parent = _tr.parent(_it)) {
        sib_it tmp_it = _it; //this is here just to memorize _it when
        //at the start of the loop,
        //this is to prevent infinite loop
        //indeed if by the end the loop _it has not
        //changed then loop would go endlessly
        if (*parent == id::sequential_and) {
            if (_it != parent.last_child()) {
                ++_it;
                return;
            } else {
                _it = parent;
            }
        } else if (*parent == id::action_action_if) {
            if (_it == parent.begin()) {
                //this implements branching based on success/failure of the
                _it = isFailed() ? parent.last_child() : ++parent.begin();
                return;
            } else { //we've already done the branching
                _it = parent;
            }
        } else if (*parent == id::action_boolean_if) {
            OC_ASSERT(_it != parent.begin(),
                             "we must be in an action branch");
            _it = parent;
        } else if (*parent == id::action_while) {
            _it = parent;
            return;
        } else if (*parent == id::repeat_n) {
            _it = parent;
            _stack.top().second = get_contin(_stack.top().second) - 1;
            return;
        } else if (*parent == id::boolean_while) {
            OC_ASSERT(_it != parent.begin()); //we must be in an action branch
            OC_ASSERT(_it == parent.last_child());
            _it = parent;
            return;
        } else if (*parent == id::action_while) {
            OC_ASSERT(_it == parent.begin());
            _it = parent;
            if (!isFailed()) //back for another action_while loop
                return;
        } else if (*parent == id::action_not) {
            OC_ASSERT(_it == parent.begin());
            _failed = !isFailed();
            _it = parent;
        }

        OC_ASSERT(!is_procedure_call(*parent));
        OC_ASSERT(_it != tmp_it,
                         "The loop go infinitly other wise, there must be a bug in that method or somewhere else");
    }
    //we are at the root and have executed it already - invalidate the iterator
    //so we will be marked as finished
    ++_it;
}


void RunningComboProcedure::init(const std::vector<vertex>& args)
{
    combo::set_bindings(_tr, args);
}

}} // ~namespace opencog::Procedure
