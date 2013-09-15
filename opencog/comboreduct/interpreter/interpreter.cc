/** interpreter.cc --- 
 *
 * Copyright (C) 2012 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#include "interpreter.h"
#include "../combo/iostream_combo.h"
#include "../crutil/exception.h"

#include <opencog/util/numeric.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/mt19937ar.h>

namespace opencog { namespace combo {

/////////////////////////
// Boolean interpreter //
/////////////////////////

boolean_interpreter::boolean_interpreter(const std::vector<builtin>& inputs)
    : boolean_inputs(inputs) {}

builtin boolean_interpreter::operator()(const combo_tree& tr) const {
    return boolean_eval(tr.begin());
}
builtin boolean_interpreter::operator()(const combo_tree::iterator it) const {
    return boolean_eval(it);
}

builtin boolean_interpreter::boolean_eval(combo_tree::iterator it) const
{
    typedef combo_tree::sibling_iterator sib_it;
    const vertex& v = *it;

    if (const argument* a = boost::get<argument>(&v)) {
        arity_t idx = a->idx;
        return idx > 0? boolean_inputs[idx - 1]
            : negate_builtin(boolean_inputs[-idx - 1]);
    }
    // boolean builtin
    else if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {

        // Boolean operators
        case id::logical_false :
        case id::logical_true :
            return *b;

        case id::logical_and : {
            if (it.is_childless()) // For correct foldr behaviour
                return id::logical_and;
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                if (boolean_eval(sib) == id::logical_false)
                    return id::logical_false;
            return id::logical_true;
        }

        case id::logical_or : {
            if (it.is_childless())  // For correct foldr behaviour
                return id::logical_or;
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                if (boolean_eval(sib) == id::logical_true)
                    return id::logical_true;
            return id::logical_false;
        }
            
        case id::logical_not :
            return negate_builtin(boolean_eval(it.begin()));
            
        default: {
            std::stringstream ss;
            ss << *b;
            OC_ASSERT(false, "boolean_interpreter does not handle builtin %s",
                      ss.str().c_str());
            return builtin();
        }
        }
    } else {
        std::stringstream ss;
        ss << v;
        OC_ASSERT(false, "boolean_interpreter does not handle vertex %s",
                  ss.str().c_str());
        return builtin();
    }
}

////////////////////////
// Contin interpreter //
////////////////////////

contin_interpreter::contin_interpreter(const std::vector<contin_t>& inputs)
    : contin_inputs(inputs) {}

contin_t contin_interpreter::operator()(const combo_tree& tr) const {
    return contin_eval(tr.begin());
}
contin_t contin_interpreter::operator()(const combo_tree::iterator it) const {
    return contin_eval(it);
}

contin_t contin_interpreter::contin_eval(combo_tree::iterator it) const
{
    typedef combo_tree::sibling_iterator sib_it;
    const vertex& v = *it;

    if (const argument* a = boost::get<argument>(&v)) {
        return contin_inputs[a->idx - 1];
    }
    // contin builtin
    else if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {

        // Continuous operators
        case id::plus : {
            // If plus does not have any argument, return plus operator
            if (it.is_childless()) // For correct foldr behaviour
                return *b;

            contin_t res = 0;
            // Assumption : plus can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                res += contin_eval(sib);
            return res;
        }

        case id::times : {
            // If times does not have any argument, return times operator
            if (it.is_childless())  // For correct foldr behaviour
                return *b;

            contin_t res = 1;
            // Assumption : times can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                res *= contin_eval(sib);
                if (0.0 == res) return res;  // avoid pointless evals
            }
            return res;
        }

        case id::div : {
            contin_t x, y;
            sib_it sib = it.begin();
            x = contin_eval(sib);
            if (0.0 == x) return x;  // avoid pointless evals
            ++sib;
            y = contin_eval(sib);
            contin_t res = x / y;
            if (isnan(res) || isinf(res))
                throw OverflowException(vertex(res));
            return res;
        }

        case id::log : {
            contin_t x = contin_eval(it.begin());
#ifdef ABS_LOG
            contin_t res = log(std::abs(x));
#else
            contin_t res = log(x);
#endif
            if (isnan(res) || isinf(res))
                throw OverflowException(vertex(res));
            return res;
        }

        case id::exp : {
            contin_t res = exp(contin_eval(it.begin()));
            // This may happen when the argument is too large, then
            // exp will be infty
            if (isinf(res)) throw OverflowException(vertex(res));
            return res;
        }

        case id::sin : {
            return sin(contin_eval(it.begin()));
        }

        case id::rand :
            return randGen().randfloat();
            
        default: {
            std::stringstream ss;
            ss << *b;
            throw ComboException(TRACE_INFO,
                  "contin_interpreter does not handle builtin %s",
                  ss.str().c_str());
            return contin_t();
        }
        }
    }
    // contin constant
    else if (const contin_t* c = boost::get<contin_t>(&v)) {
        if (isnan(*c) || isinf(*c))
            throw OverflowException(vertex(*c));
        return *c;
    }
    else {
        std::stringstream ss;
        ss << v;
        throw ComboException(TRACE_INFO,
              "contin_interpreter does not handle vertex %s",
              ss.str().c_str());
        return contin_t();
    }
}

///////////////////////
// Mixed interpreter //
///////////////////////

mixed_interpreter::mixed_interpreter(const std::vector<vertex>& inputs)
    : _use_contin_inputs(false), mixed_inputs(inputs) {}

mixed_interpreter::mixed_interpreter(const std::vector<contin_t>& inputs)
    : contin_interpreter(inputs), _use_contin_inputs(true), mixed_inputs(std::vector<vertex>()) {}

vertex mixed_interpreter::operator()(const combo_tree& tr) const
{
    return mixed_eval(tr.begin());
}
vertex mixed_interpreter::operator()(const combo_tree::iterator it) const
{
    return mixed_eval(it);
}

builtin mixed_interpreter::boolean_eval(combo_tree::iterator it) const
{
    return get_builtin(mixed_eval(it));
}

contin_t mixed_interpreter::contin_eval(combo_tree::iterator it) const
{
    return get_contin(mixed_eval(it));
}

vertex mixed_interpreter::mixed_eval(combo_tree::iterator it) const
{
    typedef combo_tree::sibling_iterator sib_it;
    const vertex& v = *it;

    if (const argument* a = boost::get<argument>(&v)) {
        arity_t idx = a->idx;

        // The mixed interpreter could be getting an array of
        // all contins, if the signature of the problem is
        // ->(contin ... contin boolean) or ->(contin ... contin enum_t)
        // so deal with this case.
        // XXX FIXME, we should also handle the cases
        // ->(bool ... bool contin) and ->(bool ... bool enum)
        // which would have an empty contin and an empty mixed ...
        if (_use_contin_inputs)
            return contin_inputs[idx - 1];

        // A negative index means boolean-negate. 
        return idx > 0 ? mixed_inputs[idx - 1]
            : negate_vertex(mixed_inputs[-idx - 1]);
    }
    // mixed, boolean and contin builtin
    else if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {

        // Boolean operators
        case id::logical_false :
        case id::logical_true :
        case id::logical_and :
        case id::logical_or :
        case id::logical_not :
            return boolean_interpreter::boolean_eval(it);

        // Contin operators
        case id::plus :
        case id::times :
        case id::div :
        case id::log :
        case id::exp :
        case id::sin :
        case id::rand :
            return contin_interpreter::contin_eval(it);
            
        // Mixed operators
        case id::greater_than_zero : {
            sib_it sib = it.begin();
            vertex x;
            try {
                // A divide-by-zero in the contin could throw. We want
                // to return a boolean in this case, anyway.  Values
                // might be +inf -inf or nan and we can still get a
                // sign bit off two of these cases...
                x = mixed_eval(sib);
            } catch (OverflowException e) {
                x = e.get_vertex();
            }
            return bool_to_vertex(0 < get_contin(x));
        }

        case id::impulse : {
            vertex i = mixed_eval(it.begin());
            return (i == id::logical_true ? 1.0 : 0.0);
        }

        // XXX TODO: contin_if should go away.
        case id::contin_if :
        case id::cond : {
            sib_it sib = it.begin();
            while (1) {
                OC_ASSERT (sib != it.end(), "Error: mal-formed cond statement");

                vertex vcond = mixed_eval(sib);
                ++sib;  // move past the condition

                // The very last value is the universal "else" clause,
                // taken when none of the previous predicates were true.
                if (sib == it.end())
                    return vcond;

                // If condition is true, then return the consequent
                // (i.e. the value immediately following.) Else, skip
                // the consequent, and loop around again.
                if (vcond == id::logical_true)
                    return mixed_eval(sib);

                ++sib;  // move past the consequent
            }
        }

        default: {
            std::stringstream ss;
            ss << *b;
            OC_ASSERT(false, "mixed_interpreter does not handle builtin %s",
                      ss.str().c_str());
            return vertex();
        }
        }
    }
    // contin constant
    else if (is_contin(v)) {
        return contin_interpreter::contin_eval(it);
    }
    // string constant
    else if (is_definite_object(v)) {
        return v;
    }
    else {
        std::stringstream ss;
        ss << v;
        OC_ASSERT(false, "mixed_interpreter does not handle vertex %s",
                  ss.str().c_str());
        return vertex();
    }
}

}} // ~namespaces combo opencog
