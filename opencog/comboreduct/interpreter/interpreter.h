/** interpreter.h --- 
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


#ifndef _OPENCOG_INTERPRETER_H
#define _OPENCOG_INTERPRETER_H

#include "../combo/vertex.h"

/**
 * Attempt to rewrite the combo interpreter into something modular and efficient.
 *
 * There are several interpreter each dealing with its data type. One
 * can use a sub-interpreter for a specific data type at a lower cost
 * of storing the variable content and avoiding the overhead of
 * building a vertex or a combo_tree.
 */

namespace opencog { namespace combo {

/**
 * Interpreter for boolean expressions.
 *
 * We use builtin as inputs and output because boolean contants are
 * encoded as builtins id::logical_true id::logical_false.
 */
struct boolean_interpreter
{
    // ctor
    boolean_interpreter(const std::vector<builtin>& inputs = std::vector<builtin>());

    // interpreters
    builtin operator()(const combo_tree& tr) const;
    builtin operator()(const combo_tree::iterator it) const;
    virtual builtin boolean_eval(combo_tree::iterator it) const;

protected:
    const std::vector<builtin>& boolean_inputs;
};

/**
 * Interpreter for contin expressions.
 *
 * We use contin as inputs and output
 */
struct contin_interpreter
{
    // ctor
    contin_interpreter(const std::vector<contin_t>& inputs = std::vector<contin_t>());

    // interpreters
    contin_t operator()(const combo_tree& tr) const;
    contin_t operator()(const combo_tree::iterator it) const;
    virtual contin_t contin_eval(combo_tree::iterator it) const;

protected:
    const std::vector<contin_t>& contin_inputs;
};

struct mixed_interpreter : public boolean_interpreter, public contin_interpreter
{
    // ctor
    mixed_interpreter(const std::vector<vertex>& inputs = std::vector<vertex>());
    mixed_interpreter(const std::vector<contin_t>& inputs);

    // interpreters
    vertex operator()(const combo_tree& tr) const;
    vertex operator()(const combo_tree::iterator it) const;
    virtual builtin boolean_eval(combo_tree::iterator it) const;
    virtual contin_t contin_eval(combo_tree::iterator it) const;
    virtual vertex mixed_eval(combo_tree::iterator it) const;

protected:
    bool _use_contin_inputs;
    const std::vector<vertex>& mixed_inputs;
};            

}} // ~namespaces combo opencog
        
#endif // _OPENCOG_INTERPRETER_H
