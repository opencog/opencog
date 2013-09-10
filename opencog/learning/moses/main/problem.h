/**
 * problem.h ---
 *
 * Copyright (C) 2013 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_MOSES_PROBLEM_H
#define _OPENCOG_MOSES_PROBLEM_H

#include <string>
#include <opencog/learning/moses/main/problem-params.h>

namespace opencog { namespace moses {

class problem_base
{
    public:
        virtual ~problem_base() {}
        virtual const std::string name() const = 0;
        virtual const std::string description() const = 0;
        virtual void run(problem_params&) = 0;
};

void register_problem(problem_base*);
problem_base* find_problem(const std::string&);

unsigned alphabet_size(const type_tree& tt, const vertex_set ignore_ops);


} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_PROBLEM_H
