/**
 * problem.cc ---
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

#include "problem.h"

namespace opencog { namespace moses {

// =================================================================
// Register and find problems, by name, so that they can be run.

map<std::string, problem_base*> problem_set;

void register_problem(problem_base* prob)
{
    std::pair<std::string, problem_base*> pr(prob->name(), prob);
    problem_set.insert(pr);
}

problem_base* find_problem(const string& name)
{
    auto it = problem_set.find(name);
    if (it != problem_set.end())
        return it->second;
    return NULL;
}


} // ~namespace moses
} // ~namespace opencog

