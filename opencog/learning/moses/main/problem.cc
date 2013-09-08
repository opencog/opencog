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

using namespace reduct;

problem_params::problem_params(const vertex_set& ignore_ops_,
                               int reduct_candidate_effort_,
                               int reduct_knob_building_effort_,
                               const moses_parameters& moses_params_,
                               metapop_printer& mmr_pa_) :
    default_nsamples(20),
    ignore_ops(ignore_ops_),

    // Logical reduction rules used during search.
    lr(logical_reduction(ignore_ops_)),
    bool_reduct(lr(reduct_candidate_effort_)),

    // Logical reduction rules used during representation building.
    bool_reduct_rep(lr(reduct_knob_building_effort_)),

    // Continuous reduction rules used during search and representation
    // building.
    contin_reduct(contin_reduction(reduct_candidate_effort_, ignore_ops)),
    moses_params(moses_params_),
    mmr_pa(mmr_pa_)
{
}


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

