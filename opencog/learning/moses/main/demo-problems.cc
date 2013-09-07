/** 
 * demo-problems.cc ---
 *
 * Copyright (C) 2013 Poulin Holdings LLC
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

#include <opencog/util/Logger.h>
#include <opencog/learning/moses/example-progs/scoring_iterators.h>

#include "problem.h"

namespace opencog { namespace moses {

class majority_problem : public problem_base
{
    public:
        virtual const std::string name() const { return "maj"; }
        virtual void run(problem_params&);
};

void majority_problem::run(problem_params& pms)
{
    if (pms.enable_feature_selection)
        logger().warn("Feature selection is not supported for that problem");

    // @todo: for the moment occam's razor and partial truth table are ignored
    majority func(pms.problem_size);

    // If no exemplar has been provided in the options, use the
    // default boolean_type exemplar (which is 'and').
    if (pms.exemplars.empty()) {
        pms.exemplars.push_back(type_to_exemplar(id::boolean_type));
    }

    type_tree tt = gen_signature(id::boolean_type, pms.arity);
    logical_bscore bscore(func, pms.arity);
    metapop_moses_results(pms.exemplars, tt,
                          pms.bool_reduct, pms.bool_reduct_rep, bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

// XXX hack
problem_base* probm = new majority_problem();

} // ~namespace moses
} // ~namespace opencog

