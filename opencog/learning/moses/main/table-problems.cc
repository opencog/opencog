/**
 * table-problems.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012 Poulin Holdings LLC
 * Copyright (C) 2013 Linas Vepstas
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
 *         Linas Vepstas <linasvepstas@gmail.com>
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

#include "problem.h"
#include "table-problems.h"

namespace opencog { namespace moses {

// XXX FIXME protytpe later
unsigned alphabet_size(const type_tree& tt, const vertex_set ignore_ops);

//XXX template should be moved to header file somwhere ... 
// set the complexity ratio.
template <typename BScorer>
void set_noise_or_ratio(BScorer& scorer, unsigned as, float noise, score_t ratio)
{
    if (noise >= 0.0)
        scorer.set_complexity_coef(as, noise);
    else
        scorer.set_complexity_coef(ratio);
}


// ==================================================================

static combo_tree ann_exemplar(combo::arity_t arity)
{
    combo_tree ann_tr(ann_type(0, id::ann));
    // ann root
    combo_tree::iterator root_node = ann_tr.begin();
    // output node
    combo_tree::iterator output_node =
        ann_tr.append_child(root_node, ann_type(1, id::ann_node));
    // input nodes
    for (combo::arity_t i = 0; i <= arity; ++i)
        ann_tr.append_child(output_node, ann_type(i + 2, id::ann_input));
    // input nodes' weights
    ann_tr.append_children(output_node, 0.0, arity + 1);

    return ann_tr;
}

/// Regression based on combo program using ann
class ann_table_problem : public problem_base
{
    public:
        virtual const std::string name() const { return "ann-it"; }
        virtual const std::string description() const {
             return "ANN-based regression on input table"; }
        virtual void run(problem_params&);
};

void ann_table_problem::run(problem_params& pms)
{
    // If no exemplar has been provided in the options,
    // insert the default.
    if (pms.exemplars.empty()) {
        pms.exemplars.push_back(ann_exemplar(pms.arity));
    }

    type_tree tt = gen_signature(id::ann_type, 0);
    int as = alphabet_size(tt, pms.ignore_ops);

    contin_bscore bscore(pms.tables.front());
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);
    metapop_moses_results(pms.exemplars, tt,
                          reduct::ann_reduction(), reduct::ann_reduction(), bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

// ==================================================================

void register_table_problems()
{
	register_problem(new ann_table_problem());
}

} // ~namespace moses
} // ~namespace opencog

