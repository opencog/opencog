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
#include <vector>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/learning/moses/metapopulation/metapop_params.h>
#include <opencog/learning/moses/moses/moses_main.h>
#include <opencog/learning/moses/moses/moses_params.h>
#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/learning/moses/optimization/hill-climbing.h>


namespace opencog { namespace moses {

struct problem_params
{
    problem_params(const reduct::rule&, const reduct::rule&,
                   const reduct::rule&,
                   const moses_parameters&,
                   metapop_printer&);

    // default number of samples to describe a problem
    const unsigned int default_nsamples;
    int nsamples;
    vertex_set ignore_ops; // should be const&
    unsigned int problem_size;

    float noise;
    score_t complexity_ratio;

    // it params
    bool it_abs_err;

    // EXPERIMENTAL
    // feature selection happens before each representation building
    /// Enable feature selection while selecting exemplar
    bool enable_feature_selection;
    std::vector<combo_tree> exemplars;
    combo::arity_t arity;

    const reduct::rule& bool_reduct;
    const reduct::rule& bool_reduct_rep;
    const reduct::rule& contin_reduct;

    optim_parameters opt_params; // XXX should be const
    hc_parameters hc_params;
    const moses_parameters& moses_params;
    metapop_parameters meta_params;
    metapop_printer& mmr_pa;
};

class problem_base
{
    public:
        virtual ~problem_base() {}
        virtual const std::string name() const = 0;
        virtual const std::string description() const = 0;
        virtual combo::arity_t arity(size_t) = 0;
        virtual void run(problem_params&) = 0;
};

void register_problem(problem_base*);
problem_base* find_problem(const string&);



} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_PROBLEM_H
