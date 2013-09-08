/**
 * demo-problems.cc ---
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
#include <opencog/learning/moses/example-progs/scoring_iterators.h>

#include "problem.h"
#include "demo-problems.h"

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
/// Demo/example problem: majority. Learn the combo program that
/// return true iff the number of true arguments is strictly
/// greater than half of the arity

class majority_problem : public problem_base
{
    public:
        virtual const std::string name() const { return "maj"; }
        virtual const std::string description() const {
             return "Majority problem demo"; }
        virtual combo::arity_t arity(size_t sz) { return sz; }
        virtual void run(problem_params&);
};

void majority_problem::run(problem_params& pms)
{
    if (pms.enable_feature_selection)
        logger().warn("Feature selection is not supported for the majority problem");

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

// ==================================================================
/// Demo/Example problem: polynomial regression.  Given the polynomial
/// p(x)=x+x^2+x^3+...x^k, this searches for the  shortest  program
/// consisting  of  nested arithmetic operators to compute p(x),
/// given x as a free variable.  So, for example the order-2 polynomial
/// can be written as x+x^2, and the shortest combo program is
/// *(+(1 $1) $1) (that is, the  solution is p(x)=x(x+1) in the usual
/// arithmetical notation).

class polynomial_problem : public problem_base
{
    public:
        virtual const std::string name() const { return "sr"; }
        virtual const std::string description() const {
             return "Simple regression of f_n(x) = sum_{k={1,n}} x^k"; }
        virtual combo::arity_t arity(size_t sz) { return 1; }
        virtual void run(problem_params&);
};

void polynomial_problem::run(problem_params& pms)
{
    if (pms.enable_feature_selection)
        logger().warn("Feature selection is not supported for the polynomial problem");

    // If no exemplar has been provided in the options, use the
    // default contin_type exemplar (+)
    if (pms.exemplars.empty()) {
        pms.exemplars.push_back(type_to_exemplar(id::contin_type));
    }

    // sr is fundamentally a kind of non-linear regression!
    // over-ride any flag settings regarding this.
    pms.meta_params.linear_contin = false;

    type_tree tt = gen_signature(id::contin_type, pms.arity);

    ITable it(tt, (pms.nsamples>0 ? pms.nsamples : pms.default_nsamples));

    int as = alphabet_size(tt, pms.ignore_ops);

    contin_bscore::err_function_type eft =
        pms.it_abs_err ? contin_bscore::abs_error :
        contin_bscore::squared_error;
    contin_bscore bscore(simple_symbolic_regression(pms.problem_size),
                         it, eft);

    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);
    metapop_moses_results(pms.exemplars, tt,
                          pms.contin_reduct, pms.contin_reduct, bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

// ==================================================================
void register_demo_problems()
{
	register_problem(new majority_problem());
	register_problem(new polynomial_problem());
}

} // ~namespace moses
} // ~namespace opencog

