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
// Boolean Demo/example problem common code.

class bool_problem_base : public problem_base
{
    public:
        virtual void run(problem_params&);
        virtual logical_bscore get_bscore(arity_t) = 0;
};

void bool_problem_base::run(problem_params& pms)
{
    if (pms.enable_feature_selection)
        logger().warn("Feature selection is not supported for the demo problems");

    // If no exemplar has been provided in the options, use the
    // default boolean_type exemplar (which is 'and').
    if (pms.exemplars.empty()) {
        pms.exemplars.push_back(type_to_exemplar(id::boolean_type));
    }

    logical_bscore bscore = get_bscore(pms.problem_size);

    type_tree sig = gen_signature(id::boolean_type, pms.arity);
    unsigned as = alphabet_size(sig, pms.ignore_ops);
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);

    metapop_moses_results(pms.exemplars, sig,
                          pms.bool_reduct, pms.bool_reduct_rep, bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

// ==================================================================
// Demo/Example: learn a combo program that determines if the
// program inputs are even parity or not.  That is, the combo
// program will be a boolean circuit that computes parity.
class pa_problem : public bool_problem_base
{
    public:
        virtual const std::string name() const { return "pa"; }
        virtual const std::string description() const {
             return "Learn parity function demo"; }
        virtual combo::arity_t arity(size_t sz) { return sz; }
        virtual logical_bscore get_bscore(int problem_size)
        {
            even_parity func;
            logical_bscore bscore(func, problem_size);
            return bscore;
        }
};


// ==================================================================
// Demo/example problem: learn the logical disjunction. That is,
// moses should learn the following program: or($1 $2 ... $k) where
// k is the number of inputs specified by the -k option.
class dj_problem : public bool_problem_base
{
    public:
        virtual const std::string name() const { return "dj"; }
        virtual const std::string description() const {
             return "Learn logicical disjunction demo"; }
        virtual combo::arity_t arity(size_t sz) { return sz; }
        virtual logical_bscore get_bscore(int problem_size)
        {
            disjunction func;
            logical_bscore bscore(func, problem_size);
            return bscore;
        }
};


// ==================================================================
/// Demo/example problem: majority. Learn the combo program that
/// return true iff the number of true arguments is strictly
/// greater than half of the arity

class majority_problem : public bool_problem_base
{
    public:
        virtual const std::string name() const { return "maj"; }
        virtual const std::string description() const {
             return "Majority problem demo"; }
        virtual combo::arity_t arity(size_t sz) { return sz; }
        virtual logical_bscore get_bscore(int problem_size)
        {
            majority func(problem_size);
            logical_bscore bscore(func, problem_size);
            return bscore;
        }
};

// ==================================================================

/// Demo/example problem: multiplex. Learn the combo program that
/// corresponds to the boolean (electrical) circuit that is a
/// (de-)multiplexer.  That is, a k-bit binary address will specify
/// one and exactly one wire out of 2^k wires.  Here, k==problem_size.

class mux_problem : public bool_problem_base
{
    public:
        virtual const std::string name() const { return "mux"; }
        virtual const std::string description() const {
            return "Multiplex problem demo"; }
        virtual combo::arity_t arity(size_t sz) {
            return sz + pow2(sz);
        }
        virtual logical_bscore get_bscore(int problem_size)
        {
            multiplex func(problem_size);
            logical_bscore bscore(func, arity(problem_size));
            return bscore;
        }
};

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
// Demo/Example: Problem based on input combo program.
// Learn a program that should be identical to the specified input
// program.

static combo_tree str_to_combo_tree(const string& combo_str)
{
    stringstream ss;
    combo_tree tr;
    ss << combo_str;
    ss >> tr;
    return tr;
}

class combo_problem_base : public problem_base
{
    public:
        virtual combo::arity_t arity(size_t sz) { return -1; }
        void check_args(problem_params&);
        //* Convert string to a combo_tree
};

void combo_problem_base::check_args(problem_params& pms)
{
    if (pms.enable_feature_selection)
        logger().warn("Feature selection is not supported for the combo demo.");

    if (pms.combo_str.empty()) {
        logger().warn() << "You must specify a combo tree to learn (option -y).";
        std::cerr << "You must specify a combo tree to learn (option -y)." << std::endl;
        exit(-1);
    }

    // get the combo_tree and infer its type
    combo_tree tr = str_to_combo_tree(pms.combo_str);
    type_tree tt = infer_type_tree(tr);
    if (not is_well_formed(tt)) {
        logger().warn() << "The combo tree " << tr << " is not well formed.";
        std::cerr << "The combo tree " << tr << " is not well formed." << std::endl;
        exit(-1);
    }
    
    combo::arity_t arity = type_tree_arity(tt);

    // If the user specifies the combo program from bash or similar
    // shells, and forgets to escape the $ in the variable names,
    // then the resulting combo program will be garbage.  Try to
    // sanity-check this, so as to avoid user frustration.
    // A symptom of this error is that the arity will be -1.
    if (-1 == arity || NULL == strchr(pms.combo_str.c_str(), '$')) {
        cerr << "Error: the combo program " << tr << "\n"
             << "appears not to contain any arguments. Did you\n"
             << "forget to escape the $'s in the shell command line?"
             << endl;
        exit(-2);
    }
}

//* Get largest contin constant in a combo tree
static contin_t largest_const_in_tree(const combo_tree &tr)
{
    contin_t rc = 0.0;
    combo_tree::pre_order_iterator it;
    for(it = tr.begin(); it != tr.end(); it++) {
        if (is_contin(*it)) {
            contin_t val = get_contin(*it);
            if (rc < val) rc = val;
        }
    }

    return rc;
}

class combo_problem : public combo_problem_base
{
    public:
        virtual const std::string name() const { return "cp"; }
        virtual const std::string description() const {
             return "Demo: Learn a given combo program"; }
        virtual void run(problem_params&);
};

void combo_problem::run(problem_params& pms)
{
    check_args(pms);

    // get the combo_tree and infer its type
    combo_tree tr = str_to_combo_tree(pms.combo_str);
    type_tree tt = infer_type_tree(tr);
    type_node output_type = get_type_node(get_signature_output(tt));
    combo::arity_t arity = type_tree_arity(tt);

    // If no exemplar has been provided in the options, use the
    // default one.
    if (pms.exemplars.empty()) {
        pms.exemplars.push_back(type_to_exemplar(output_type));
    }

    if (output_type == id::boolean_type) {
        // @todo: Occam's razor and nsamples is not taken into account
        logical_bscore bscore(tr, arity);
        metapop_moses_results(pms.exemplars, tt,
                              pms.bool_reduct, pms.bool_reduct_rep, bscore,
                              pms.opt_params, pms.hc_params, pms.meta_params,
                              pms.moses_params, pms.mmr_pa);
    }
    else if (output_type == id::contin_type) {

        // Naive users of the combo regression mode will fail
        // to understand that the input program must be sampled
        // in order for the fitness function to be evaluated.
        // The default sample range 0<x<1 is probably too small
        // for any fancy-pants input program, so try to make
        // a reasonable guess.  Yes, this is a stupid hack, but
        // it does avoid the problem of naive users saying
        // "aww moses sucks" when they fail to invoke it correctly.
        if ((0.0 == pms.min_rand_input) && (1.0 == pms.max_rand_input)) {
            pms.max_rand_input = 2.0 * largest_const_in_tree(tr);
            pms.min_rand_input = -pms.max_rand_input;
            if ((pms.nsamples <= 0) &&
                (pms.default_nsamples < 2 * arity * pms.max_rand_input)) {
                pms.nsamples = 2 * arity * pms.max_rand_input;
            }
        }

        if (pms.nsamples <= 0)
            pms.nsamples = pms.default_nsamples;

        logger().info() << "Will sample combo program " << tr << "\n"
                        << "\tat " << pms.nsamples << " input values, "
                        << "ranging between " << pms.min_rand_input
                        << " and " << pms.max_rand_input;

        // @todo: introduce some noise optionally
        ITable it(tt, pms.nsamples, pms.max_rand_input, pms.min_rand_input);
        OTable ot(tr, it);

        int as = alphabet_size(tt, pms.ignore_ops);

        contin_bscore bscore(ot, it);
        set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);
        metapop_moses_results(pms.exemplars, tt,
                              pms.contin_reduct, pms.contin_reduct, bscore,
                              pms.opt_params, pms.hc_params, pms.meta_params,
                              pms.moses_params, pms.mmr_pa);
    } else {
        logger().error() << "Error: combo_problem: type " << tt << " not supported.";
        std::cerr << "Error: combo_problem: type " << tt << " not supported." << std::endl;
        exit(-1);
    }
}

/// Regression based on combo program using ann
class ann_combo_problem : public combo_problem_base
{
    public:
        virtual const std::string name() const { return "ann-cp"; }
        virtual const std::string description() const {
             return "Demo: Learn a given combo program using ANN"; }
        virtual void run(problem_params&);
};

void ann_combo_problem::run(problem_params& pms)
{
    check_args(pms);

    // get the combo_tree and infer its type
    combo_tree tr = str_to_combo_tree(pms.combo_str);
    type_tree tt = infer_type_tree(tr);
    // type_tree tt = gen_signature(id::ann_type, 0);

    // If no exemplar has been provided in the options, use the
    // default one.
    if (pms.exemplars.empty()) {
        type_node output_type = get_type_node(get_signature_output(tt));
        pms.exemplars.push_back(type_to_exemplar(output_type));
    }

    if (pms.nsamples <= 0)
        pms.nsamples = pms.default_nsamples;

    ITable it(tt, pms.nsamples, pms.max_rand_input, pms.min_rand_input);
    OTable ot(tr, it);

    contin_bscore bscore(ot, it);
    int as = alphabet_size(tt, pms.ignore_ops);
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);

    metapop_moses_results(pms.exemplars, tt,
                          pms.contin_reduct, pms.contin_reduct, bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

// ==================================================================

void register_demo_problems()
{
	register_problem(new pa_problem());
	register_problem(new dj_problem());
	register_problem(new majority_problem());
	register_problem(new mux_problem());
	register_problem(new polynomial_problem());
	register_problem(new combo_problem());
	register_problem(new ann_combo_problem());
}

} // ~namespace moses
} // ~namespace opencog

