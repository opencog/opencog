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
#include <opencog/comboreduct/table/table_io.h>

#include "../moses/partial.h"

#include "moses_exec_def.h"
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

static void log_output_error_exit(string err_msg) {
    logger().error() << "Error: " << err_msg;
    cerr << "Error: " << err_msg << endl;
    exit(1);
}

// ==================================================================

class table_problem_base : public problem_base
{
protected:
    typedef boost::ptr_vector<bscore_base> BScorerSeq;

    void common_setup(problem_params&);
    void common_type_setup(problem_params&);

    type_tree table_type_signature;
    type_tree table_output_tt;
    type_node table_output_tn;
    type_node output_type;
};

void table_problem_base::common_setup(problem_params& pms)
{
    if (pms.input_data_files.empty()) {
        stringstream ss;
        ss << "no input data file has been specified (option -"
           << input_data_file_opt.second << ")";
        log_output_error_exit(ss.str());
    }

    // Read input data files
    size_t num_rows = 0;
    for (const string& idf : pms.input_data_files) {
        logger().info("Read data file %s", idf.c_str());
        Table table = loadTable(idf, pms.target_feature, pms.ignore_features_str);
        num_rows += table.size();
        // possible subsample the table
        if (pms.nsamples > 0)
            subsampleTable(table, pms.nsamples);
        pms.tables.push_back(table);
        pms.ctables.push_back(table.compressed());
    }
    logger().info("Number of rows in tables = %d", num_rows);

    // Get the labels contained in the data file.
    if (pms.output_with_labels)
        pms.ilabels = pms.tables.front().itable.get_labels();

    pms.arity = pms.tables.front().get_arity();

    // Check that all input data files have the same arity
    if (pms.tables.size() > 1) {
        combo::arity_t test_arity;
        for (size_t i = 1; i < pms.tables.size(); ++i) {
            test_arity = pms.tables[i].get_arity();
            if (test_arity != pms.arity) {
                stringstream ss;
                ss << "File " << pms.input_data_files[0] << " has arity " << pms.arity
                   << " while file " << pms.input_data_files[i] << " has arity " << test_arity;
                log_output_error_exit(ss.str());
            }
        }
    }
    logger().info("Inferred arity = %d", pms.arity);

    pms.mmr_pa.ilabels = pms.ilabels;
}

void table_problem_base::common_type_setup(problem_params& pms)
{
    // Infer the signature based on the input table.
    table_type_signature = pms.tables.front().get_signature();
    logger().info() << "Inferred data signature " << table_type_signature;

    // Infer the type of the input table
    table_output_tt = get_signature_output(table_type_signature);
    table_output_tn = get_type_node(table_output_tt);

    // Determine the default exemplar to start with
    // problem == pre  precision-based scoring
    // precision combo trees always return booleans.
    if (pms.exemplars.empty())
        pms.exemplars.push_back(type_to_exemplar(
            pms.problem == pre? id::boolean_type : table_output_tn));

    // XXX This seems strange to me .. when would the exemplar output
    // type ever differ from the table?  Well,, it could for pre problem,
    // but that's over-ridden later, anyway...
    output_type =
        get_type_node(get_output_type_tree(*pms.exemplars.begin()->begin()));
    if (output_type == id::unknown_type)
        output_type = table_output_tn;

    logger().info() << "Inferred output type: " << output_type;
}

// ==================================================================

/// Find interesting predicates
class ip_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "ip"; }
        virtual const std::string description() const {
             return "Find interesting patterns"; }
        virtual void run(problem_params&);
};

void ip_problem::run(problem_params& pms)
{
    common_setup(pms);
    // ip assumes that the inputs are boolean and the output is contin
    type_tree ettt = gen_signature(id::boolean_type,
                                   id::contin_type, pms.arity);
    OC_ASSERT(ettt == pms.tables.front().get_signature(),
              "The input table doesn't have the right data types."
              " The output should be contin and the inputs should"
              " be boolean");
    // signature of the functions to learn
    type_tree tt = gen_signature(id::boolean_type, pms.arity);

    // determine the default exemplar to start with
    if (pms.exemplars.empty())
        pms.exemplars.push_back(type_to_exemplar(id::boolean_type));

    int as = alphabet_size(tt, pms.ignore_ops);

    typedef interesting_predicate_bscore BScore;
    typedef boost::ptr_vector<bscore_base> BScorerSeq;
    BScorerSeq bscores;
    for (const CTable& ctable : pms.ctables) {
        BScore *r = new BScore(ctable,
                               pms.ip_kld_weight,
                               pms.ip_skewness_weight,
                               pms.ip_stdU_weight,
                               pms.ip_skew_U_weight,
                               pms.min_rand_input,
                               pms.max_rand_input,
                               pms.hardness, pms.hardness >= 0);
        set_noise_or_ratio(*r, as, pms.noise, pms.complexity_ratio);
        bscores.push_back(r);
    }
    multibscore_based_bscore bscore(bscores);
    metapop_moses_results(pms.exemplars, tt,
                          *pms.bool_reduct, *pms.bool_reduct_rep, bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);

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
class ann_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "ann-it"; }
        virtual const std::string description() const {
             return "ANN-based regression on input table"; }
        virtual void run(problem_params&);
};

void ann_table_problem::run(problem_params& pms)
{
    common_setup(pms);
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

// Generic table regression code.  Could be a template, I suppose, but
// the args are variable length, tables is a variable, and scorer is a type,
// and I don't feel like fighting templates to make all three happen just
// exactly right.
#define REGRESSION(OUT_TYPE, REDUCT, REDUCT_REP, TABLES, SCORER, ARGS) \
{                                                                    \
    common_setup(pms);                                               \
    common_type_setup(pms);                                          \
    /* Enable feature selection while selecting exemplar */          \
    if (pms.enable_feature_selection && pms.fs_params.target_size > 0) { \
        /* XXX FIXME: should use the concatenation of all */         \
        /* tables, and not just the first. */                        \
        pms.meta_params.fstor = new feature_selector(TABLES.front(), pms.festor_params); \
    }                                                                \
    /* Keep the table input signature, just make sure */             \
    /* the output is the desired type. */                            \
    type_tree cand_sig = gen_signature(                              \
        get_signature_inputs(table_type_signature),                  \
        type_tree(OUT_TYPE));                                        \
    int as = alphabet_size(cand_sig, pms.ignore_ops);                \
    typedef SCORER BScore;                                           \
    BScorerSeq bscores;                                              \
    for (const auto& table : TABLES) {                               \
        BScore* r = new BScore ARGS ;                                \
        set_noise_or_ratio(*r, as, pms.noise, pms.complexity_ratio); \
        bscores.push_back(r);                                        \
    }                                                                \
    multibscore_based_bscore bscore(bscores);                        \
    metapop_moses_results(pms.exemplars, cand_sig,                   \
                      REDUCT, REDUCT_REP, bscore,                    \
                      pms.opt_params, pms.hc_params, pms.meta_params, \
                      pms.moses_params, pms.mmr_pa);                 \
}

// ==================================================================
/// precision-based scoring
// regression based on input table by maximizing precision (or negative
// predictive value), holding activation const.

class pre_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "pre"; }
        virtual const std::string description() const {
             return "Precision-Activation scoring"; }
        virtual void run(problem_params&);
};

void pre_table_problem::run(problem_params& pms)
{
    // Very nearly identical to the REGRESSION macro above,
    // except that some new experimental features are being tried.
    common_setup(pms);
    common_type_setup(pms);

    // Keep the table input signature, just make sure the
    // output is a boolean.
    type_tree cand_sig = gen_signature(
        get_signature_inputs(table_type_signature),
        type_tree(id::boolean_type));
    int as = alphabet_size(cand_sig, pms.ignore_ops);
    typedef precision_bscore BScore;
    BScorerSeq bscores;
    for (const CTable& ctable : pms.ctables) {
        BScore* r = new BScore(ctable,
                               fabs(pms.hardness),
                               pms.min_rand_input,
                               pms.max_rand_input,
                               pms.hardness >= 0,
                               pms.pre_worst_norm);
        set_noise_or_ratio(*r, as, pms.noise, pms.complexity_ratio);
        bscores.push_back(r);
        if (pms.gen_best_tree) {
            // experimental: use some canonically generated
            // candidate as exemplar seed
            combo_tree tr = r->gen_canonical_best_candidate();
            logger().info() << "Canonical program tree (non reduced) maximizing precision = " << tr;
            pms.exemplars.push_back(tr);
        }
    }

    // Enable feature selection while selecting exemplar
    if (pms.enable_feature_selection && pms.fs_params.target_size > 0) {
        // XXX FIXME should use the concatenation of all ctables, not just first
        pms.meta_params.fstor = new feature_selector(pms.ctables.front(),
                                                 pms.festor_params);
    }

    multibscore_based_bscore bscore(bscores);
    metapop_moses_results(pms.exemplars, cand_sig,
                          *pms.bool_reduct, *pms.bool_reduct_rep, bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);

}

// ==================================================================
/// precision-based scoring (maximizing number of conjunctions)
class pre_conj_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "pre-conj"; }
        virtual const std::string description() const {
             return "Precision-Conjunction-Maximization"; }
        virtual void run(problem_params&);
};

void pre_conj_table_problem::run(problem_params& pms)
{
    // Very nearly identical to the REGRESSION macro above,
    // except that some new experimental features are being tried.
    common_setup(pms);
    common_type_setup(pms);

    // Keep the table input signature, just make sure the
    // output is a boolean.
    type_tree cand_sig = gen_signature(
        get_signature_inputs(table_type_signature),
        type_tree(id::boolean_type));
    int as = alphabet_size(cand_sig, pms.ignore_ops);
    typedef precision_conj_bscore BScore;
    BScorerSeq bscores;
    for (const CTable& ctable : pms.ctables) {
        BScore* r = new BScore(ctable,
                               fabs(pms.hardness),
                               pms.hardness >= 0);
        set_noise_or_ratio(*r, as, pms.noise, pms.complexity_ratio);
        bscores.push_back(r);
    }

    // Enable feature selection while selecting exemplar
    if (pms.enable_feature_selection && pms.fs_params.target_size > 0) {
        // XXX FIXME should use the concatenation of all ctables, not just first
        pms.meta_params.fstor = new feature_selector(pms.ctables.front(),
                                                 pms.festor_params);
    }
    multibscore_based_bscore bscore(bscores);
    metapop_moses_results(pms.exemplars, cand_sig,
                          *pms.bool_reduct, *pms.bool_reduct_rep, bscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

// ==================================================================
/// maximize precision, holding recall const.
class prerec_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "prerec"; }
        virtual const std::string description() const {
             return "Precision Maximization (holding recall constant)"; }
        virtual void run(problem_params&);
};

void prerec_table_problem::run(problem_params& pms)
{
    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.5;
        pms.max_rand_input = 1.0; }
    REGRESSION(id::boolean_type,
               *pms.bool_reduct, *pms.bool_reduct_rep,
               pms.ctables, prerec_bscore,
               (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

// ==================================================================
/// maximize recall, holding precision const.
class recall_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "recall"; }
        virtual const std::string description() const {
             return "Recall Maximization (holding precision constant)"; }
        virtual void run(problem_params&);
};

void recall_table_problem::run(problem_params& pms)
{
    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.8;
        pms.max_rand_input = 1.0; }
    REGRESSION(id::boolean_type,
               *pms.bool_reduct, *pms.bool_reduct_rep,
               pms.ctables, recall_bscore,
               (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

// ==================================================================
/// bep == beak-even point between bep and precision.
class bep_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "bep"; }
        virtual const std::string description() const {
             return "Maximize Break-even Point"; }
        virtual void run(problem_params&);
};

void bep_table_problem::run(problem_params& pms)
{
    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.0;
        pms.max_rand_input = 0.5; }
    REGRESSION(id::boolean_type,
               *pms.bool_reduct, *pms.bool_reduct_rep,
               pms.ctables, bep_bscore,
               (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

// ==================================================================
/// f_one = F_1 harmonic mean of recall and precision
class f_one_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "f_one"; }
        virtual const std::string description() const {
             return "Maximize F_1 score"; }
        virtual void run(problem_params&);
};

void f_one_table_problem::run(problem_params& pms)
{
        REGRESSION(id::boolean_type,
                   *pms.bool_reduct, *pms.bool_reduct_rep,
                   pms.ctables, f_one_bscore,
                   (table));
}

// ==================================================================

/// Maximize accuracy
class it_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "it"; }
        virtual const std::string description() const {
             return "Maximize Accuracy"; }
        virtual void run(problem_params&);
};

void it_table_problem::run(problem_params& pms)
{
    // problem == it  i.e. input-table based scoring.
    OC_ASSERT(output_type == table_output_tn);

    // --------- Boolean output type
    if (output_type == id::boolean_type) {
        REGRESSION(output_type,
                   *pms.bool_reduct, *pms.bool_reduct_rep,
                   pms.ctables, ctruth_table_bscore,
                   (table));
    }

    // --------- Enumerated output type
    else if (output_type == id::enum_type) {

        // For enum targets, like boolean targets, the score
        // can never exceed zero (perfect score).
        // XXX Eh ??? for precision/recall scorers,
        // the score range is 0.0 to 1.0 so this is wrong...
        if (0.0 < pms.moses_params.max_score) {
            pms.moses_params.max_score = 0.0;
            pms.opt_params.terminate_if_gte = 0.0;
        }
        if (pms.use_well_enough) {
            // The "leave well-enough alone" algorithm.
            // Works. Kind of. Not as well as hoped.
            // Might be good for some problem. See diary.
            partial_solver well(pms.ctables,
                            table_type_signature,
                            pms.exemplars, *pms.contin_reduct,
                            pms.opt_params, pms.hc_params, pms.meta_params,
                            pms.moses_params, pms.mmr_pa);
            well.solve();
        } else {
            // Much like the boolean-output-type above,
            // just uses a slightly different scorer.
            REGRESSION(output_type,
                       *pms.contin_reduct, *pms.contin_reduct,
                       pms.ctables, enum_effective_bscore,
                       (table));
        }
    }

    // --------- Contin output type
    else if (output_type == id::contin_type) {
        if (pms.discretize_thresholds.empty()) {

            contin_bscore::err_function_type eft =
                pms.it_abs_err ? contin_bscore::abs_error :
                contin_bscore::squared_error;

            REGRESSION(output_type,
                       *pms.contin_reduct, *pms.contin_reduct,
                       pms.tables, contin_bscore,
                       (table, eft));

        } else {
            REGRESSION(output_type,
                       *pms.contin_reduct, *pms.contin_reduct,
                       pms.tables, discretize_contin_bscore,
                       (table.otable, table.itable,
                            pms.discretize_thresholds, pms.weighted_accuracy));
        }
    }

    // --------- Unknown output type
    else {
        logger().error() << "Error: type " << type_tree(output_type)
            << " currently not supported.";
        std::cerr << "Error: type " << type_tree(output_type)
            << " currently not supported." << std::endl;
        exit(1);
    }
}

// ==================================================================

void register_table_problems()
{
	register_problem(new ip_problem());
	register_problem(new ann_table_problem());
	register_problem(new pre_table_problem());
	register_problem(new pre_conj_table_problem());
	register_problem(new prerec_table_problem());
	register_problem(new recall_table_problem());
	register_problem(new bep_table_problem());
	register_problem(new f_one_table_problem());
	register_problem(new it_table_problem());
}

} // ~namespace moses
} // ~namespace opencog

