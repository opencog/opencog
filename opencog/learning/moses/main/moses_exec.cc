/** moses_exec.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012 Poulin Holdings LLC
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

#include <signal.h>
#include <unistd.h>

#include <boost/range/algorithm/transform.hpp>
#include <boost/lexical_cast.hpp>

#include <opencog/util/log_prog_name.h>
#include <opencog/util/numeric.h>

#include <opencog/comboreduct/table/table.h>

#include "moses_exec.h"
#include "demo-problems.h"
#include "problem.h"

#include "../moses/moses_main.h"
#include "../moses/partial.h"
#include "../optimization/hill-climbing.h"
#include "../optimization/optimization.h"
#include "../example-progs/scoring_iterators.h"



namespace opencog { namespace moses {

using boost::lexical_cast;
using namespace std;
using namespace reduct;


// problems
static const string it="it"; // regression based on input table
                             // maximize accuracy.

static const string recall="recall"; // regression based on input table,
                                  // maximize recall, while holding
                                  // precision const.

static const string prerec="prerec"; // regression based on input table,
                                  // maximize precision, while holding
                                  // recall const.

static const string bep="bep";    // regression based on input table,
                                  // maximize break-even point

static const string f_one="f_one"; // regression based on input table,
                                  // maximize f_1 score

static const string pre="pre";    // regression based on input table by
                                  // maximizing precision (or negative
                                  // predictive value), holding activation
                                  // const.

static const string pre_conj="pre_conj";    // simplified version of
                                            // pre, but tries to
                                            // maximize number of
                                            // conjunctions

static const string ip="ip"; // find interesting patterns

/* ANN problems */
static const string ann_it="ann-it"; // regression based on input table using ann

static void log_output_error_exit(string err_msg) {
    logger().error() << "Error: " << err_msg;
    cerr << "Error: " << err_msg << endl;
    exit(1);    
}

/**
 * Display error message about unsupported type and exit
 */
static void unsupported_type_exit(const type_tree& tt)
{
    stringstream ss;
    ss << "type " << tt << " currently not supported.";
    log_output_error_exit(ss.str());
}

static void unsupported_type_exit(type_node type)
{
    unsupported_type_exit(type_tree(type));
}

/**
 * Display error message about unsupported problem and exit
 */
void unsupported_problem_exit(const string& problem)
{
    stringstream ss;
    ss << "problem type \"" << problem << "\" is currently unsupported.";
    log_output_error_exit(ss.str());
}


combo_tree ann_exemplar(combo::arity_t arity)
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

static bool contains_type(const type_tree_pre_it it, id::type_node ty)
{
    for (type_tree_sib_it tts = it.begin(); tts != it.end(); tts++)
        if (*tts == ty) return true;
    return false;
}

/**
 * Determine the alphabet size given the type_tree of the problem and
 * a list of operators to ignore.  This is a somewhat ad-hoc value,
 * meant to help compute the complexity ratio.
 */
unsigned alphabet_size(const type_tree& tt, const vertex_set ignore_ops)
{
    unsigned as = 0;
    // arity will be zero for anything that isn't a lambda_type.
    // However, all tables will be lambda_type ...
    combo::arity_t arity = type_tree_arity(tt);
    type_node output_type = get_type_node(get_signature_output(tt));

    switch (output_type) {
        case id::boolean_type:
            // 3 operators: and, or, not
            as = 3 + arity;
            break;

        case id::contin_type:
            // Set alphabet size, 8 is roughly the number of operators
            // in contin formula, it will have to be adapted
            // Well, there could be a mix of booleans too.
            as = 8 + arity - ignore_ops.size();
            break;

        case id::enum_type: {

            // Try to take into account a table with both booleans and
            // contins.
            type_tree_pre_it ttl = tt.begin();

            as = arity + enum_t::size();

            if (contains_type(ttl, id::contin_type))
                as += 8 - ignore_ops.size();

            if (contains_type(ttl, id::boolean_type))
                as += 3;

            break;
        }
        case id::ann_type:

            as = 2 + arity*arity; // to account for hidden neurons, very roughly
            break;

        case id::lambda_type:
        case id::application_type:
        case id::union_type:
        case id::arg_list_type:
        case id::action_result_type:
        case id::definite_object_type:
        case id::action_definite_object_type:
        case id::indefinite_object_type:
        case id::message_type:
        case id::action_symbol_type:
        case id::wild_card_type:
        case id::unknown_type:
        case id::ill_formed_type:
        default:
            unsupported_type_exit(tt);
    }

    logger().info() << "Alphabet size = " << as
                    << " output = " << output_type;
    return as;
}

// set the complexity ratio.
template <typename BScorer>
void set_noise_or_ratio(BScorer& scorer, unsigned as, float noise, score_t ratio)
{
    if (noise >= 0.0)
        scorer.set_complexity_coef(as, noise);
    else
        scorer.set_complexity_coef(ratio);
}

//* return true iff the problem is based on data file
bool datafile_based_problem(const string& problem)
{
    static set<string> dbp
        = {it, pre, pre_conj, recall, prerec, bep, f_one, ann_it, ip};
    return dbp.find(problem) != dbp.end();
}

int moses_exec(int argc, char** argv)
{
    problem_params pms;
    pms.parse_options(argc, argv);

    register_demo_problems();

    problem_base* probm = find_problem(pms.problem);
    if (probm)
    {
        probm->run(pms);
        exit(0);
    }

    // Problem based on input table.
    if (not datafile_based_problem(pms.problem))
        unsupported_problem_exit(pms.problem);

    typedef boost::ptr_vector<bscore_base> BScorerSeq;

    // Infer the signature based on the input table.
    const type_tree& table_type_signature = pms.tables.front().get_signature();
    logger().info() << "Inferred data signature " << table_type_signature;

    // 'it' means regression based on input table; we maximimze accuracy.
    // 'pre' means we must maximize precision (i.e minimize the number of
    // false positives) while holding activation fixed.
    // 'prerec' means we must maximize precision (i.e minimize the number of
    // false positives) while holding recall fixed.
    // 'recall' means we must maximize recall while holding precision fixed.
    // 'f_one' means we must maximize the f_one score while holding ratio const
    // 'bep' means we must maximize the break-even point while holding difference const
    if (pms.problem == it || pms.problem == pre || pms.problem == pre_conj ||
        pms.problem == prerec || pms.problem == recall ||
        pms.problem == f_one || pms.problem == bep)
    {
        // Infer the type of the input table
        type_tree table_output_tt = get_signature_output(table_type_signature);
        type_node table_output_tn = get_type_node(table_output_tt);

        // Determine the default exemplar to start with
        // problem == pre  precision-based scoring
        // precision combo trees always return booleans.
        if (pms.exemplars.empty())
            pms.exemplars.push_back(type_to_exemplar(
                pms.problem == pre? id::boolean_type : table_output_tn));

        // XXX This seems strange to me .. when would the exemplar output
        // type ever differ from the table?  Well,, it could for pre problem,
        // but that's over-ridden later, anyway...
        type_node output_type =
            get_type_node(get_output_type_tree(*pms.exemplars.begin()->begin()));
        if (output_type == id::unknown_type)
            output_type = table_output_tn;

        logger().info() << "Inferred output type: " << output_type;

// Generic table regression code.  Could be a template, I suppose, but
// the args are variable length, tables is a variable, and scorer is a type,
// and I don't feel like fighting templates to make all three happen just
// exactly right.
#define REGRESSION(OUT_TYPE, REDUCT, REDUCT_REP, TABLES, SCORER, ARGS) \
{                                                            \
/* Enable feature selection while selecting exemplar */      \
if (pms.enable_feature_selection && pms.fs_params.target_size > 0) { \
    /* XXX FIXME: should use the concatenation of all */     \
    /* tables, and not just the first. */                    \
    pms.meta_params.fstor = new feature_selector(TABLES.front(), pms.festor_params); \
}                                                            \
/* Keep the table input signature, just make sure */         \
/* the output is the desired type. */                        \
type_tree cand_sig = gen_signature(                          \
    get_signature_inputs(table_type_signature),              \
    type_tree(OUT_TYPE));                                    \
int as = alphabet_size(cand_sig, pms.ignore_ops);            \
typedef SCORER BScore;                                       \
BScorerSeq bscores;                                          \
for (const auto& table : TABLES) {                           \
    BScore* r = new BScore ARGS ;                            \
    set_noise_or_ratio(*r, as, pms.noise, pms.complexity_ratio); \
    bscores.push_back(r);                                    \
}                                                            \
multibscore_based_bscore bscore(bscores);                    \
metapop_moses_results(pms.exemplars, cand_sig,               \
                      REDUCT, REDUCT_REP, bscore,            \
                      pms.opt_params, pms.hc_params, pms.meta_params, \
                      pms.moses_params, pms.mmr_pa);                 \
}
 
        // problem == pre  precision-based scoring
        if (pms.problem == pre) {

            // Very nearly identical to the REGRESSION macro above,
            // except that some new experimental features are being tried.

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

        // problem == pre_conj  precision-based scoring (maximizing # conj)
        else if (pms.problem == pre_conj) {

            // Very nearly identical to the REGRESSION macro above,
            // except that some new experimental features are being tried.

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

        // problem == prerec  maximize precision, holding recall const.
        // Identical to above, just uses a different scorer.
        else if (pms.problem == prerec) {
            if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.5;
                pms.max_rand_input = 1.0; }
            REGRESSION(id::boolean_type,
                       *pms.bool_reduct, *pms.bool_reduct_rep,
                       pms.ctables, prerec_bscore,
                       (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
        }

        // problem == recall  maximize recall, holding precision const.
        else if (pms.problem == recall) {
            if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.8;
                pms.max_rand_input = 1.0; }
            REGRESSION(id::boolean_type,
                       *pms.bool_reduct, *pms.bool_reduct_rep,
                       pms.ctables, recall_bscore,
                       (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
        }

        // bep == beak-even point between recall and precision.
        else if (pms.problem == bep) {
            if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.0;
                pms.max_rand_input = 0.5; }
            REGRESSION(id::boolean_type,
                       *pms.bool_reduct, *pms.bool_reduct_rep,
                       pms.ctables, bep_bscore,
                       (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
        }

        // f_one = F_1 harmonic ratio of recall and precision
        else if (pms.problem == f_one) {
            REGRESSION(id::boolean_type,
                       *pms.bool_reduct, *pms.bool_reduct_rep,
                       pms.ctables, f_one_bscore,
                       (table));
        }

        // problem == it  i.e. input-table based scoring.
        else {
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
                unsupported_type_exit(output_type);
            }
        }
    }

    // Find interesting predicates
    else if (pms.problem == ip) {
        // ip assumes that the inputs are boolean and the output is contin
        type_tree ettt = gen_signature(id::boolean_type,
                                       id::contin_type, pms.arity);
        OC_ASSERT(ettt == table_type_signature,
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

    // regression based on input table using ann
    else if (pms.problem == ann_it)
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
                              ann_reduction(), ann_reduction(), bscore,
                              pms.opt_params, pms.hc_params, pms.meta_params,
                              pms.moses_params, pms.mmr_pa);
    }
    return 0;
}

int moses_exec(const vector<string>& argvs)
{
    char** argv = new char*[argvs.size()];
    for(size_t i = 0; i < argvs.size(); ++i) {
        argv[i] = const_cast<char*>(argvs[i].c_str());
    }
    int res = moses_exec(argvs.size(), argv);
    delete[] argv;
    return res;
}

} // ~namespace moses
} // ~namespace opencog
