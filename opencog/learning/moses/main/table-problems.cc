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
#include "table-problems.h"

namespace opencog { namespace moses {

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

void table_problem_params::add_options(boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;

    desc.add_options()

        ("input-file,i",
         po::value<vector<string>>(&input_data_files),
         "Input table file in DSV format (with comma, whitespace "
         "and tabulation as seperator). Colums correspond to features "
         "and rows to observations. Can be used several times, in such "
         "a case the behavioral score of the whole problem is the "
         "concatenation of the behavioral scores of the sub-problems "
         "associated with the files. Each file must have the same number "
         "of features in the same order.\n")

        ("target-feature,u",
         po::value<string>(&target_feature),
         "Label of the target feature to fit. If none is given the "
         "first one is used.\n")

        ("score-weight",
         po::value<string>(&weighting_feature),
         "Feature (table column) to use as a weight during scoring. "
         "The score of the combo model on each row of the table is "
         "weighted by this value, to determine the final score. This "
         "option is useful when not all of the rows in a table are "
         "equally important to model correctly.\n")

        ("ignore-feature,Y",
         po::value<vector<string>>(&ignore_features_str),
         "Ignore feature from the datasets. Can be used several times "
         "to ignore several features.\n")

    ;  // end of desc add options
}

// ==================================================================

void table_problem_base::common_setup(problem_params& pms)
{
    if (_tpp.input_data_files.empty()) {
        stringstream ss;
        ss << "no input data file has been specified (option -i)";
        log_output_error_exit(ss.str());
    }

    // Read input data files
    tables.clear();
    ctables.clear();
    size_t num_rows = 0;
    for (const string& idf : _tpp.input_data_files) {
        logger().info("Read data file %s", idf.c_str());
        Table table = loadTable(idf, _tpp.target_feature, _tpp.ignore_features_str);
        num_rows += table.size();
        // Possibly subsample the table
        if (pms.nsamples > 0)
            subsampleTable(table, pms.nsamples);
        // Compressed table
        ctables.push_back(table.compressed(_tpp.weighting_feature));
        // The compressed table removes the weighting feature, too.
        if (not _tpp.weighting_feature.empty())
            table.itable.delete_column(_tpp.weighting_feature);
        tables.push_back(table);
    }
    logger().info("Number of rows in tables = %d", num_rows);

    // Get the labels contained in the data file.
    ilabels.clear();
    if (pms.output_with_labels)
        ilabels = tables.front().itable.get_labels();

    arity = tables.front().get_arity();

    // Check that all input data files have the same arity
    if (tables.size() > 1) {
        for (size_t i = 1; i < tables.size(); ++i) {
            combo::arity_t test_arity = tables[i].get_arity();
            if (test_arity != arity) {
                stringstream ss;
                ss << "File " << _tpp.input_data_files[0] << " has arity " << arity
                   << " while file " << _tpp.input_data_files[i] << " has arity " << test_arity;
                log_output_error_exit(ss.str());
            }
        }
    }
    logger().info("Inferred arity = %d", arity);

    pms.mmr_pa.ilabels = ilabels;
}

void table_problem_base::common_type_setup(problem_params& pms)
{
    // Infer the signature based on the input table.
    table_type_signature = tables.front().get_signature();
    logger().info() << "Inferred data signature " << table_type_signature;

    // Infer the type of the input table
    table_output_tt = get_signature_output(table_type_signature);
    table_output_tn = get_type_node(table_output_tt);

    // Determine the default exemplar to start with
    // problem == pre  precision-based scoring
    // precision combo trees always return booleans.
    if (pms.exemplars.empty()) {
        type_node pbr_type = pms.problem == pre_table_problem(_tpp).name() ?
            id::boolean_type : table_output_tn;
        pms.exemplars.push_back(type_to_exemplar(pbr_type));
    }

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

void ip_problem_params::add_options(boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;

    desc.add_options()

        ("ip_kld_weight,K",
         po::value<double>(&ip_kld_weight)->default_value(1.0),
         "Interesting patterns (ip). Weight of the KLD.\n")

        ("ip_skewness_weight,J",
         po::value<double>(&ip_skewness_weight)->default_value(1.0),
         "Interesting patterns (ip). Weight of skewness.\n")

        ("ip_stdU_weight,U",
         po::value<double>(&ip_stdU_weight)->default_value(1.0),
         "Interesting patterns (ip). Weight of stdU.\n")

        ("ip_skew_U_weight,X",
         po::value<double>(&ip_skew_U_weight)->default_value(1.0),
         "Interesting patterns (ip). Weight of skew_U.\n")

    ;  // end of desc add options
}

void ip_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    // ip assumes that the inputs are boolean and the output is contin
    type_tree ettt = gen_signature(id::boolean_type,
                                   id::contin_type, arity);
    OC_ASSERT(ettt == tables.front().get_signature(),
              "The input table doesn't have the right data types."
              " The output should be contin and the inputs should"
              " be boolean");
    // signature of the functions to learn
    type_tree tt = gen_signature(id::boolean_type, arity);

    // determine the default exemplar to start with
    if (pms.exemplars.empty())
        pms.exemplars.push_back(type_to_exemplar(id::boolean_type));

    int as = alphabet_size(tt, pms.ignore_ops);

    typedef interesting_predicate_bscore BScore;
    typedef boost::ptr_vector<bscore_base> BScorerSeq;
    BScorerSeq bscores;
    for (const CTable& ctable : ctables) {
        BScore *r = new BScore(ctable,
                               _ippp.ip_kld_weight,
                               _ippp.ip_skewness_weight,
                               _ippp.ip_stdU_weight,
                               _ippp.ip_skew_U_weight,
                               pms.min_rand_input,
                               pms.max_rand_input,
                               pms.hardness, pms.hardness >= 0);
        set_noise_or_ratio(*r, as, pms.noise, pms.complexity_ratio);
        bscores.push_back(r);
    }
    multibehave_cscore mbcscore(bscores);
    multibscore_based_bscore bscore(bscores); // XXX TODO REMOVE ME
    metapop_moses_results(pms.exemplars, tt,
                          *pms.bool_reduct, *pms.bool_reduct_rep, 
                          bscore, mbcscore,
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

void ann_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    // If no exemplar has been provided in the options,
    // insert the default.
    if (pms.exemplars.empty()) {
        pms.exemplars.push_back(ann_exemplar(arity));
    }

    type_tree tt = gen_signature(id::ann_type, 0);
    int as = alphabet_size(tt, pms.ignore_ops);

    contin_bscore bscore(tables.front());
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);
    behave_cscore cscore(bscore);
    metapop_moses_results(pms.exemplars, tt,
                          reduct::ann_reduction(), reduct::ann_reduction(),
                          bscore, cscore,
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
    multibehave_cscore mbcscore(bscores);                            \
    multibscore_based_bscore bscore(bscores);                        \
    metapop_moses_results(pms.exemplars, cand_sig,                   \
                      REDUCT, REDUCT_REP, bscore, mbcscore,          \
                      pms.opt_params, pms.hc_params, pms.meta_params, \
                      pms.moses_params, pms.mmr_pa);                 \
}

void pre_table_problem::run(option_base* ob)
{
    // Very nearly identical to the REGRESSION macro above,
    // except that some new experimental features are being tried.
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
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
    for (const CTable& ctable : ctables) {
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
        pms.meta_params.fstor = new feature_selector(ctables.front(),
                                                     pms.festor_params);
    }

    multibehave_cscore mbcscore(bscores);
    multibscore_based_bscore bscore(bscores);
    metapop_moses_results(pms.exemplars, cand_sig,
                          *pms.bool_reduct, *pms.bool_reduct_rep,
                          bscore, mbcscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);

}

void pre_conj_table_problem::run(option_base* ob)
{
    // Very nearly identical to the REGRESSION macro above,
    // except that some new experimental features are being tried.
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
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
    for (const CTable& ctable : ctables) {
        BScore* r = new BScore(ctable,
                               fabs(pms.hardness),
                               pms.hardness >= 0);
        set_noise_or_ratio(*r, as, pms.noise, pms.complexity_ratio);
        bscores.push_back(r);
    }

    // Enable feature selection while selecting exemplar
    if (pms.enable_feature_selection && pms.fs_params.target_size > 0) {
        // XXX FIXME should use the concatenation of all ctables, not just first
        pms.meta_params.fstor = new feature_selector(ctables.front(),
                                                 pms.festor_params);
    }
    multibehave_cscore mbcscore(bscores);
    multibscore_based_bscore bscore(bscores); // XXX TODO REMOVE ME
    metapop_moses_results(pms.exemplars, cand_sig,
                          *pms.bool_reduct, *pms.bool_reduct_rep,
                          bscore, mbcscore,
                          pms.opt_params, pms.hc_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

void prerec_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms);

    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.5;
        pms.max_rand_input = 1.0; }
    REGRESSION(id::boolean_type,
               *pms.bool_reduct, *pms.bool_reduct_rep,
               ctables, prerec_bscore,
               (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

void recall_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms);

    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.8;
        pms.max_rand_input = 1.0; }
    REGRESSION(id::boolean_type,
               *pms.bool_reduct, *pms.bool_reduct_rep,
               ctables, recall_bscore,
               (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

void bep_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms);

    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.0;
        pms.max_rand_input = 0.5; }
    REGRESSION(id::boolean_type,
               *pms.bool_reduct, *pms.bool_reduct_rep,
               ctables, bep_bscore,
               (table, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

void f_one_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms);

    REGRESSION(id::boolean_type,
               *pms.bool_reduct, *pms.bool_reduct_rep,
               ctables, f_one_bscore,
               (table));
}

void it_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms);

    // problem == it  i.e. input-table based scoring.
    OC_ASSERT(output_type == table_output_tn);

    // --------- Boolean output type
    if (output_type == id::boolean_type) {
        REGRESSION(output_type,
                   *pms.bool_reduct, *pms.bool_reduct_rep,
                   ctables, ctruth_table_bscore,
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
            partial_solver well(ctables,
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
                       ctables, enum_effective_bscore,
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
                       tables, contin_bscore,
                       (table, eft));

        } else {
            REGRESSION(output_type,
                       *pms.contin_reduct, *pms.contin_reduct,
                       tables, discretize_contin_bscore,
                       (table.otable, table.itable,
                            pms.discretize_thresholds, pms.weighted_accuracy));
        }
    }

    // --------- Unknown output type
    else {
        logger().error() << "Error: output type " << type_tree(output_type)
            << " currently not supported.";
        std::cerr << "Error: output type " << type_tree(output_type)
            << " currently not supported." << std::endl;
        exit(1);
    }
}

void cluster_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms);

    REGRESSION(output_type,
               *pms.contin_reduct, *pms.contin_reduct,
               tables, cluster_bscore,
               (table.itable));
}

// ==================================================================

void register_table_problems(problem_manager& pmr, option_manager& mgr)
{
	table_problem_params *tpp = new table_problem_params();
	mgr.register_options(tpp);

	ip_problem_params *ippp = new ip_problem_params();
	mgr.register_options(ippp);

	pmr.register_problem(new ip_problem(*tpp, *ippp));
	pmr.register_problem(new ann_table_problem(*tpp));
	pmr.register_problem(new pre_table_problem(*tpp));
	pmr.register_problem(new pre_conj_table_problem(*tpp));
	pmr.register_problem(new prerec_table_problem(*tpp));
	pmr.register_problem(new recall_table_problem(*tpp));
	pmr.register_problem(new bep_table_problem(*tpp));
	pmr.register_problem(new f_one_table_problem(*tpp));
	pmr.register_problem(new it_table_problem(*tpp));
	pmr.register_problem(new cluster_table_problem(*tpp));
}

} // ~namespace moses
} // ~namespace opencog

