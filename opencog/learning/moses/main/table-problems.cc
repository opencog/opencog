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
#include "../scoring/bscores.h"
#include "../scoring/discriminating_bscore.h"
#include "../scoring/select_bscore.h"

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

        ("timestamp-feature",
         po::value<string>(&timestamp_feature),
         "Label of the timestamp feature. If none is given it is ignored.\n")

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
    _tables.clear();
    _ctables.clear();
    size_t num_rows = 0;
    for (const string& idf : _tpp.input_data_files) {
        logger().info("Read data file %s", idf.c_str());
        Table table = loadTable(idf, _tpp.target_feature,
                                _tpp.timestamp_feature,
                                _tpp.ignore_features_str);
        num_rows += table.size();

        // Possibly subsample the table
        if (pms.nsamples > 0)
            subsampleTable(table, pms.nsamples);

        // Compressed table
        _ctables.push_back(table.compressed(_tpp.weighting_feature));

        // The compressed table removes the weighting feature, too.
        if (not _tpp.weighting_feature.empty())
            table.itable.delete_column(_tpp.weighting_feature);
        _tables.push_back(table);

        // Possibly balance the ctable
        if (pms.balance)
            _ctables.back().balance();
    }
    logger().info("Number of rows in tables = %d", num_rows);

    // XXX FIXME -- the multiple tables should be merged into one.
    ctable = _ctables.front();
    table = _tables.front();

    // Get the labels contained in the data file.
    ilabels.clear();
    if (pms.output_with_labels)
        ilabels = table.itable.get_labels();

    arity = table.get_arity();

    // Check that all input data files have the same arity
    // XXX FIXME .. check that they all have the same signature.
    if (_tables.size() > 1) {
        for (size_t i = 1; i < _tables.size(); ++i) {
            combo::arity_t test_arity = _tables[i].get_arity();
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

/**
 * Set up the output
 */
void table_problem_base::common_type_setup(problem_params& pms,
                                           type_node out_type)
{
    // Infer the signature based on the input table.
    table_type_signature = table.get_signature();
    logger().info() << "Table signature " << table_type_signature;

    // Infer the type of the input table
    if (id::unknown_type == out_type) {
        type_tree table_output_tt = get_signature_output(table_type_signature);
        out_type = get_type_node(table_output_tt);
    }

    // Determine the default exemplar to start with
    if (pms.exemplars.empty()) {
        pms.exemplars.push_back(type_to_exemplar(out_type));
    }

    // The exemplar output type can differ from the table output type
    // for scorers that are trying to select rows (the pre and select scorers)
    output_type =
        get_type_node(get_output_type_tree(*pms.exemplars.begin()->begin()));

    if (id::unknown_type == output_type) output_type = out_type;

    cand_type_signature = gen_signature(
        get_signature_inputs(table_type_signature),
        type_tree(output_type));

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
    OC_ASSERT(ettt == table.get_signature(),
              "The input table doesn't have the right data types."
              " The output should be contin and the inputs should"
              " be boolean");
    // signature of the functions to learn
    type_tree tt = gen_signature(id::boolean_type, arity);

    // determine the default exemplar to start with
    if (pms.exemplars.empty())
        pms.exemplars.push_back(type_to_exemplar(id::boolean_type));

    int as = alphabet_size(tt, pms.ignore_ops);

    interesting_predicate_bscore bscore(ctable,
                  _ippp.ip_kld_weight,
                  _ippp.ip_skewness_weight,
                  _ippp.ip_stdU_weight,
                  _ippp.ip_skew_U_weight,
                  pms.min_rand_input,
                  pms.max_rand_input,
                  pms.hardness, pms.hardness >= 0);
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);

    // In order to support boosting, the interesting_predicate_bscore
    // would have to modified to always return a bscore that is the
    // smae length, and so that the same item akways refered to the
    // same row in the ctable.
    OC_ASSERT(not pms.meta_params.do_boosting,
        "Boosting not supported for the ip problem!");
    behave_cscore mbcscore(bscore, pms.cache_size);
    metapop_moses_results(pms.exemplars, tt,
                          *pms.bool_reduct, *pms.bool_reduct_rep,
                          mbcscore,
                          pms.opt_params, pms.hc_params,
                          pms.deme_params, pms.filter_params, pms.meta_params,
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

    contin_bscore bscore(table);
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);

    // Boosing for contin values needs a whole bunch of new code.
    OC_ASSERT(not pms.meta_params.do_boosting,
        "Boosting not supported for the ann problem!");
    behave_cscore cscore(bscore, pms.cache_size);
    metapop_moses_results(pms.exemplars, tt,
                          reduct::ann_reduction(), reduct::ann_reduction(),
                          cscore,
                          pms.opt_params, pms.hc_params,
                          pms.deme_params, pms.filter_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

// ==================================================================

// Generic table regression code.  Could be a template, I suppose, but
// the args are variable length, tables is a variable, and scorer is a type,
// and I don't feel like fighting templates to make all three happen just
// exactly right.
#define REGRESSION(TABLE, SCORER, ARGS)                              \
{                                                                    \
    /* Enable feature selection while selecting exemplar */          \
    if (pms.enable_feature_selection and pms.fs_params.target_size > 0) { \
        pms.deme_params.fstor =                                      \
                     new feature_selector(TABLE, pms.festor_params); \
    }                                                                \
    int as = alphabet_size(cand_type_signature, pms.ignore_ops);     \
    SCORER bscore ARGS ;                                             \
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio); \
    if (pms.meta_params.do_boosting) bscore.use_weighted_scores();   \
                                                                     \
    /* When boosting, cache must not be used, as otherwise, stale */ \
    /* composite scores get cached and returned.                  */ \
    if (pms.meta_params.do_boosting) pms.cache_size = 0;             \
    behave_cscore mbcscore(bscore, pms.cache_size);                  \
    reduct::rule* reduct_cand = pms.bool_reduct;                     \
    reduct::rule* reduct_rep = pms.bool_reduct_rep;                  \
    /* Use the contin reductors for everything else */               \
    if (id::boolean_type != output_type) {                           \
        reduct_cand = pms.contin_reduct;                             \
        reduct_rep = pms.contin_reduct;                              \
    }                                                                \
    metapop_moses_results(pms.exemplars, cand_type_signature,        \
                      *reduct_cand, *reduct_rep, mbcscore,           \
                      pms.opt_params, pms.hc_params,                 \
                      pms.deme_params, pms.filter_params, pms.meta_params,              \
                      pms.moses_params, pms.mmr_pa);                 \
}

void pre_table_problem::run(option_base* ob)
{
    // Very nearly identical to the REGRESSION macro above,
    // except that some new experimental features are being tried.
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::boolean_type);

    // Enable feature selection while selecting exemplar
    if (pms.enable_feature_selection && pms.fs_params.target_size > 0) {
        pms.deme_params.fstor = new feature_selector(ctable,
                                                     pms.festor_params);
    }

    int as = alphabet_size(cand_type_signature, pms.ignore_ops);
    precision_bscore bscore(ctable,
                            fabs(pms.hardness),
                            pms.min_rand_input,
                            pms.max_rand_input,
                            pms.time_dispersion_pressure,
                            pms.time_dispersion_exponent,
                            pms.meta_params.ensemble_params.exact_experts,
                            pms.meta_params.ensemble_params.bias_scale,
                            pms.hardness >= 0,
                            pms.time_bscore,
                            pms.time_bscore_granularity);
    set_noise_or_ratio(bscore, as, pms.noise, pms.complexity_ratio);
    if (pms.meta_params.do_boosting) bscore.use_weighted_scores();

    if (pms.gen_best_tree) {
        // experimental: use some canonically generated
        // candidate as exemplar seed
        combo_tree tr = bscore.gen_canonical_best_candidate();
        logger().info() << "Canonical program tree (non reduced) maximizing precision = " << tr;
        pms.exemplars.push_back(tr);
    }

    // We support boosting by generating an "ensemble of experts" model.
    pms.meta_params.ensemble_params.experts = true;
    // When boosting, cache must not be used, as otherwise, stale
    // composite scores get cached and returned.
    if (pms.meta_params.do_boosting) pms.cache_size = 0;

    behave_cscore mbcscore(bscore, pms.cache_size);
    metapop_moses_results(pms.exemplars, cand_type_signature,
                          *pms.bool_reduct, *pms.bool_reduct_rep,
                          mbcscore,
                          pms.opt_params, pms.hc_params,
                          pms.deme_params, pms.filter_params, pms.meta_params,
                          pms.moses_params, pms.mmr_pa);
}

void pre_conj_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::boolean_type);

    // In order to support boosting, the precision_conj_bscore
    // would need to be reworked to always return a predictable,
    // indexed number of rows.
    OC_ASSERT(not pms.meta_params.do_boosting,
        "Boosting not supported for the pre problem!");
    REGRESSION(ctable, precision_conj_bscore,
               (ctable, fabs(pms.hardness), pms.hardness >= 0));
}

void prerec_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::boolean_type);

    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.5;
        pms.max_rand_input = 1.0; }
    REGRESSION(ctable, prerec_bscore,
               (ctable, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

void recall_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::boolean_type);

    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.8;
        pms.max_rand_input = 1.0; }
    REGRESSION(ctable, recall_bscore,
               (ctable, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

void bep_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::boolean_type);

    if (0.0 == pms.hardness) { pms.hardness = 1.0; pms.min_rand_input= 0.0;
        pms.max_rand_input = 0.5; }
    REGRESSION(ctable, bep_bscore,
               (ctable, pms.min_rand_input, pms.max_rand_input, fabs(pms.hardness)));
}

void f_one_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::boolean_type);

    REGRESSION(ctable, f_one_bscore, (ctable));
}

// problem == it  i.e. input-table based scoring.
void it_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms);

    // --------- Boolean output type
    if (output_type == id::boolean_type) {
        REGRESSION(ctable, ctruth_table_bscore, (ctable));
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
            partial_solver well(ctable,
                                pms.exemplars, *pms.contin_reduct,
                                pms.opt_params, pms.hc_params,
                                pms.deme_params, pms.filter_params,
                                pms.meta_params,
                                pms.moses_params, pms.mmr_pa);
            well.solve();
        } else {
            // Much like the boolean-output-type above,
            // just uses a slightly different scorer.
            REGRESSION(ctable, enum_effective_bscore, (ctable));
        }
    }

    // --------- Contin output type
    else if (output_type == id::contin_type) {
        if (pms.discretize_thresholds.empty()) {

            contin_bscore::err_function_type eft =
                pms.it_abs_err ? contin_bscore::abs_error :
                contin_bscore::squared_error;

            REGRESSION(table, contin_bscore, (table, eft));

        } else {
            REGRESSION(table, discretize_contin_bscore,
                       (table.otable, table.itable,
                        pms.discretize_thresholds, pms.weighted_accuracy));
        }
    }

    // --------- Unknown output type
    else {
        logger().error() << "Error: output type \"" << type_tree(output_type)
            << "\" currently not supported.";
        std::cerr << "Error: output type " << type_tree(output_type)
            << " currently not supported." << std::endl;
        exit(1);
    }
}

void select_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::boolean_type);

    REGRESSION(ctable, select_bscore,
               (ctable, pms.min_rand_input,
                        pms.max_rand_input,
                        fabs(pms.hardness),
                        pms.hardness >= 0.0));
}

void cluster_table_problem::run(option_base* ob)
{
    problem_params& pms = *dynamic_cast<problem_params*>(ob);
    common_setup(pms);
    common_type_setup(pms, id::contin_type);

    REGRESSION(table, cluster_bscore, (table.itable));
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
	pmr.register_problem(new select_table_problem(*tpp));
	pmr.register_problem(new cluster_table_problem(*tpp));
}

} // ~namespace moses
} // ~namespace opencog

