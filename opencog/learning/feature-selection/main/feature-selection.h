/** feature-selection.h --- 
 *
 * Copyright (C) 2011 Nil Geisweiller
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_FEATURE_SELECTION_H
#define _OPENCOG_FEATURE_SELECTION_H

#include <opencog/learning/moses/eda/field_set.h>
#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/comboreduct/combo/table.h>

#include "../feature_scorer.h"

using namespace eda;
using namespace moses;
using namespace combo;

// optimization algorithms
static const string un="un"; // univariate
static const string sa="sa"; // simulation annealing
static const string hc="hc"; // hillclimbing

static const string default_log_file_prefix = "feature-selection";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;

// program option names and abbreviations
// for their meanings see options_description in moses-exec.cc
static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> opt_algo_opt("opt-algo", "a");
static const pair<string, string> input_data_file_opt("input-file", "i");
static const pair<string, string> max_evals_opt("max-evals", "m");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> log_level_opt("log-level", "l");
static const pair<string, string> log_file_opt("log-file", "f");
static const pair<string, string> log_file_dep_opt_opt("log-file-dep-opt", "L");
static const pair<string, string> cache_size_opt("cache-size", "s");
static const pair<string, string> complexity_penalty_intensity_opt("complexity-penalty-intensity", "p");

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

// parameters of feature-selection, see desc.add_options() in
// feature-selection.cc for their meaning
struct feature_selection_parameters {
    std::string algorithm;
    unsigned int max_evals;
    std::string input_file;
    std::string output_file;
    std::vector<std::string> initial_features;
    unsigned long cache_size;
    double cpi; // complexity penalty intensity
};

std::set<arity_t> get_feature_set(const field_set& fields,
                                  const eda::instance& inst) {
    std::set<arity_t> fs;
    field_set::const_bit_iterator bit = fields.begin_bits(inst);
    for(arity_t i = 0; bit != fields.end_bits(inst); bit++, i++)
        if(*bit)
            fs.insert(i);
    return fs;
}

template<typename IT, typename OT, typename Optimize, typename Scorer>
void feature_selection(IT& it, const OT& ot,
                       const field_set& fields,
                       instance_set<composite_score>& deme,
                       eda::instance& init_inst,
                       Optimize& optimize, const Scorer& scorer,
                       const feature_selection_parameters& fs_params) {
    // optimize feature set
    optimize(deme, init_inst, scorer, fs_params.max_evals);
    // get the best one
    std::sort(deme.begin(), deme.end(),
              std::greater<scored_instance<composite_score> >());
    eda::instance best_inst = deme[0].first;
    // set the input table accordingly
    it.set_consider_args_from_zero(get_feature_set(fields, best_inst));
    // prin the filtered table
    if(fs_params.output_file.empty())
        ostreamTable(std::cout, it, ot);
    else
        ostreamTable(fs_params.output_file, it, ot);
}

// Mutual Information + Occam's Razor scorer
template<typename IT, typename OT>
struct MIORScorer : public unary_function<eda::instance, composite_score> {
    MIORScorer(const IT& _it, const OT& _ot, const field_set& _fields,
               double _cpi = 1.0)
        : it(_it), ot(_ot), fields(_fields), cpi(_cpi) {}
    composite_score operator()(const eda::instance& inst) const {
        std::set<arity_t> fs = get_feature_set(fields, inst);
        MutualInformation<IT, OT, std::set<arity_t> > MI(it, ot);
        complexity_t c = -fields.count(inst);
        composite_score csc(MI(fs) * exp(cpi * c), c);
        // Logger
        if(logger().getLevel() >= opencog::Logger::FINE) {
            stringstream ss;
            ss << "MIORScorer - Evaluate instance: " 
               << fields.stream(inst) << " " << csc << std::endl;
            logger().fine(ss.str());
        }
        // ~Logger
        return csc;
    }
    const IT& it;
    const OT& ot;
    const field_set& fields;
    double cpi; // compexity penalty intensity
};

template<typename IT, typename OT>
void feature_selection(IT& it, const OT& ot,
                       const feature_selection_parameters& fs_params,
                       RandGen& rng) {
    arity_t arity = it.get_arity();
    if(fs_params.algorithm == un) {
        OC_ASSERT(false, "TODO");
    } else if(fs_params.algorithm == sa) {
        OC_ASSERT(false, "TODO");        
    } else if(fs_params.algorithm == hc) {
        field_set fields(field_set::disc_spec(2), arity);
        optim_parameters op_param(20, 1, 2);
        hc_parameters hc_param(false); // do not terminate if improvement
        iterative_hillclimbing hc(rng, op_param, hc_param);
        instance_set<composite_score> deme(fields);
        eda::instance init_inst(fields.packed_width());
        typedef MIORScorer<IT, OT> Scorer;
        Scorer sc(it, ot, fields, fs_params.cpi);
        if(fs_params.cache_size > 0) {
            typedef lru_cache<Scorer> ScorerCache;
            ScorerCache sc_cache(fs_params.cache_size, sc);
            feature_selection(it, ot, fields, deme, init_inst, hc, sc_cache,
                              fs_params);
        } else {
            feature_selection(it, ot, fields, deme, init_inst, hc, sc,
                              fs_params);            
        }
    }
}

#endif // _OPENCOG_FEATURE-SELECTION_H
