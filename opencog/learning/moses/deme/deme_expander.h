/** deme_expander.h --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
 *
 * Author: Nil Geisweiller 
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

#ifndef _OPENCOG_DEME_EXPANDER_H
#define _OPENCOG_DEME_EXPANDER_H

#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>

#include "../optimization/optimization.h"
#include "../scoring/behave_cscore.h"
#include "../metapopulation/metapop_params.h"
#include "deme_params.h"

namespace opencog {
namespace moses {

struct deme_expander
{
    deme_expander(const type_tree& type_signature,
                  const reduct::rule& si_ca,
                  const reduct::rule& si_kb,
                  behave_cscore& sc,
                  optimizer_base& opt,
                  const deme_parameters& pa = deme_parameters(),
                  const subsample_deme_filter_parameters& fp = subsample_deme_filter_parameters());

    ~deme_expander() {}

    /**
     * Create demes
     *
     * @return return true if it creates demes successfully, otherwise false.
     */
    // bool create_deme(scored_combo_tree_set::const_iterator exemplar)
    bool create_demes(const combo_tree& exemplar, int n_expansions = 0);

    /**
     * Do some optimization according to the scoring function.
     *
     * @param max_evals the maximum number of evaluations of the scoring
     *                  function to perform.
     * @param max_time the maximum elapsed (wall-clock) time to allow.
     */
    void optimize_demes(int max_evals, time_t max_time);

    void free_demes();

    /**
     * Return the total number of evaluations across all demes
     */
    unsigned total_evals();

    // Structures related to the current deme
    boost::ptr_vector<representation> _reps; // representations of the demes
    std::vector<std::vector<deme_t>> _demes; // current demes, vector of
                                             // vectors is used because a
                                             // deme can be further
                                             // subdiveded into multiple
                                             // demes using a subsampled
                                             // dataset

    optimizer_base &_optimize;

protected:
    /**
     * Subsample by time. Return a vector (of size n_ss_demes) of sets
     * of timestamps to be discarded during subsampling.
     */
    std::vector<std::set<TTable::value_type>> subsample_by_time() const;

    /**
     * Subsample by row. return a vector (of size n_ss_demes) of sets
     * of indexes (of a corresponding uncompressed table) to be
     * discarded during subsampling.
     */
    std::vector<std::set<unsigned>> subsample_by_row() const;

    /**
     * Create deme IDs
     */
    void create_demeIDs(int n_expansions);

    /**
     * Create representations
     *
     * @return return true if it creates representations successfully, false otherwise
     */
    bool create_representations(const combo_tree& exemplar);

    /**
     * Takes a feature set, a header of the input features and return
     * a vector of the names of the featire set.
     */
    string_seq fs_to_names(const feature_set& fs, const string_seq& ilabels) const;

    /**
     * Log selected features, the new ones and the ones part of the
     * exemplar.
     */
    void log_selected_feature_sets(const feature_set_pop& sf_pop,
                                   const feature_set& xmplr_features,
                                   const string_seq& ilabels) const;

    /**
     * Return pruned exemplar from non-selected features
     */
    combo_tree prune_xmplr(const combo_tree& xmplr,
                           const feature_set& selected_features) const;

    const combo::type_tree& _type_sig;          // type signature of the exemplar
    const reduct::rule& simplify_candidate;     // rule to simplify candidates
    const reduct::rule& simplify_knob_building; // during knob building

    // Used by random_shuffle
    std::function<ptrdiff_t(ptrdiff_t)> random_shuffle_gen;

public:
    behave_cscore& _cscorer; // composite score

protected:
    // This is used to keep track of the ignored indices for
    // optimizing evaluation (in case of feature selection) and
    // calculate max score per deme
    std::vector<std::set<arity_t>> _ignore_cols_seq;

    // Deme ids
    std::vector<demeID_t> _demeIDs;

    const deme_parameters& _params;

    const subsample_deme_filter_parameters& _filter_params;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_DEME_EXPANDER_H
