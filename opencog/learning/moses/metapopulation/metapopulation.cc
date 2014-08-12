/** metapopulation.cc ---
 *
 * Copyright (C) 2010 Novemente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Authors: Nil Geisweiller, Moshe Looks, Linas Vepstas
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

#include <math.h>

#include <opencog/util/oc_omp.h>
#include <opencog/util/selection.h>

#include "metapopulation.h"

namespace opencog {
namespace moses {

using namespace combo;

metapopulation::metapopulation(const std::vector<combo_tree>& bases,
               behave_cscore& sc,
               const metapop_parameters& pa,
               const subsample_deme_filter_parameters& subp) :
    _cached_dst(pa.diversity),
    _params(pa),
    _filter_params(subp),
    _cscorer(sc),
    _merge_count(0),
    _best_cscore(worst_composite_score),
    _ensemble(sc, pa.ensemble_params)
{
    init(bases);
}


metapopulation::metapopulation(const combo_tree& base,
               behave_cscore& sc,
               const metapop_parameters& pa,
               const subsample_deme_filter_parameters& subp) :
    _cached_dst(pa.diversity),
    _params(pa),
    _filter_params(subp),
    _cscorer(sc),
    _merge_count(0),
    _best_cscore(worst_composite_score),
    _ensemble(sc, pa.ensemble_params)
{
    std::vector<combo_tree> bases(1, base);
    init(bases);
}


// Init the metapopulation with the following set of exemplars.
void metapopulation::init(const std::vector<combo_tree>& exemplars)
{
    scored_combo_tree_set candidates;
    for (const combo_tree& base : exemplars) {
        composite_score csc(_cscorer.get_cscore(base));

        // The behavioral score must be recomputed here, so we can
        // store it in case diversity preservation is used
        behavioral_score bs(_cscorer.get_bscore(base));

        scored_combo_tree sct(base, demeID_t(), csc, bs);

        candidates.insert(sct);
    }

    update_best_candidates(candidates);
    merge_candidates(candidates);
}

// -------------------------------------------------------------------
// Exemplar selection-related code

bool metapopulation::has_been_visited(const scored_combo_tree& tr) const
{
    return _visited_exemplars.find(tr) != _visited_exemplars.cend();
}

void metapopulation::log_selected_exemplar(scored_combo_tree_ptr_set::const_iterator exemplar_it)
{
    if (not logger().isDebugEnabled()) return;

    if (exemplar_it == _scored_trees.cend()) {
        logger().debug() << "No exemplar found";
    } else {
        const auto& xmplr = *exemplar_it;
        unsigned pos = std::distance(_scored_trees.cbegin(), exemplar_it) + 1,
            nth_vst = _visited_exemplars[xmplr];

        logger().debug() << "Selected the " << pos
                         << "th exemplar, from deme " << xmplr.get_demeID()
                         << ", for the " << nth_vst << "th time(s)";
        logger().debug() << "Exemplar tree : " << xmplr.get_tree();
        logger().debug() << "With composite score : "
                         << xmplr.get_composite_score();
    }
}

scored_combo_tree_ptr_set::const_iterator metapopulation::select_exemplar()
{
    OC_ASSERT(!empty(), "Empty metapopulation in select_exemplar().");

    logger().debug("Select exemplar");

    // Shortcut for special case, as sometimes, the very first time
    // though, the score is invalid.
    if (size() == 1) {
        scored_combo_tree_ptr_set::const_iterator selex = _scored_trees.cbegin();
        if(_params.revisit < 0 or
           (_params.revisit + 1 > (int)_visited_exemplars[*selex])) // not enough visited
            _visited_exemplars[*selex]++;
        else selex = _scored_trees.cend();    // enough visited

        log_selected_exemplar(selex);
        return selex;
    }

    std::vector<score_t> probs;
    // Set flag to true, when a suitable exemplar is found.
    bool found_exemplar = false;
#define UNEVALUATED_SCORE -1.0e37
    score_t highest_score = UNEVALUATED_SCORE;

    // The exemplars are stored in order from best score to worst;
    // the iterator follows this order.
    for (const scored_combo_tree& bsct : *this) {

        score_t sc = bsct.get_penalized_score();

        // Skip exemplars that have been visited enough
        if (_params.revisit < 0 or
            (_params.revisit + 1 > (int)_visited_exemplars[bsct])) {
            probs.push_back(sc);
            found_exemplar = true;
            if (highest_score < sc) highest_score = sc;
        } else // If the tree is visited enough then put a
               // nan score so we know it must be ignored
            probs.push_back(NAN);
    }

    // Nothing found, we've already tried them all.
    if (!found_exemplar) {
        log_selected_exemplar(_scored_trees.cend());
        return _scored_trees.cend();
    }

    // Compute the probability normalization, needed for the
    // roullete choice of exemplars with equal scores, but
    // differing complexities. Empirical work on 4-parity suggests
    // that a temperature of 3 or 4 works best.
    score_t inv_temp = 100.0f / _params.complexity_temperature;
    score_t sum = 0.0f;
    // Convert scores into (non-normalized) probabilities
    for (score_t& p : probs) {
        // If p is invalid (or already visited, because it has nan)
        // then it is skipped, i.e. assigned probability of 0.0f
        if (isfinite(p))
            p = expf((p - highest_score) * inv_temp);
        else
            p = 0.0;

        sum += p;
    }

    // log the distribution probs
    if (logger().isFineEnabled())
    {
        std::stringstream ss;
        ss << "Non-normalized probability distribution of candidate selection: ";
        ostreamContainer(ss, probs);
        logger().fine() << ss.str();
    }

    OC_ASSERT(sum > 0.0f, "There is an internal bug, please fix it");

    size_t fwd = std::distance(probs.begin(), roulette_select(probs.begin(),
                                                         probs.end(),
                                                         sum, randGen()));
    // cout << "select_exemplar(): sum=" << sum << " fwd =" << fwd
    // << " size=" << probs.size() << " frac=" << fwd/((float)probs.size()) << endl;
    scored_combo_tree_ptr_set::const_iterator selex = std::next(_scored_trees.begin(), fwd);

    // We increment _visited_exemplar
    _visited_exemplars[*selex]++;

    log_selected_exemplar(selex);
    return selex;
}

// -------------------------------------------------------------------
// Search-termination-related routines.  The scores that are returned
// are used by the main program to terminate the search.

/**
 * Return the composite score of the highest-scoring tree in the metapop.
 */
composite_score metapopulation::best_composite_score() const
{
    if (not _params.do_boosting)
        return _best_cscore;

    // XXX FIXME should probably not recompute every time ...
    // need to figure who is calling this method, and what they are expecting.
    return _cscorer.get_cscore(_ensemble.get_ensemble());
}

/**
 * Return the set of scored combo trees with the highest composite
 * scores.  If boosting (ensemble learning) is enabled, these are
 * not the trees that you are looking for.  Call get_ensemble()
 * instead.
 */
const scored_combo_tree_set& metapopulation::best_candidates() const
{
    return _best_candidates;
}

/**
 * Return the best combo tree (shortest best candidate).
 */
const combo_tree& metapopulation::best_tree() const
{
    if (_params.do_boosting) {
        return _ensemble.get_weighted_tree();
    }
    return best_candidates().begin()->get_tree();
}

std::ostream& metapopulation::ostream_metapop(std::ostream& out, int maxcnt) const
{
    const scored_combo_tree_set& tree_set = best_candidates();
    int cnt = 0;
    for (const scored_combo_tree& sct : tree_set) {
        if (maxcnt < ++cnt) break;
        out << sct;
    }
    return out;
}

} // ~namespace moses
} // ~namespace opencog

