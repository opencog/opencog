/** metapopulation.cc ---
 *
 * Copyright (C) 2010 Novemente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
 * Copyright (C) 2014 Aidyia Limited
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

#include <boost/range/algorithm/sort.hpp>

#include <opencog/util/oc_omp.h>
#include <opencog/util/selection.h>

#include "metapopulation.h"

namespace opencog {
namespace moses {

using namespace combo;


/// Trim the demes down to size.  The point here is that the next
/// stage, deme_to_trees(), is very cpu-intensive; we should keep
/// only those candidates that will survive in the metapop.  But what
/// are these?  Well, select_exemplar() uses an exponential choice
/// function; instances below a cut-off score have no chance at all
/// of getting selected. So just eliminate them now, instead of later.
///
/// However, trimming too much is bad: it can happen that none
/// of the best-scoring instances lead to a solution. So keep
/// around a reasonable pool. Wild choice ot 250 seems reasonable.
/// (_min_pool_size defaults to 250)
///
/// In order for this algo to work, the instances in the deme must be
/// in score-sorted order, with the highest scores first, lowest
/// scores last. The tail of the deme (a vector) is cut off.
void metapopulation::trim_down_deme(deme_t& deme) const
{
    // Don't bother, if the deme is tiny.
    if (_min_pool_size >= deme.size())
        return;

    if (logger().isDebugEnabled())
    {
        std::stringstream ss;
        ss << "Trim down deme " << deme.getID()
           << " of size: " << deme.size();
        logger().debug(ss.str());
    }

    score_t top_sc = deme.begin()->second.get_penalized_score();
    score_t bot_sc = top_sc - useful_score_range();

    for (size_t i = deme.size()-1; 0 < i; --i) {
        const composite_score &cscore = deme[i].second;
        score_t score = cscore.get_penalized_score();
        if (score < bot_sc) {
            deme.pop_back();
        }
    }

    if (logger().isDebugEnabled())
    {
        std::stringstream ss;
        ss << "Deme trimmed down, new size: " << deme.size();
        logger().debug(ss.str());
    }
}

/// Convert the instances in the deme into scored combo_trees.
/// Return the resulting set of trees. The number of trees returned
/// is limited to _parms.max_candidates; in order for this limitation
/// to work, the deme must be presented in score-sorted order (highest
/// scores first, lowest scores last).
///
void metapopulation::deme_to_trees(deme_t& deme,
                                   const representation& rep,
                                   scored_combo_tree_set& pot_candidates)
{
    std::mutex mtx;

    auto select_candidates =
        [&](const scored_instance<composite_score>& inst)
    {
        const composite_score& inst_csc = inst.second;
        score_t inst_sc = inst_csc.get_score();
        // If score is really bad, don't bother.
        if (inst_sc <= very_worst_score || !isfinite(inst_sc))
            return;

        // Get the combo_tree associated to inst, cleaned and reduced.
        combo_tree tr = rep.get_candidate(inst, true);

        // update the set of potential exemplars
        scored_combo_tree sct(tr, deme.getID(), inst_csc);

        std::lock_guard<std::mutex> lock(mtx);
        pot_candidates.insert(sct);
    };

    if (logger().isDebugEnabled()) {
        logger().debug() << "Select " << deme.size()
                         << " candidates from deme " << deme.getID();
    }

    OMP_ALGO::for_each(deme.cbegin(), deme.cend(), select_candidates);
}

// Recompute the composite score for each member of the metapop.
// Used only when boosting. See header file for additioal documentation.
void metapopulation::rescore()
{
    bscore_base& bscorer = _cscorer.get_bscorer();
#define SERIAL_RESCORING 1
#if SERIAL_RESCORING
    for (scored_combo_tree& sct : _scored_trees) {
        score_t new_score = bscorer.sum_bscore(sct.get_bscore());
        sct.get_composite_score().set_score(new_score);
    }
#else
    auto rescore_sct = [&](scored_combo_tree& sct) {
        score_t new_score = bscorer.sum_bscore(sct.get_bscore());
        sct.get_composite_score().set_score(new_score);
    }
    OMP_ALGO::for_each(_scored_trees.begin(), _scored_trees.end(), rescore_sct);
#endif
}

/// Merge the given set of candidates into the metapopulation.
/// It is assumed that these candiates have already be vetted for
/// quality, quantity, suitability, etc.  This simply performs that
/// final, actual merge.
void metapopulation::merge_candidates(scored_combo_tree_set& candidates)
{
    if (logger().isDebugEnabled()) {
        logger().debug("Going to merge %u candidates with the metapopulation",
                       candidates.size());
        if (logger().isFineEnabled()) {
            std::stringstream ss;
            ss << "Candidates to merge with the metapopulation:" << std::endl;
            for (const auto& cnd : candidates)
                ss << cnd;
            logger().fine(ss.str());
        }
    }

    // Serialize access
    std::lock_guard<std::mutex> lock(_merge_mutex);

    // Note that merge_nondominated() is very cpu-expensive and
    // complex...
    if (not _params.discard_dominated) {
        logger().debug("Insert all candidates in the metapopulation");
        for (const auto& cnd : candidates)
            _scored_trees.insert(new scored_combo_tree(cnd));
    } else {
        logger().debug("Insert non-dominated candidates in the metapopulation");
        unsigned old_size = size();
        merge_nondominated(candidates, _params.jobs);
        logger().debug("Inserted %u non-dominated candidates "
                       "in the metapopulation", size() - old_size);
    }
}


/// Given a vector of demes, and the corresponding representations for
/// those demes, convert the instances in the demes into scored combo
/// trees, and merge them into the metapopulation.
///
/// During this merging, assorted cleanup and tuning is performed:
/// low-scoring instances are discarded, duplicates are discarded,
/// some diversity work is done.  The number of instances that are
/// finally merged are limited by the _params.max_candidates value.
/// (Its wise to set this value to something small-ish, say, a few
/// thousand, at most, to avoid excess CPU-time consumption, and
/// excess RAM usage.  Some parts of this merger can be very CPU-time
/// consuming.
//
// See also header file for a desciption of this method.
bool metapopulation::merge_demes(std::vector<std::vector<deme_t>>& all_demes,
                                 const boost::ptr_vector<representation>& reps)
{
    // Note that univariate reports far more evals than the deme size;
    // this is because univariate over-writes deme entries.
    logger().debug("Close and merge demes");

    // Sort all demes, keep_top_unique_candidates, trim_dowm_demes and
    // deme_to_trees rely on the assumption that the demes are sorted.
    sort_demes(all_demes);

    keep_top_unique_candidates(all_demes, reps);

    // In case of subsampling filter
    recompute_scores_over_whole_dataset(all_demes, reps);

    // Perform subsampling filtering and return a vector of bool,
    // corresponding to each breadth first deme indicating whether it
    // passes the filter or not.
    std::vector<bool> pass_filter = ss_filter(all_demes, reps);

    // Loop through breadth first demes
    scored_combo_tree_set pot_candidates;
    for (unsigned j = 0; j < all_demes.size(); j++) {

        // Skip merging that breadth first deme if it hasn't passed
        // the filter
        if (!pass_filter[j])
            continue;

        std::vector<deme_t>& demes = all_demes[j];
        for (unsigned i = 0; i < demes.size(); i++) {
            deme_t& deme = demes[i];

            // Discard the truly poor-scoring instances in each deme.
            trim_down_deme(deme);

            // Convert the instances in the deme to trees.
            deme_to_trees(deme, reps[j], pot_candidates);
        }
    }
    logger().debug("Selected %u candidate trees to be merged into the metapop",
                   pot_candidates.size());

    // Remove candidate trees that are already in the metapop.
    scored_combo_tree_set candidates = get_new_candidates(pot_candidates);
    logger().debug("Selected %u candidates (%u were already in the metapopulation)",
                   candidates.size(), pot_candidates.size()-candidates.size());

    // Behavioral scores are needed only if domination-based
    // merging is asked for, or if the diversity penalty is in use.
    // Save CPU time, as well as RAM, by not computing them.
    // Note that the instances were previously bscored; the bscores
    // were thrown away way back when, to save on RAM... so we have to
    // do it again here.
    if (diversity_enabled()
        or _params.do_boosting
        or _params.discard_dominated
        or _params.keep_bscore)
    {
        logger().debug("Compute behavioral score of %d selected candidates",
                       candidates.size());

        // XXX FIXME: we should use a pointer set for scored_combo_tree_set
        // This would avoid some pointless copying here and a few other
        // places.  This is easier said than done, because the stupid
        // domination code is so snarky and icky.  Domination should die.
#define PARALLEL_SCORE 1
#ifdef PARALLEL_SCORE
        std::mutex insert_mutex;
        scored_combo_tree_set new_pot;
        auto compute_bscore = [&, this](const scored_combo_tree& cand)
        {
            behavioral_score bs(this->_cscorer.get_bscore(cand.get_tree()));
            scored_combo_tree sct(cand.get_tree(),
                                  cand.get_demeID(),
                                  cand.get_composite_score(), bs);
            std::lock_guard<std::mutex> lock(insert_mutex);
            new_pot.insert(sct);
        };
        OMP_ALGO::for_each(candidates.begin(), candidates.end(),
                           compute_bscore);
        candidates = new_pot;
#else
        scored_combo_tree_set new_pot;
        for (const scored_combo_tree& cand : pot_candidates)
        {
            behavioral_score bs(_cscorer.get_bscore(cand.get_tree()));
            scored_combo_tree sct(cand.get_tree(),
                                  cand.get_demeID(),
                                  cand.get_composite_score(), bs);
            new_pot.insert(sct);
        }
        candidates = new_pot;
#endif
    }

    if (_params.discard_dominated) {

        // The final merge of the candidates into the metapop will
        // remove the dominated trees; what we do here is to trim down
        // the deme some more, so that the final merge can go faster.
        logger().debug("Remove dominated candidates");
        if (logger().isFineEnabled()) {
            std::stringstream ss;
            ss << "Candidates with their bscores before"
                " removing the dominated candidates" << std::endl;
            for (const auto& cnd : candidates)
                ss << cnd;
            logger().fine(ss.str());
        }

        size_t old_size = candidates.size();
        remove_dominated(candidates, _params.jobs);

        logger().debug("Removed %u dominated candidates out of %u",
                       old_size - candidates.size(), old_size);
        if (logger().isFineEnabled()) {
            std::stringstream ss;
            ss << "Candidates with their bscores after"
                " removing the dominated candidates" << std::endl;
            for (const auto& cnd : candidates)
                ss << cnd;
            logger().fine(ss.str());
        }
    }

    if (0 == candidates.size()) return false;

    // Update the record of the best-seen score & trees
    update_best_candidates(candidates);

    // Finally, merge the candidates into the metapop
    bool done = false;
    if (_params.merge_callback)
        done = (*_params.merge_callback)(candidates, _params.callback_user_data);
    merge_candidates(candidates);

    // Insert candidates into the ensemble.
    if (_params.do_boosting) {
        _ensemble.add_candidates(candidates);
        rescore();
    }

    // update diversity penalties
    if (diversity_enabled())
        set_diversity();

    // resize the metapopulation
    resize_metapop();

    return done;
}


// See header file for a desciption of this method.
void metapopulation::resize_metapop()
{
    if (size() <= _min_pool_size)
        return;

    unsigned old_size = size();
    logger().debug("Resize the metapopulation (current size=%u), "
                   "removing worst candidates",
                   old_size);

    // pointers to deallocate
    std::vector<scored_combo_tree*> ptr_seq;

    score_t top_score = _scored_trees.begin()->get_penalized_score();
    score_t range = useful_score_range();
    score_t worst_score = top_score - range;

    // Erase all the lowest scores.  The metapop is in quasi-sorted
    // order (since the deme was sorted before being appended), so
    // this bulk remove mostly works "correctly". It is also 25%
    // faster than above. I think this is because the erase() above
    // causes the std::set to try to sort the contents, and this
    // ends up costing a lot.  I think... not sure.

    // Get the first score below worst_score (from begin() + min_pool_size)
    scored_combo_tree_ptr_set::iterator it = std::next(_scored_trees.begin(), _min_pool_size);
    while (it != _scored_trees.end()) {
        score_t sc = it->get_penalized_score();
        if (sc < worst_score) break;
        it++;
    }

    while (it != _scored_trees.end()) {
        ptr_seq.push_back(&*it);
        it = _scored_trees.erase(it);
    }

    // Is the population still too large?  Yes, it is, if it is more
    // than cap as defined by the function of the number of
    // generations defined below
    //
    // popsize cap =  _params.cap_coef*(x+250)*(1+2*exp(-x/500))
    //
    // when x is the number of generations so far. The goal of capping
    // is to keep the metapop small enough that it does not blow out the
    // available RAM on the machine, but large enough that deme expansion
    // can always find some suitable exemplar to explore.  The above
    // formula was arrived at via some ad-hoc experimentation.  A default
    // value of _params.cap_coef=50 seems to work well.
    //
    // XXX TODO fix the cap so its more sensitive to the size of
    // each exemplar, right!? So if the exemplars are huges, then the
    // population size has to be smaller.  ... On the other hand, if
    // the exemplars are huge, then MOSES has probably wandered into
    // a bad corner, and is failing to explore a big enough space.
    //
    // size_t nbelts = get_bscore(*begin()).size();
    // double cap = 1.0e6 / double(nbelts);
    _merge_count++;
    double cap = _params.cap_coef;
    cap *= _merge_count + 250.0;
    cap *= 1 + 2.0*exp(- double(_merge_count) / 500.0);
    size_t popsz_cap = cap;
    size_t popsz = size();
    while (popsz_cap < popsz)
    {
        // Leave the first 50 alone.
        static const int offset = 50;
        int which = offset + randGen().randint(popsz-offset);
        // using std is necessary to break the ambiguity between
        // boost::next and std::next. Weirdly enough this appears
        // only 32bit arch
        scored_combo_tree_ptr_set::iterator it = std::next(_scored_trees.begin(), which);
        ptr_seq.push_back(&*it);
        _scored_trees.erase(it);
        popsz --;
    }

    // remove them from _cached_dst
    std::sort(ptr_seq.begin(), ptr_seq.end());
    _cached_dst.erase_ptr_seq(ptr_seq);

    if (logger().isDebugEnabled()) {
        logger().debug("Removed %u candidates from the metapopulation",
                       old_size - size());

        logger().debug("Metapopulation size is %u", size());
        if (logger().isFineEnabled()) {
            std::stringstream ss;
            ss << "Metapopulation:" << std::endl;
            ostream_metapop(ss);
            logger().fine(ss.str());
        }
    }
}

/// Given a set of candidates, return the set of candidates not already
/// present in the metapopulation.  This usually makes merging faster;
/// for example, if domination is enabled, this will result in fewer
/// calls to dominates().
scored_combo_tree_set metapopulation::get_new_candidates(const scored_combo_tree_set& mcs)
{

#define PARALLEL_INSERT 1
#ifdef PARALLEL_INSERT
    // Parallel insert, uses locking to avoid corruption.
    // This is probably faster than the lock-free version below,
    // but who knows ... (the version below is also parallel, just
    // that it's finer-grained than this one, which means its probably
    // slower!?)
    scored_combo_tree_set res;
    std::mutex insert_cnd_mutex;

    scored_combo_tree_ptr_set::const_iterator cbeg = _scored_trees.begin();
    scored_combo_tree_ptr_set::const_iterator cend = _scored_trees.end();
    auto insert_new_candidate = [&](const scored_combo_tree& cnd) {
        const combo_tree& tr = cnd.get_tree();
        scored_combo_tree_ptr_set::const_iterator fcnd =
            std::find_if(cbeg, cend,
                [&](const scored_combo_tree& v) { return tr == v.get_tree(); });
        if (fcnd == cend) {
            std::lock_guard<std::mutex> lock(insert_cnd_mutex);
            res.insert(cnd);
        }
    };
    OMP_ALGO::for_each(mcs.begin(), mcs.end(), insert_new_candidate);
    return res;

#else
    scored_combo_tree_set res;
    for (const auto& cnd : mcs) {
        const combo_tree& tr = cnd.get_tree();
        scored_combo_tree_ptr_set::const_iterator fcnd =
            OMP_ALGO::find_if(_scored_trees.begin(), _scored_trees.end(),
                              [&](const scored_combo_tree& v) {
                    return tr == v.get_tree(); });
        if (fcnd == _scored_trees.end())
            res.insert(cnd);
    }
    return res;
#endif
}

void metapopulation::sort_demes(std::vector<std::vector<deme_t>>& all_demes) {
    logger().debug("Sort the deme(s)");
    for (auto& ss_demes : all_demes)
        for (auto& deme : ss_demes)
            boost::sort(deme, std::greater<scored_instance<composite_score> >());
}

void metapopulation::keep_top_unique_candidates(
    std::vector<std::vector<deme_t>>& all_demes,
    const boost::ptr_vector<representation>& reps)
{
    for (unsigned i = 0; i < all_demes.size(); ++i) {
        const representation& rep = reps[i];
        std::vector<deme_t>& ss_demes = all_demes[i];
        for (auto& deme : ss_demes) {
            // Determine number of top unique candidate to keep

            // It can happen that the true number of evals is less than the
            // deme size (certain cases involving the univariate optimizer)
            // But also, the deme size can be smaller than the number of evals,
            // if the deme was shrunk to save space.
            unsigned top_cnd = std::min(deme.n_evals, (unsigned)deme.size());

            // If the user have specified to keep only the top candidates
            if (_params.max_candidates >= 0)
                top_cnd = std::min(top_cnd, (unsigned)_params.max_candidates);
            if (_filter_params.n_subsample_demes > 1)
                top_cnd = std::min(top_cnd,
                                   _filter_params.n_top_candidates);

            if (logger().isDebugEnabled())
            {
                std::stringstream ss;
                ss << "Keep " << top_cnd
                   << " top unique candidates from deme " << deme.getID()
                   << " of size: " << deme.size();
                logger().debug(ss.str());
            }

            // Remove duplicates till at most top_cnd unique
            // candidates remains
            for (auto it = deme.begin(), prev_it = it++;
                 it != deme.begin() + top_cnd;) {
                score_t sc = (select_tag()(*it)).get_penalized_score(),
                    prev_sc = (select_tag()(*prev_it)).get_penalized_score();
                if (isApproxEq(prev_sc, sc)) { // they might be identical
                    combo_tree tr = rep.get_candidate(*it, true),
                        prev_tr = rep.get_candidate(*prev_it, true);
                    if (prev_tr == tr) { // they actually are identical
                        it = deme.erase(it);
                        top_cnd = std::min(top_cnd, (unsigned)deme.size());
                        continue;
                    }
                }

// XXX FIXME looks to me like it++ can often be collaed twice within this loop!
                prev_it = it++;
            }

            // Remove the bottom
            deme.erase(deme.begin() + top_cnd, deme.end());

            if (logger().isDebugEnabled())
            {
                std::stringstream ss;
                ss << "Kept unique top candidates, new size: " << deme.size();
                logger().debug(ss.str());
            }
        }
    }
}

/// Update the record of the best score seen, and the associated tree.
/// Safe to call in a multi-threaded context.
void metapopulation::update_best_candidates(const scored_combo_tree_set& candidates)
{
    if (candidates.empty())
        return;

    // Make this routine thread-safe.
    // XXX this lock probably doesn't have to be the same one
    // that merge uses.  I think.
    std::lock_guard<std::mutex> lock(_merge_mutex);

    // Candidates are kept in penalized score order, not in
    // absolute score order.  Thus, we need to search through
    // the first few to find the true best score.  Also, there
    // may be several candidates with the best score.
    score_t best_sc = _best_cscore.get_score();
    complexity_t best_cpx = _best_cscore.get_complexity();

    for (const scored_combo_tree& cnd : candidates)
    {
        const composite_score& csc = cnd.get_composite_score();
        score_t sc = csc.get_score();
        complexity_t cpx = csc.get_complexity();
        if ((sc > best_sc) || ((sc == best_sc) && (cpx <= best_cpx)))
        {
            if ((sc > best_sc) || ((sc == best_sc) && (cpx < best_cpx)))
            {
                _best_cscore = csc;
                best_sc = _best_cscore.get_score();
                best_cpx = _best_cscore.get_complexity();
                _best_candidates.clear();
                logger().debug() << "New best score: " << _best_cscore;
            }
            _best_candidates.insert(cnd);
        }
    }
}

// log the best candidates
void metapopulation::log_best_candidates() const
{
    if (!logger().isInfoEnabled())
        return;

    if (best_candidates().empty())
        logger().info() << "No best candidates";
    else {
        if (not _params.do_boosting) {
            logger().info()
               << "The following candidate(s) have the best score "
               << best_composite_score();
            for (const auto& cand : best_candidates()) {
                logger().info() << cand.get_score() << " " << cand.get_tree();
            }
        } else {
            logger().info()
               << "Ensemble has " << _ensemble.get_ensemble().size()
               << " members, and a score of "
               << best_composite_score();
            if (logger().isDebugEnabled()) {
                logger().debug() << "The ensemble is " << std::endl;
                for (const auto& cand : _ensemble.get_ensemble()) {
                    logger().debug() << cand.get_weight() << " " << cand.get_tree();
                }
            }
        }
    }
}

} // ~namespace moses
} // ~namespace opencog

