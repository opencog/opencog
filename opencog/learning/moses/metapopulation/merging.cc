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

using namespace std;
using namespace combo;


void metapopulation::merge_candidates(scored_combo_tree_set& candidates)
{
    if (logger().isDebugEnabled()) {
        logger().debug("Going to merge %u candidates with the metapopulation",
                       candidates.size());
        if (logger().isFineEnabled()) {
            stringstream ss;
            ss << "Candidates to merge with the metapopulation:" << std::endl;
            for (const auto& cnd : candidates)
                ostream_scored_combo_tree(ss, cnd, true, true);
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

bool metapopulation::has_been_visited(const combo_tree& tr) const
{
    return _visited_exemplars.find(tr) != _visited_exemplars.cend();
}

/// Convert the instances in the deme into scored combo_trees.
/// While we are at it, we will also reject some obviously bad cases:
/// instances with a whacked score; combo trees that are also in the
/// already-visited list.  The rejection of combo trees that are already
/// in the metapop happens in another stage, and not here.
///
/// The resulting scored combo trees are added to pot_candidates
void metapopulation::deme_to_trees(deme_t& deme,
                                   const representation& rep,
                                   unsigned n_evals,
                                   scored_combo_tree_set& pot_candidates)
{
    // Use a shared mutex to allow multiple readers, but one writer
    // for the list.
    typedef boost::shared_mutex mutex;
    typedef boost::shared_lock<mutex> shared_lock;
    typedef boost::unique_lock<mutex> unique_lock;

    mutex pot_cnd_mutex; // mutex for pot_candidates

    // NB, this is an anonymous function. In particular, some
    // compilers require that members be explicitly referenced
    // with this-> as otherwise we get compile fails:
    // https://bugs.launchpad.net/bugs/933906
    auto select_candidates =
        [&, this](const scored_instance<composite_score>& inst)
    {
        const composite_score& inst_csc = inst.second;
        score_t inst_sc = inst_csc.get_score();
        // if it's really bad stops
        if (inst_sc <= very_worst_score || !isfinite(inst_sc))
            return;

        // Get the combo_tree associated to inst, cleaned and reduced.
        //
        // @todo: below, the candidate is reduced possibly for the
        // second time.  This second reduction could probably be
        // avoided with some clever cache or something. (or a flag?)
        combo_tree tr = rep.get_candidate(inst, true);

// XXX FIXME, this ios obviously broken ... 
        // Look for tr in the list of potential candidates.
        // Return true if not found.
        auto thread_safe_tr_not_found = [&]() {
            shared_lock lock(pot_cnd_mutex);
            scored_combo_tree sct(tr);
            return pot_candidates.find(sct) == pot_candidates.end();
        };

        // XXX To make merge_deme thread safe, this needs to be
        // locked too.  (to avoid collision with threads updating
        // _visited, e.g. the MPI case.
        bool not_already_visited = !this->has_been_visited(tr);

        // update the set of potential exemplars
        if (not_already_visited && thread_safe_tr_not_found()) {
            unique_lock lock(pot_cnd_mutex);
            scored_combo_tree sct(tr, deme.getID(), inst_csc);
            pot_candidates.insert(sct);
        }
    };

    // It can happen that the true number of evals is less than the
    // deme size (certain cases involving the univariate optimizer)
    // But also, the deme size can be smaller than the number of evals,
    // if the deme was shrunk to save space.
    unsigned max_pot_cnd = std::min(n_evals, (unsigned)deme.size());
    if (_params.max_candidates >= 0)
        max_pot_cnd = std::min(max_pot_cnd, (unsigned)_params.max_candidates);
    unsigned total_max_pot_cnd = pot_candidates.size() + max_pot_cnd;

    if (logger().isDebugEnabled()) {
        logger().debug() << "Select candidates from deme " << deme.getID()
                         << " to merge amongst " << max_pot_cnd;
    }

    // select_candidates() can be very time consuming; it currently
    // takes anywhere from 25 to 500(!!) millisecs per instance (!!)
    // for me; my (reduced, simplified) instances have complexity
    // of about 100. This seems too long/slow (circa summer 2012).
    //
    // We first select the top max_pot_cnd from the deme. But some
    // candidates will be redundant so in order to reach the
    // max_pot_cnd target we reiterate with max_pot_cnd -
    // pot_candidates.size(), till either the target is reached
    // (pot_candidates.size() == max_pot_cnd), or there is no more
    // candidate in the deme.
    //
    // Note that we really need to take that twisted road because
    // otherwise, if we just iterate in parallel till we get
    // enough candidates, it can create race conditions
    // (indeterminism)
    typedef deme_t::const_iterator deme_cit;
    for (deme_cit deme_begin = deme.cbegin(),
             deme_end = deme_begin + max_pot_cnd;
         deme_begin != deme.cend()
             && pot_candidates.size() < total_max_pot_cnd;)
    {
        // logger().debug("ITERATING TILL TARGET REACHED (%u/%u) "
        //                "pot_candidates.size() = %u",
        //                i, demes.size(), pot_candidates.size());


        // select candidates in range = [deme_being, deme_end)
        OMP_ALGO::for_each(deme_begin, deme_end, select_candidates);

        // update range
        OC_ASSERT(pot_candidates.size() <= total_max_pot_cnd,
                  "there must be a bug");
        unsigned delta = total_max_pot_cnd - pot_candidates.size();
        deme_begin = deme_end;
        deme_end = (unsigned int)std::distance(deme_begin, deme.cend()) <= delta ?
            deme.end() : deme_begin + delta;
    }
}  

bool metapopulation::merge_demes(boost::ptr_vector<deme_t>& demes,
                                 const boost::ptr_vector<representation>& reps,
                                 const vector<unsigned>& evals_seq)
{
    // Note that univariate reports far more evals than the deme size;
    // this is because univariate over-writes deme entries.
    logger().debug("Close deme(s); evaluations reported: %d",
                   boost::accumulate(evals_seq, 0U));

    // Discard the truly poor-scoring instances in each deme.
    for (deme_t& deme : demes) {
        trim_down_deme(deme);
    }

    // Convert the instances in the deme to trees.
    scored_combo_tree_set pot_candidates;
    for (unsigned i = 0; i < demes.size(); i++) {
        deme_to_trees(demes[i], reps[i], evals_seq[i], pot_candidates);
    }

    logger().debug("Selected %u candidates to be merged",
                   pot_candidates.size());

    // Behavioral scores are needed only if domination-based
    // merging is asked for, or if the diversity penalty is in use.
    // Save CPU time by not computing them.
    if (_params.keep_bscore
        or _params.discard_dominated
        or diversity_enabled())
    {
        logger().debug("Compute behavioral score of %d selected candidates",
                       pot_candidates.size());

#ifdef FIXME_LATER
        // XXX TODO FIXME: the scores should be computed in-place, above,
        // instead of involving copies.  This needs a lock.
        scored_combo_tree_set new_pot;
        auto compute_bscore = [this, new_pot](const scored_combo_tree& cand) {
            behavioral_score bs = this->_bscorer(cand.get_tree());
            scored_combo_tree sct(cand.get_tree(),
                                  cand.get_demeID(),
                                  cand.get_composite_score(), bs);
            // XXX this needs to be locked!
            new_pot.insert(sct);
        };
        OMP_ALGO::for_each(pot_candidates.begin(), pot_candidates.end(),
                           compute_bscore);
        pot_candidates = new_pot;
#endif
#if WTF
// XXX WTF compiler dislikes the below, it keeps complaining about constness.
// I can't figure it out!
        for (scored_combo_tree& cand : pot_candidates)
        {
            behavioral_score bs(_cscorer.get_bscore(cand.get_tree()));
            cand._bscore = bs;
        }
#endif
        scored_combo_tree_set new_pot;
        for (const scored_combo_tree& cand : pot_candidates)
        {
            behavioral_score bs(_cscorer.get_bscore(cand.get_tree()));
            scored_combo_tree sct(cand.get_tree(),
                                  cand.get_demeID(), 
                                  cand.get_composite_score(), bs);
            new_pot.insert(sct);
        }
        pot_candidates = new_pot;
    }

    // XXX FIXME TODO: we should reverse the step below and the step above,
    // so that we only compute the bscores on the cands that are not yet
    // in the metapop.
    logger().debug("Select only candidates not already in the metapopulation");
    scored_combo_tree_set candidates = get_new_candidates(pot_candidates);
    logger().debug("Selected %u candidates (%u were in the metapopulation)",
                   candidates.size(), pot_candidates.size()-candidates.size());

    if (_params.discard_dominated) {

        logger().debug("Remove dominated candidates");
        if (logger().isFineEnabled()) {
            stringstream ss;
            ss << "Candidates with their bscores before"
                " removing the dominated candidates" << std::endl;
            for (const auto& cnd : candidates)
                ostream_scored_combo_tree(ss, cnd, true, true, true);
            logger().fine(ss.str());
        }

        size_t old_size = candidates.size();
        remove_dominated(candidates, _params.jobs);

        logger().debug("Removed %u dominated candidates out of %u",
                       old_size - candidates.size(), old_size);
        if (logger().isFineEnabled()) {
            stringstream ss;
            ss << "Candidates with their bscores after"
                " removing the dominated candidates" << std::endl;
            for (const auto& cnd : candidates)
                ostream_scored_combo_tree(ss, cnd, true, true, true);
            logger().fine(ss.str());
        }
    }

    // update the record of the best-seen score & trees
    update_best_candidates(candidates);

    bool done = false;
    if (_params.merge_callback)
        done = (*_params.merge_callback)(candidates, _params.callback_user_data);
    merge_candidates(candidates);

    // update diversity penalties
    if (diversity_enabled())
        set_diversity();

    // resize the metapopulation
    resize_metapop();

    return done;
}

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
            stringstream ss;
            ss << "Metapopulation:" << std::endl;
            logger().fine(ostream(ss, -1, true, true).str());
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
            std::unique_lock<mutex> lock(insert_cnd_mutex);
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

/// Trim the demes down to size.  The point here is that the next
/// stage, select_candidates, is very cpu-intensive; we should keep
/// only those candidates that will survive in the metapop.  But what
/// are these?  Well, select_exemplar() uses an exponential choice
/// function; instances below a cut-off score have no chance at all
/// of getting selected. So just eliminate them now, instead of later.
///
/// However, trimming too much is bad: it can happen that none
/// of the best-scoring instances lead to a solution. So keep
/// around a reasonable pool. Wild choice ot 250 seems reasonable.
/// (_min_pool_size defaults to 250)
void metapopulation::trim_down_deme(deme_t& deme) const
{
    // Don't bother, if the deme is tiny.
    if (_min_pool_size >= deme.size())
        return;

    if (logger().isDebugEnabled())
    {
        stringstream ss;
        ss << "Trim down deme " << deme.getID()
           << " of size: " << deme.size();
        logger().debug(ss.str());
    }

    // Sort the deme according to composite_score (descending order)
    // We pop the poorest scorers off the back, below.
    std::sort(deme.begin(), deme.end(),
              std::greater<scored_instance<composite_score> >());

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
        stringstream ss;
        ss << "Deme trimmed down, new size: " << deme.size();
        logger().debug(ss.str());
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
        logger().info("No new best candidates");
    else {
        logger().info()
           << "The following candidate(s) have the best score "
           << best_composite_score();
        for (const auto& cand : best_candidates()) {
            logger().info() << "" << cand.get_tree();
        }
    }
}

} // ~namespace moses
} // ~namespace opencog

