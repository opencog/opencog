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
#include <future>

#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/range/algorithm/find_if.hpp>

#include <opencog/util/oc_omp.h>
#include <opencog/util/selection.h>
#include <opencog/util/numeric.h>

#include "metapopulation.h"

namespace opencog {
namespace moses {

using std::pair;
using std::make_pair;
using boost::logic::tribool;
using boost::logic::indeterminate;
using namespace combo;

// Init the metapopulation with the following set of exemplars.
void metapopulation::init(const std::vector<combo_tree>& exemplars,
                          const reduct::rule& simplify_candidate,
                          const cscore_base& cscorer)
{
    metapop_candidates candidates;
    for (const combo_tree& base : exemplars) {
        combo_tree si_base(base);
        simplify_candidate(si_base);

        penalized_bscore pbs(_bscorer(si_base));
        // XXX Compute the bscore a second time.   The first time
        // was immediately above.  We do it again, because the
        // caching scorer lacks the correct signature.
        // composite_score csc(_cscorer (pbs, tree_complexity(si_base)));
        composite_score csc(cscorer(si_base));
        composite_penalized_bscore cpb(pbs, csc);
        cpbscore_demeID cbs_demeID(cpb, demeID_t());
        
        candidates[si_base] = cbs_demeID;
    }

    pbscored_combo_tree_set mps(candidates.begin(), candidates.end());
    for (const auto& cnd : candidates) {
        cpbscore_demeID cbs_demeID(get_composite_penalized_bscore(cnd),
                                   demeID_t());
        pbscored_combo_tree pct(get_tree(cnd), cbs_demeID);
        mps.insert(pct);
    }
    update_best_candidates(mps);
    merge_candidates(mps);
}
void metapopulation::set_diversity()
{
    logger().debug("Compute diversity penalties of the metapopulation");
    
    pbscored_combo_tree_ptr_set pool; // new metapopulation

    // structure to remember a partially aggredated distorted
    // diversity penalties between the candidates and the ones in
    // the pool (to avoid recomputing them)
    typedef pbscored_combo_tree_ptr_set_it psi;
    typedef std::pair<psi, dp_t> bsct_dp_pair;
    std::vector<bsct_dp_pair> tmp;
    for (psi bsct_it = begin(); bsct_it != end(); ++bsct_it)
        tmp.push_back(bsct_dp_pair(bsct_it, 0.0));
    
    // pointer to the last candidate moved from tmp to pool
    const pbscored_combo_tree* last_ptr(nullptr);

    // // debug
    // std::atomic<unsigned> dp_count(0); // count the number of
    //                                    // calls of _cached_dst
    // unsigned lhits = _cached_dst.hits.load(),
    //     lmisses = _cached_dst.misses.load();
    // // ~debug

    // if diversity_exponent is negative or null then the generalized
    // mean is replaced by the max
    bool dp_max = params.diversity.exponent <= 0.0;

    auto update_diversity_penalty = [&](bsct_dp_pair& v) {

        if (!pool.empty()) { // only do something if the pool is
                             // not empty (WARNING: this assumes
                             // that all diversity penalties are
                             // initially zero)

            pbscored_combo_tree& bsct = *v.first;
            OC_ASSERT(get_bscore(bsct).size(),
                      "Behavioral score is needed for diversity!");

            // compute diversity penalty between bs and the last
            // element of the pool
            dp_t last_dst = this->_cached_dst(&bsct, last_ptr);
            OC_ASSERT(last_dst >= 0.0, "The distance cannot be negative."
                      "There could be a bug or you are not using a true distance. "
                      "For instance the Tanimoto distance may be negative "
                      "when some of its components are negative. "
                      "If that is the case you might want to switch to the angular "
                      "distance.");
            dp_t last_dp = params.diversity.dst2dp(last_dst),
            last_ddp = dp_max ? last_dp : pow(last_dp, params.diversity.exponent);

            // // debug
            // ++dp_count;
            // // ~debug

            // add it to v.second and compute the aggregated
            // diversity penalty
            dp_t adp;
            if (dp_max) {
                v.second = std::max(v.second, last_ddp);
                adp = v.second;
            } else {
                v.second += last_ddp;
                unsigned N = params.diversity.normalize ? pool.size() : 1;
                adp = this->aggregated_dps(v.second, N);
            }

            // update v.first
            if (params.diversity.dst2dp_type == params.diversity.pthpower)
                get_composite_score(bsct).multiply_diversity = true;
            get_composite_score(bsct).set_diversity_penalty(adp);

            if (logger().isFineEnabled()) {
                stringstream ss;
                ss << "Diversity for candidate: " << get_tree(bsct)
                   << ", last_dst = " << last_dst
                   << ", last_dp = " << last_dp
                   << ", last_ddp = " << last_ddp
                   << ", adp = " << adp;
                logger().fine(ss.str());
            }
        }
    };

    while (tmp.size()) {

        // // debug
        // logger().fine("Pool =");
        // for (pbscored_combo_tree* ptr : pool) {
        //     stringstream ss;
        //     ostream_pbscored_combo_tree(ss, *ptr, true, true, true);
        //     logger().fine(ss.str());
        // }
        // // ~debug

        // update all diversity penalties of tmp
        OMP_ALGO::for_each(tmp.begin(), tmp.end(), update_diversity_penalty);

        // take the max score, insert in the pool and remove from tmp
        pbscored_combo_tree_greater bsct_gt;
        auto gt = [&](const bsct_dp_pair& l,
                      const bsct_dp_pair& r) {
            return bsct_gt(*l.first, *r.first);
        };
        // note although we do use min_element it returns the
        // candidate with the best penalized score because we use
        // a greater_than function instead of a less_than function
        auto mit = OMP_ALGO::min_element(tmp.begin(), tmp.end(), gt);
        // remember the last one added in the pool to calculate its
        // distance with the other in tmp
        last_ptr = &*mit->first;
        // move the last candidate to the pool and remove it from tmp
        pool.transfer(mit->first, *this);
        tmp.erase(mit);
    }

    // // debug
    // logger().debug() << "Number of dst evals = " << dp_count;
    // logger().debug() << "Is lock free? " << dp_count.is_lock_free();
    // logger().debug() << "Number of hits = "
    //                  << _cached_dst.hits.load() - lhits;
    // logger().debug() << "Number of misses = "
    //                  << _cached_dst.misses.load() - lmisses;
    // logger().debug() << "Total number of hits = " << _cached_dst.hits;
    // logger().debug() << "Total number of misses = " << _cached_dst.misses;
    // // ~debug

    // Replace the existing metapopulation with the new one.
    swap(pool);

    if (logger().isFineEnabled()) {
        stringstream ss;
        ss << "Metapopulation after setting diversity:" << std::endl;
        logger().fine(ostream(ss, -1, true, true, true /*bscore*/).str());
    }
}

void metapopulation::log_selected_exemplar(const_iterator exemplar_it)
{
    if (exemplar_it == cend()) {
        logger().debug() << "No exemplar found";
    } else {
        const auto& xmplr = *exemplar_it;
        unsigned pos = std::distance(cbegin(), exemplar_it) + 1,
            nth_vst = _visited_exemplars[get_tree(xmplr)];

        logger().debug() << "Selected the " << pos
                         << "th exemplar, from deme " << get_demeID(xmplr)
                         << ", for the " << nth_vst << "th time(s)";
        logger().debug() << "Exemplar tree : " << get_tree(xmplr);
        logger().debug() << "With composite score : "
                         << get_composite_score(xmplr);
    }
}

pbscored_combo_tree_ptr_set::const_iterator metapopulation::select_exemplar()
{
    OC_ASSERT(!empty(), "Empty metapopulation in select_exemplar().");

    logger().debug("Select exemplar");

    // Shortcut for special case, as sometimes, the very first time
    // though, the score is invalid.
    if (size() == 1) {
        const_iterator selex = cbegin();
        const combo_tree& tr = get_tree(*selex);
        if(params.revisit + 1 > _visited_exemplars[tr]) // not enough visited
            _visited_exemplars[tr]++;
        else selex = cend();    // enough visited

        log_selected_exemplar(selex);
        return selex;
    }

    vector<score_t> probs;
    // Set flag to true, when a suitable exemplar is found.
    bool found_exemplar = false;
#define UNEVALUATED_SCORE -1.0e37
    score_t highest_score = UNEVALUATED_SCORE;

    // The exemplars are stored in order from best score to worst;
    // the iterator follows this order.
    for (const pbscored_combo_tree& bsct : *this) {

        score_t sc = get_penalized_score(bsct);

        // Skip exemplars that have been visited enough
        const combo_tree& tr = get_tree(bsct);
        if (params.revisit + 1 > _visited_exemplars[tr]) {
            probs.push_back(sc);
            found_exemplar = true;
            if (highest_score < sc) highest_score = sc;
        } else // hack: if the tree is visited enough then put a
               // positive score so we know it must be ignored
#define SKIP_OVER_ME (1.0e38)
            probs.push_back(SKIP_OVER_ME);
    }

    // Nothing found, we've already tried them all.
    if (!found_exemplar) {
        log_selected_exemplar(cend());
        return cend();
    }

    // Compute the probability normalization, needed for the
    // roullete choice of exemplars with equal scores, but
    // differing complexities. Empirical work on 4-parity suggests
    // that a temperature of 3 or 4 works best.
    score_t inv_temp = 100.0f / params.complexity_temperature;
    score_t sum = 0.0f;
    // Convert scores into (non-normalized) probabilities
    for (score_t& p : probs) {
        // In case p has the max complexity (already visited) then
        // the probability is set to null
        p = (p > (0.1*SKIP_OVER_ME) ? 0.0f : expf((p - highest_score) * inv_temp));
        sum += p;
    }

    // log the distribution probs
    if (logger().isFineEnabled())
    {
        stringstream ss;
        ss << "Non-normalized probability distribution of candidate selection: ";
        ostreamContainer(ss, probs);
        logger().fine() << ss.str();
    }

    OC_ASSERT(sum > 0.0f, "There is an internal bug, please fix it");

    size_t fwd = distance(probs.begin(), roulette_select(probs.begin(),
                                                         probs.end(),
                                                         sum, randGen()));
    // cout << "select_exemplar(): sum=" << sum << " fwd =" << fwd
    // << " size=" << probs.size() << " frac=" << fwd/((float)probs.size()) << endl;
    const_iterator selex = std::next(begin(), fwd);

    // We increment _visited_exemplar
    _visited_exemplars[get_tree(*selex)]++;

    log_selected_exemplar(selex);
    return selex;
}

void metapopulation::merge_candidates(pbscored_combo_tree_set& candidates)
{
    if (logger().isDebugEnabled()) {
        logger().debug("Going to merge %u candidates with the metapopulation",
                       candidates.size());
        if (logger().isFineEnabled()) {
            stringstream ss;
            ss << "Candidates to merge with the metapopulation:" << std::endl;
            for (const auto& cnd : candidates)
                ostream_pbscored_combo_tree(ss, cnd, true, true);
            logger().fine(ss.str());
        }
    }
    
    // Serialize access
    std::lock_guard<std::mutex> lock(_merge_mutex);
    
    // Note that merge_nondominated() is very cpu-expensive and
    // complex...
    if (params.diversity.include_dominated) {
        logger().debug("Insert all candidates in the metapopulation");
        for (const auto& cnd : candidates)
            insert(new pbscored_combo_tree(cnd));
    } else {
        logger().debug("Insert non-dominated candidates in the metapopulation");
        unsigned old_size = size();
        merge_nondominated(candidates, params.jobs);
        logger().debug("Inserted %u non-dominated candidates "
                       "in the metapopulation", size() - old_size);
    }
}

bool metapopulation::merge_demes(boost::ptr_vector<deme_t>& demes,
                                 const boost::ptr_vector<representation>& reps,
                                 const vector<unsigned>& evals_seq)
{
    // Note that univariate reports far more evals than the deme size;
    // this is because univariate over-write deme entries.
    logger().debug("Close deme(s); evaluations reported: %d",
                   boost::accumulate(evals_seq, 0U));

    // Add, as potential exemplars for future demes, all unique
    // trees in the final deme.
    metapop_candidates pot_candidates;


    logger().debug("Sort the deme(s)");

    // Sort the deme according to composite_score (descending order)
    for (deme_t& deme : demes)
        boost::sort(deme, std::greater<scored_instance<composite_score> >());

    trim_down_demes(demes);
    
    ///////////////////////////////////////////////////////////////
    // select the set of candidates to add in the metapopulation //
    ///////////////////////////////////////////////////////////////
    typedef boost::shared_mutex mutex;
    typedef boost::shared_lock<mutex> shared_lock;
    typedef boost::unique_lock<mutex> unique_lock;

    mutex pot_cnd_mutex; // mutex for pot_candidates

    for (unsigned i = 0; i < demes.size(); i++) {
        // NB, this is an anonymous function. In particular, some
        // compilers require that members be explicitly referenced
        // with this-> as otherwise we get compile fails:
        // https://bugs.launchpad.net/bugs/933906
        auto select_candidates =
            [&, this](const scored_instance<composite_score>& inst) {
            
            const composite_score& inst_csc = inst.second;
            score_t inst_sc = get_score(inst_csc);
            // if it's really bad stops
            if (inst_sc <= very_worst_score || !isfinite(inst_sc))
                return;

            // Get the combo_tree associated to inst, cleaned and reduced.
            //
            // @todo: below, the candidate is reduced possibly for the
            // second time.  This second reduction could probably be
            // avoided with some clever cache or something. (or a flag?)
            combo_tree tr = reps[i].get_candidate(inst, true);

            // Look for tr in the list of potential candidates.
            // Return true if not found.
            auto thread_safe_tr_not_found = [&]() {
                shared_lock lock(pot_cnd_mutex);
                return pot_candidates.find(tr) == pot_candidates.end();
            };

            // XXX To make merge_deme thread safe, this needs to be
            // locked too.  (to avoid collision with threads updating
            // _visited, e.g. the MPI case.
            bool not_already_visited = !this->has_been_visited(tr);
            
            // update the set of potential exemplars
            if (not_already_visited && thread_safe_tr_not_found()) {
                penalized_bscore pbs; // empty bscore till
                                      // it gets computed
                composite_penalized_bscore cbsc(pbs, inst_csc);
                cpbscore_demeID cbs_demeID(cbsc, demes[i].getID());

                unique_lock lock(pot_cnd_mutex);
                pot_candidates[tr] = cbs_demeID;
            }
        };

        // It can happen that the true number of evals is less than the
        // deme size (certain cases involving the univariate optimizer)
        // But also, the deme size can be smaller than the number of evals,
        // if the deme was shrunk to save space. 
        unsigned max_pot_cnd = std::min(evals_seq[i], (unsigned)demes[i].size());
        if (params.max_candidates >= 0)
            max_pot_cnd = std::min(max_pot_cnd, (unsigned)params.max_candidates);
        unsigned total_max_pot_cnd = pot_candidates.size() + max_pot_cnd;

        stringstream ss;
        ss << "Select candidates from deme " << demes[i].getID()
           << " to merge amongst " << max_pot_cnd;
        logger().debug(ss.str());

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
        for (deme_cit deme_begin = demes[i].cbegin(),
                 deme_end = deme_begin + max_pot_cnd;
             deme_begin != demes[i].cend()
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
            deme_end = (unsigned int)std::distance(deme_begin, demes[i].cend()) <= delta ?
                demes[i].end() : deme_begin + delta;
        }
    }
    
    logger().debug("Selected %u candidates to be merged",
                   pot_candidates.size());

    // Behavioural scores are needed only if domination-based
    // merging is asked for, or if the diversity penalty is in use.
    // Save CPU time by not computing them.
    if (params.keep_bscore
        || !params.diversity.include_dominated
        || params.diversity.pressure > 0.0) {
        logger().debug("Compute behavioral score of %d selected candidates",
                       pot_candidates.size());

        auto compute_bscore = [this](metapop_candidates::value_type& cand) {
            penalized_bscore pbs = this->_bscorer(get_tree(cand));
            composite_penalized_bscore cpb(pbs, get_composite_score(cand));
            cand.second = cpbscore_demeID(cpb, get_demeID(cand));
        };
        OMP_ALGO::for_each(pot_candidates.begin(), pot_candidates.end(),
                           compute_bscore);
    }

    logger().debug("Select only candidates not already in the metapopulation");
    pbscored_combo_tree_set candidates = get_new_candidates(pot_candidates);
    logger().debug("Selected %u candidates (%u were in the metapopulation)",
                   candidates.size(), pot_candidates.size()-candidates.size());

    if (!params.diversity.include_dominated) {

        logger().debug("Remove dominated candidates");
        if (logger().isFineEnabled()) {
            stringstream ss;
            ss << "Candidates with their bscores before"
                " removing the dominated candidates" << std::endl;
            for (const auto& cnd : candidates)
                ostream_pbscored_combo_tree(ss, cnd, true, true, true);
            logger().fine(ss.str());
        }

        size_t old_size = candidates.size();
        remove_dominated(candidates, params.jobs);

        logger().debug("Removed %u dominated candidates out of %u",
                       old_size - candidates.size(), old_size);
        if (logger().isFineEnabled()) {
            stringstream ss;
            ss << "Candidates with their bscores after"
                " removing the dominated candidates" << std::endl;
            for (const auto& cnd : candidates)
                ostream_pbscored_combo_tree(ss, cnd, true, true, true);
            logger().fine(ss.str());
        }
    }

    // update the record of the best-seen score & trees
    update_best_candidates(candidates);

    bool done = false;
    if (params.merge_callback)
        done = (*params.merge_callback)(candidates, params.callback_user_data);
    merge_candidates(candidates);

    // update diversity penalties
    if (params.diversity.pressure > 0.0)
        set_diversity();

    // resize the metapopulation
    resize_metapop();

    return done;
}

void metapopulation::resize_metapop()
{    
    if (size() <= min_pool_size)
        return;

    unsigned old_size = size();
    logger().debug("Resize the metapopulation (current size=%u), "
                   "removing worst candidates",
                   old_size);

    // pointers to deallocate
    std::vector<pbscored_combo_tree*> ptr_seq;

    score_t top_score = get_penalized_score(*begin());
    score_t range = useful_score_range();
    score_t worst_score = top_score - range;

    // Erase all the lowest scores.  The metapop is in quasi-sorted
    // order (since the deme was sorted before being appended), so
    // this bulk remove mostly works "correctly". It is also 25%
    // faster than above. I think this is because the erase() above
    // causes the std::set to try to sort the contents, and this
    // ends up costing a lot.  I think... not sure.
    iterator it = std::next(begin(), min_pool_size);
    while (it != end()) {
        score_t sc = get_penalized_score(*it);
        if (sc < worst_score) break;
        it++;
    }

    while (it != end()) {
        ptr_seq.push_back(&*it);
        it = erase(it);
    }

    // Is the population still too large?  Yes, it is, if it is more
    // than cap as defined by the function of the number of
    // generations defined below
    //
    // popsize cap =  params.cap_coef*(x+250)*(1+2*exp(-x/500))
    //
    // when x is the number of generations so far.
    //
    // XXX TODO fix the cap so its more sensitive to the size of
    // each exemplar, right!?
    // size_t nbelts = get_bscore(*begin()).size();
    // double cap = 1.0e6 / double(nbelts);
    _merge_count++;
    double cap = params.cap_coef;
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
        iterator it = std::next(begin(), which);
        ptr_seq.push_back(&*it);
        erase(it);
        popsz --;
    }

    // remove them from _cached_dst
    boost::sort(ptr_seq);
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

// Return the set of candidates not present in the metapopulation.
// This makes merging faster because at best it decreases the number
// of calls of dominates.
pbscored_combo_tree_set metapopulation::get_new_candidates(const metapop_candidates& mcs)
{
    pbscored_combo_tree_set res;
    for (const auto& cnd : mcs) {
        const combo_tree& tr = get_tree(cnd);
        const_iterator fcnd =
            OMP_ALGO::find_if(begin(), end(), [&](const pbscored_combo_tree& v) {
                    return tr == get_tree(v); });
        if (fcnd == end())
            res.insert(cnd);
    }
    return res;
}

// reciprocal of random_access_view
pbscored_combo_tree_set
metapopulation::to_set(const pbscored_combo_tree_ptr_vec& bcv)
{
    pbscored_combo_tree_set res;
    for (const pbscored_combo_tree* cnd : bcv)
        res.insert(*cnd);
    return res;
}

void metapopulation::remove_dominated(pbscored_combo_tree_set& bcs, unsigned jobs)
{
    // get the nondominated candidates
    pbscored_combo_tree_ptr_vec bcv = random_access_view(bcs);
    pbscored_combo_tree_ptr_vec res = get_nondominated_rec(bcv, jobs);
    // get the dominated by set difference
    boost::sort(bcv); boost::sort(res);
    pbscored_combo_tree_ptr_vec dif = set_difference(bcv, res);
    // remove the dominated ones
    for (const pbscored_combo_tree* cnd_ptr : dif)
        bcs.erase(*cnd_ptr);
}

pbscored_combo_tree_set
metapopulation::get_nondominated_iter(const pbscored_combo_tree_set& bcs)
{
    typedef std::list<pbscored_combo_tree> pbscored_combo_tree_list;
    typedef pbscored_combo_tree_list::iterator pbscored_combo_tree_list_it;
    pbscored_combo_tree_list mcl(bcs.begin(), bcs.end());
    // remove all dominated candidates from the list
    for(pbscored_combo_tree_list_it it1 = mcl.begin(); it1 != mcl.end();) {
        pbscored_combo_tree_list_it it2 = it1;
        ++it2;
        if(it2 != mcl.end())
            for(; it2 != mcl.end();) {
                tribool dom = dominates(get_bscore(*it1), get_bscore(*it2));
                if(dom)
                    it2 = mcl.erase(it2);
                else if(!dom) {
                    it1 = mcl.erase(it1);
                    it2 = mcl.end();
                } else
                    ++it2;
                if(it2 == mcl.end())
                    ++it1;
            }
        else
            ++it1;
    }
    return pbscored_combo_tree_set(mcl.begin(), mcl.end());
}

typedef pair<pbscored_combo_tree_set,
             pbscored_combo_tree_set> pbscored_combo_tree_set_pair;

typedef std::vector<const pbscored_combo_tree*> pbscored_combo_tree_ptr_vec;
typedef pbscored_combo_tree_ptr_vec::iterator pbscored_combo_tree_ptr_vec_it;
typedef pbscored_combo_tree_ptr_vec::const_iterator pbscored_combo_tree_ptr_vec_cit;
typedef pair<pbscored_combo_tree_ptr_vec,
             pbscored_combo_tree_ptr_vec> pbscored_combo_tree_ptr_vec_pair;


pbscored_combo_tree_ptr_vec
metapopulation::get_nondominated_rec(const pbscored_combo_tree_ptr_vec& bcv,
                     unsigned jobs)
{
    ///////////////
    // base case //
    ///////////////
    if (bcv.size() < 2) {
        return bcv;
    }
    //////////////
    // rec case //
    //////////////
//  The names in enum std::launch have not yet been standardized.
#if defined(__GNUC__) && (__GNUC__ == 4) && (__GNUC_MINOR__ >= 5) && (__GNUC_MINOR__ < 7)
 #define LAUNCH_SYNC std::launch::sync
#else
 #define LAUNCH_SYNC std::launch::deferred
#endif
    pbscored_combo_tree_ptr_vec_pair bcv_p = split(bcv);
    if (jobs > 1) { // multi-threaded
        auto s_jobs = split_jobs(jobs); // pair
        // recursive calls
        std::future<pbscored_combo_tree_ptr_vec> task =
            std::async(jobs > 1 ? std::launch::async : LAUNCH_SYNC,
                       bind(&metapopulation::get_nondominated_rec, this,
                            bcv_p.first, s_jobs.first));
        pbscored_combo_tree_ptr_vec bcv2_nd =
            get_nondominated_rec(bcv_p.second, s_jobs.second);
        pbscored_combo_tree_ptr_vec_pair res_p =
            get_nondominated_disjoint_rec(task.get(), bcv2_nd, jobs);
        // union and return
        append(res_p.first, res_p.second);
        return res_p.first;
    } else { // single-threaded
        // recursive calls
        pbscored_combo_tree_ptr_vec
            bcv1_nd = get_nondominated_rec(bcv_p.first),
            bcv2_nd = get_nondominated_rec(bcv_p.second);
        pbscored_combo_tree_ptr_vec_pair
            res_p = get_nondominated_disjoint_rec(bcv1_nd, bcv2_nd);
        // union and return
        append(res_p.first, res_p.second);
        return res_p.first;
    }
}

pbscored_combo_tree_set_pair
metapopulation::get_nondominated_disjoint(const pbscored_combo_tree_set& bcs1,
                          const pbscored_combo_tree_set& bcs2,
                          unsigned jobs)
{
    pbscored_combo_tree_ptr_vec_pair res_p =
        get_nondominated_disjoint_rec(random_access_view(bcs1),
                                      random_access_view(bcs2),
                                      jobs);
    return make_pair(to_set(res_p.first), to_set(res_p.second));
}

pbscored_combo_tree_ptr_vec_pair
metapopulation::get_nondominated_disjoint_rec(const pbscored_combo_tree_ptr_vec& bcv1,
                              const pbscored_combo_tree_ptr_vec& bcv2,
                              unsigned jobs)
{
    ///////////////
    // base case //
    ///////////////
    if (bcv1.empty() || bcv2.empty())
        return make_pair(bcv1, bcv2);
    else if (bcv1.size() == 1) {
        pbscored_combo_tree_ptr_vec bcv_res1, bcv_res2;
        pbscored_combo_tree_ptr_vec_cit it1 = bcv1.begin(),
            it2 = bcv2.begin();
        bool it1_insert = true; // whether *it1 is to be inserted
                                // in bcv_res1
        for (; it2 != bcv2.end(); ++it2) {
            tribool dom = dominates(get_pbscore(**it1).first, get_pbscore(**it2).first);
            if (!dom) {
                it1_insert = false;
                bcv_res2.insert(bcv_res2.end(), it2, bcv2.end());
                break;
            } else if (indeterminate(dom))
                bcv_res2.push_back(*it2);
        }
        if (it1_insert)
            bcv_res1.push_back(*it1);
        return make_pair(bcv_res1, bcv_res2);
    }
    //////////////
    // rec case //
    //////////////
    // split bcs1 in 2
    pbscored_combo_tree_ptr_vec_pair bcv1_p = split(bcv1);
    if(jobs > 1) { // multi-threaded
        unsigned jobs1 = jobs / 2;
        unsigned jobs2 = std::max(1U, jobs - jobs1);
        std::future<pbscored_combo_tree_ptr_vec_pair> task =
            std::async(std::launch::async,
                       bind(&metapopulation::get_nondominated_disjoint_rec, this,
                            bcv1_p.first, bcv2, jobs1));
        pbscored_combo_tree_ptr_vec_pair bcv_m2 =
            get_nondominated_disjoint_rec(bcv1_p.second, bcv2, jobs2);
        pbscored_combo_tree_ptr_vec_pair bcv_m1 = task.get();
        // merge results
        append(bcv_m1.first, bcv_m2.first);
        boost::sort(bcv_m1.second); boost::sort(bcv_m2.second);
        pbscored_combo_tree_ptr_vec bcv_m2_inter =
            set_intersection(bcv_m1.second, bcv_m2.second);
        return make_pair(bcv_m1.first, bcv_m2_inter);
    } else { // single-threaded
        pbscored_combo_tree_ptr_vec_pair
            bcv_m1 = get_nondominated_disjoint_rec(bcv1_p.first, bcv2),
            bcv_m2 = get_nondominated_disjoint_rec(bcv1_p.second,
                                                   bcv_m1.second);
        // merge results
        append(bcv_m1.first, bcv_m2.first);
        return make_pair(bcv_m1.first, bcv_m2.second);
    }
}

// merge nondominated candidate to the metapopulation assuming
// that bcs contains no dominated candidates within itself
void metapopulation::merge_nondominated(const pbscored_combo_tree_set& bcs, unsigned jobs)
{
    pbscored_combo_tree_ptr_vec bcv = random_access_view(bcs);
    pbscored_combo_tree_ptr_vec bcv_mp;
    for (const pbscored_combo_tree& cnd : *this)
        bcv_mp.push_back(&cnd);
    pbscored_combo_tree_ptr_vec_pair bcv_p =
        get_nondominated_disjoint_rec(bcv, bcv_mp, jobs);
    
    // remove the dominated ones from the metapopulation
    boost::sort(bcv_mp);
    boost::sort(bcv_p.second);
    pbscored_combo_tree_ptr_vec diff_bcv_mp =
        set_difference(bcv_mp, bcv_p.second);
    for (const pbscored_combo_tree* cnd : diff_bcv_mp)
        erase(*cnd);

    // add the nondominated ones from bsc
    for (const pbscored_combo_tree* cnd : bcv_p.first)
        insert(new pbscored_combo_tree(*cnd));
}

void metapopulation::trim_down_demes(boost::ptr_vector<deme_t>& demes) const {
    // Trim the demes down to size.  The point here is that the next
    // stage, select_candidates below, is very cpu-intensive; we
    // should keep only those candidates that will survive in the
    // metapop.  But what are these? Well, select_exemplar() uses
    // an exponential choice function; instances below a cut-off
    // score have no chance at all of getting selected. So just
    // eliminate them now, instead of later.
    //
    // However, trimming too much is bad: it can happen that none
    // of the best-scoring instances lead to a solution. So keep
    // around a reasonable pool. Wild choice ot 250 seems reasonable.
    for (deme_t& deme : demes) {

        if (logger().isDebugEnabled())
        {
            stringstream ss;
            ss << "Trim down deme " << deme.getID()
               << " of size: " << deme.size();
            logger().debug(ss.str());
        }

        if (min_pool_size < deme.size()) {
            score_t top_sc = get_penalized_score(deme.begin()->second);
            score_t bot_sc = top_sc - useful_score_range();

            for (size_t i = deme.size()-1; 0 < i; --i) {
                const composite_score &cscore = deme[i].second;
                score_t score = get_penalized_score(cscore);
                if (score < bot_sc) {
                    deme.pop_back();
                }
            }
        }

        if (logger().isDebugEnabled())
        {
            stringstream ss;
            ss << "Deme trimmed down, new size: " << deme.size();
            logger().debug(ss.str());
        }
        
    }
}

/// Update the record of the best score seen, and the associated tree.
/// Safe to call in a multi-threaded context.
void metapopulation::update_best_candidates(const pbscored_combo_tree_set& candidates)
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
    score_t best_sc = get_score(_best_cscore);
    complexity_t best_cpx = get_complexity(_best_cscore);

    for (const pbscored_combo_tree& cnd : candidates)
    {
        const composite_score& csc = get_composite_score(cnd);
        score_t sc = get_score(csc);
        complexity_t cpx = get_complexity(csc);
        if ((sc > best_sc) || ((sc == best_sc) && (cpx <= best_cpx)))
        {
            if ((sc > best_sc) || ((sc == best_sc) && (cpx < best_cpx)))
            {
                _best_cscore = csc;
                best_sc = get_score(_best_cscore);
                best_cpx = get_complexity(_best_cscore);
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
            logger().info() << "" << get_tree(cand);
        }
    }
}

// Like above, but using std::cout.
void metapopulation::print(long n,
           bool output_score,
           bool output_penalty,
           bool output_bscore,
           bool output_visited,
           bool output_only_best)
{
    ostream(std::cout, n, output_score, output_penalty,
            output_bscore, output_visited, output_only_best);
}

bool metapopulation::has_been_visited(const combo_tree& tr) const
{
    return _visited_exemplars.find(tr) != _visited_exemplars.cend();
}

} // ~namespace moses
} // ~namespace opencog

