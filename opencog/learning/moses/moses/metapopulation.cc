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

diversity_parameters::diversity_parameters(bool _include_dominated)
    : include_dominated(_include_dominated),
      pressure(0.0),
      exponent(-1.0),       // max
      normalize(true)       // sum or mean (default mean)
{
    set_dst(p_norm, 2.0 /* Euclidean */);
    set_dst2dp(inverse);
}

void diversity_parameters::set_dst(diversity_parameters::dst_enum_t de,
                                   diversity_parameters::dp_t p)
{
    typedef behavioral_score bs_t;
    switch(de) {
    case p_norm:
        dst = [p](const behavioral_score& a, const behavioral_score& b) {
            return p_norm_distance(a, b, p);
        };
        break;
    case tanimoto:
        dst = [](const behavioral_score& a, const behavioral_score& b) {
            return tanimoto_distance<behavioral_score, dp_t>(a, b);
        };
        break;
    case angular:
        dst = [](const behavioral_score& a, const behavioral_score& b) {
            return angular_distance<behavioral_score, dp_t>(a, b);
        };
        break;
    default:
        OC_ASSERT(false);
    }    
}

void diversity_parameters::set_dst2dp(diversity_parameters::dst2dp_enum_t d2de)
{
    switch(d2de) {
    case inverse:
        dst2dp = [&](dp_t dst) { return pressure / (1 + dst); };
        break;
    case complement:
        dst2dp = [&](dp_t dst) { return pressure * (1 - dst); };
        break;
    default:
        OC_ASSERT(false);
    }
}
    
bool deme_expander::create_deme(const combo_tree& exemplar)
{
    using namespace reduct;

    OC_ASSERT(_rep == NULL);
    OC_ASSERT(_deme == NULL);

    _exemplar = exemplar;

    if (logger().isDebugEnabled())
        logger().debug() << "Attempt to build rep from exemplar: " << _exemplar;

    // [HIGHLY EXPERIMENTAL]. Limit the number of features used to build
    // the exemplar to a more manageable number, by only working with a
    // a list that is correlated with the output value.  Basically, this is
    // 'on-the-fly' feature selection.  This differs from an ordinary, one-time
    // only, up-front round of feature selection by using only those features
    // which score well with the current exemplar.
    operator_set ignore_ops = _params.ignore_ops;
    if (_params.fstor) {
        // copy, any change in the parameters will not be remembered
        feature_selector festor = *_params.fstor;

        // get the set of features of the exemplar
        auto xmplar_features = get_argument_abs_idx_from_zero_set(_exemplar);

        // Use the features of the exemplar as initial feature set to
        // seed the feature selection algorithm. That way the new
        // features will be selected to combine well with the
        // exemplar.
        if (festor.params.init_exemplar_features) {
            const auto& ilabels = festor._ctable.get_input_labels();
            for (arity_t i : xmplar_features)
                festor.params.fs_params.initial_features.push_back(ilabels[i]);
            // we increase the size to output new features (not the
            // ones already in the exemplar)
            festor.params.increase_target_size = true;
        }

        // If the combo tree is already using N features, we want to find
        // and additional M features which might make it better.  So bump
        // up the count.  Of course, the feat selector might not find any
        // of the existing args in the exemplar; but we want to avoid the
        // case where the feat sel is returning only those features already
        // in the exemplar.
        if (festor.params.increase_target_size) {
            festor.params.fs_params.target_size += xmplar_features.size();
        }

        // Alternatively one can ignore the features in the exemplar
        // during feature selection.
        festor.params.ignore_features = festor.params.ignore_exemplar_features ?
            xmplar_features : set<arity_t>();

        // return the set of selected features as column index
        // (left most column corresponds to 0)
        auto selected_features = festor(_exemplar);
        logger().info() << "Feature selection of " << selected_features.size()
                        << " features for representation";

        // add the complement of the selected features to ignore_ops
        // (but only if they are not present in the exemplar as to not
        // ignore the exemplar features).
        unsigned arity = festor._ctable.get_arity();
        for (unsigned i = 0; i < arity; i++)
            if (selected_features.find(i) == selected_features.end()
                and xmplar_features.find(i) == xmplar_features.end())
                ignore_ops.insert(argument(i + 1));

        // debug print
        // std::vector<std::string> ios;
        // auto vertex_to_str = [](const vertex& v) {
        //     std::stringstream ss;
        //     ss << v;
        //     return ss.str();
        // };
        // boost::transform(ignore_ops, back_inserter(ios), vertex_to_str);
        // printlnContainer(ios);
        // ~debug print
    }

    // Build a representation by adding knobs to the exemplar,
    // creating a field set, and a mapping from field set to knobs.
    _rep = new representation(simplify_candidate,
                              simplify_knob_building,
                              _exemplar, _type_sig,
                              ignore_ops,
                              _params.perceptions,
                              _params.actions,
                              _params.linear_contin,
                              _params.perm_ratio);

    // If the representation is empty, try the next
    // best-scoring exemplar.
    if (_rep->fields().empty()) {
        delete(_rep);
        _rep = NULL;
        logger().warn("The representation is empty, perhaps the reduct "
                      "effort for knob building is too high.");
    }

    if (!_rep) return false;

    // Create an empty deme.
    _deme = new deme_t(_rep->fields());

    return true;
}

int deme_expander::optimize_deme(int max_evals, time_t max_time)
{
    if (logger().isDebugEnabled()) {
        logger().debug()
           << "Optimize deme; max evaluations allowed: "
           << max_evals;
    }

    complexity_based_scorer cpx_scorer =
        complexity_based_scorer(_cscorer, *_rep, _params.reduce_all);
    return _optimize(*_deme, cpx_scorer, max_evals, max_time);
}

void deme_expander::free_deme()
{
    delete _deme;
    delete _rep;
    _deme = NULL;
    _rep = NULL;
}

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

        candidates[si_base] = composite_penalized_bscore(pbs, csc);
    }

    pbscored_combo_tree_set mps(candidates.begin(), candidates.end());
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
            dp_t last_dst = this->_cached_dst(&bsct, last_ptr),
            last_dp = params.diversity.dst2dp(last_dst),
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
            get_composite_score(bsct).set_diversity_penalty(adp);
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
        unsigned pos = std::distance(cbegin(), exemplar_it) + 1;
        logger().debug() << "Selected the " << pos << "th exemplar: "
                         << get_tree(*exemplar_it);
        logger().debug() << "With composite score :"
                         << get_composite_score(*exemplar_it);
    }
}

pbscored_combo_tree_ptr_set::const_iterator metapopulation::select_exemplar()
{
    OC_ASSERT(!empty(), "Empty metapopulation in select_exemplar().");

    logger().debug("Select exemplar");

    // Shortcut for special case, as sometimes, the very first time
    // through, the score is invalid.
    if (size() == 1) {
        const_iterator selex = cbegin();
        const combo_tree& tr = get_tree(*selex);
        if (!has_been_visited(tr))
            _visited_exemplars.insert(tr);
        else selex = cend();
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

        // Skip any exemplars we've already used in the past.
        const combo_tree& tr = get_tree(bsct);
        if (!has_been_visited(tr)) {
            probs.push_back(sc);
            found_exemplar = true;
            if (highest_score < sc) highest_score = sc;
        } else // hack: if the tree is visited then put a positive
               // score so we know it must be ignored
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

    // Mark the exemplar so we won't look at it again.
    _visited_exemplars.insert(get_tree(*selex));

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
        logger().debug("Inserted %u non-dominated candidates in the metapopulation",
                       size() - old_size);
    }
}

bool metapopulation::merge_deme(deme_t* __deme, representation* __rep, size_t evals)
{
    OC_ASSERT(__rep);
    OC_ASSERT(__deme);

    // Note that univariate reports far more evals than the deme size;
    // this is because univariate over-write deme entries.
    logger().debug("Close deme; evaluations reported: %d", evals);

    // Add, as potential exemplars for future demes, all unique
    // trees in the final deme.
    metapop_candidates pot_candidates;

    logger().debug("Sort the deme");

    // Sort the deme according to composite_score (descending order)
    boost::sort(*__deme, std::greater<scored_instance<composite_score> >());

    // Trim the deme down to size.  The point here is that the next
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
    if (min_pool_size < __deme->size()) {
        score_t top_sc = get_penalized_score(__deme->begin()->second);
        score_t bot_sc = top_sc - useful_score_range();

        for (size_t i = __deme->size()-1; 0 < i; --i) {
            const composite_score &cscore = (*__deme)[i].second;
            score_t score = get_penalized_score(cscore);
            if (score < bot_sc) {
                __deme->pop_back();
            }
        }
    }

    ///////////////////////////////////////////////////////////////
    // select the set of candidates to add in the metapopulation //
    ///////////////////////////////////////////////////////////////
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
        score_t inst_sc = get_score(inst_csc);
        // if it's really bad stops
        if (inst_sc <= very_worst_score || !isfinite(inst_sc))
            return;

        // Get the combo_tree associated to inst, cleaned and reduced.
        //
        // @todo: below, the candidate is reduced possibly for the
        // second time.  This second reduction could probably be
        // avoided with some clever cache or something. (or a flag?)
        combo_tree tr = __rep->get_candidate(inst, true);

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

            unique_lock lock(pot_cnd_mutex);
            pot_candidates[tr] = cbsc;
        }
    };

    // It can happen that the true number of evals is less than the
    // deme size (certain cases involving the univariate optimizer)
    // But also, the deme size can be smaller than the number of evals,
    // if the deme was shrunk to save space. 
    unsigned max_pot_cnd = std::min(evals, __deme->size());
    if (params.max_candidates >= 0)
        max_pot_cnd = std::min(max_pot_cnd, (unsigned)params.max_candidates);

    logger().debug("Select candidates to merge (amongst %u)", max_pot_cnd);

    // select_candidates() can be very time consuming; it currently
    // takes anywhere from 25 to 500(!!) millisecs per instance (!!)
    // for me; my (reduced, simplified) instances have complexity
    // of about 100. This seems too long/slow (circa summer 2012).
    deme_cit deme_begin = __deme->begin();
    deme_cit deme_end = deme_begin + max_pot_cnd;
    OMP_ALGO::for_each(deme_begin, deme_end, select_candidates);

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
            composite_score csc = get_composite_score(cand.second);
            penalized_bscore pbs = this->_bscorer(cand.first);
            cand.second = composite_penalized_bscore(pbs, csc);
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
// This makes merging faster because it decreases the number of
// calls of dominates.
pbscored_combo_tree_set metapopulation::get_new_candidates(const metapop_candidates& mcs)
{
    pbscored_combo_tree_set res;
    for (const pbscored_combo_tree& cnd : mcs) {
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

    for (const pbscored_combo_tree& it : candidates)
    {
        const composite_score& cit = get_composite_score(it);
        score_t sc = get_score(cit);
        complexity_t cpx = get_complexity(cit);
        if ((sc > best_sc) || ((sc == best_sc) && (cpx <= best_cpx)))
        {
            if ((sc > best_sc) || ((sc == best_sc) && (cpx < best_cpx)))
            {
                _best_cscore = cit;
                best_sc = get_score(_best_cscore);
                best_cpx = get_complexity(_best_cscore);
                _best_candidates.clear();
                logger().debug() << "New best score: " << _best_cscore;
            }
            _best_candidates.insert(it);
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
        for (const pbscored_combo_tree& cand : best_candidates()) {
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

