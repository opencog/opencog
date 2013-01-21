/** metapopulation.h ---
 *
 * Copyright (C) 2010 Novemente LLC
 * Copyright (C) 2012 Poulin Holdings
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


#ifndef _OPENCOG_METAPOPULATION_H
#define _OPENCOG_METAPOPULATION_H

#include <atomic>
#include <mutex>

#include <boost/unordered_set.hpp>
#include <boost/logic/tribool.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#include <opencog/comboreduct/combo/combo.h>
#include <opencog/comboreduct/reduct/reduct.h>

#include "../representation/instance_set.h"
#include "../representation/representation.h"
#include "../optimization/optimization.h"
#include "feature_selector.h"
#include "scoring.h"
#include "types.h"

#define EVALUATED_ALL_AVAILABLE 1234567

// Dst cache is temporarily disabled because it creates some
// indeterminism
// #define ENABLE_DST_CACHE

namespace opencog {
namespace moses {

using std::pair;
using std::make_pair;
using boost::logic::tribool;
using boost::logic::indeterminate;
using namespace combo;

static const operator_set empty_ignore_ops = operator_set();

struct diversity_parameters
{
    typedef score_t dp_t;

    diversity_parameters(bool _include_dominated = true);

    // Ignore behavioral score domination when merging candidates in
    // the metapopulation. Keeping dominated candidates may improves
    // performance by avoiding local maxima. Discarding dominated
    // candidates may increase the diversity of the metapopulation.
    bool include_dominated;

    // Diversity pressure of to enforce diversification of the
    // metapop. 0 means no diversity pressure, the higher the value
    // the strong the pressure.
    dp_t pressure;

    // exponent of the generalized mean (or sum, see below) used to
    // aggregate the diversity penalties of a candidate between a set
    // of candidates. If the exponent is negative (default) then the
    // max is used instead (generalized mean with infinite exponent).
    dp_t exponent;

    // If normalize is false then the aggregation of diversity
    // penalties is a generalize mean, otherwise it is a generalize
    // sum, defined here as
    //
    // generalized_mean(X) * |X|
    //
    // in other words it is a generalized mean without normalization
    // by the number of elements.
    //
    // If diversity_exponent is negative then this has no effect, it
    // is the max anyway.
    bool normalize;

    // There are 3 distances available to compare bscores, the p-norm,
    // the Tanimoto and the angular distances.
    enum dst_enum_t { p_norm, tanimoto, angular };
    void set_dst(dst_enum_t de, dp_t p = 0.0 /* optional distance parameter */);
    std::function<dp_t(const behavioral_score&, const behavioral_score&)> dst;

    // Function to convert the distance into diversity penalty. There
    // are 2 possible functions
    //
    // The inserve (offset by 1)
    //
    // f(x) = pressure / (1+x)
    //
    // The complement
    //
    // f-x) = pressure * (1-x)
    //
    // The idea is that it should tend to pressure when the distance
    // tends to 0, and tends to 0 when the distance tends to its
    // maximum. Obviously the inverse is adequate when the maximum is
    // infinity and the complement is adequate when it is 1.
    enum dst2dp_enum_t { inverse, complement };
    void set_dst2dp(dst2dp_enum_t d2de);
    std::function<dp_t(dp_t)> dst2dp;
};

/**
 * parameters about deme management
 */
struct metapop_parameters
{
    metapop_parameters(int _max_candidates = -1,
                       bool _reduce_all = true,
                       bool _revisit = false,
                       score_t _complexity_temperature = 3.0f,
                       const operator_set& _ignore_ops = empty_ignore_ops,
                       // bool _enable_cache = false,    // adaptive_cache
                       unsigned _cache_size = 100000,     // is disabled
                       unsigned _jobs = 1,
                       diversity_parameters _diversity = diversity_parameters(),
                       const combo_tree_ns_set* _perceptions = NULL,
                       const combo_tree_ns_set* _actions = NULL,
                       const feature_selector* _fstor = NULL) :
        max_candidates(_max_candidates),
        reduce_all(_reduce_all),
        revisit(_revisit),
        keep_bscore(false),
        complexity_temperature(_complexity_temperature),
        cap_coef(50),
        ignore_ops(_ignore_ops),
        // enable_cache(_enable_cache),   // adaptive_cache
        cache_size(_cache_size),          // is disabled
        jobs(_jobs),
        diversity(_diversity),
        perceptions(_perceptions),
        actions(_actions),
        merge_callback(NULL),
        callback_user_data(NULL),
        fstor(_fstor),
        linear_contin(true)
        {}

    // The max number of candidates considered to be added to the
    // metapopulation, if negative then all candidates are considered.
    int max_candidates;

    // If true then all candidates are reduced before evaluation.
    bool reduce_all;

    // When true then visited exemplars can be revisited.
    bool revisit;

    // keep track of the bscores even if not needed (in case the user
    // wants to keep them around)
    bool keep_bscore;

    // Boltzmann temperature ...
    score_t complexity_temperature;

    // The metapopulation size is capped according to the following
    // formula:
    //
    // cap = cap_coef*(x+250)*(1+2*exp(-x/500))
    //
    // where x is the number of generations so far
    double cap_coef;

    // the set of operators to ignore
    operator_set ignore_ops;

    // Enable caching of scores.
    // bool enable_cache;   // adaptive_cache
    unsigned cache_size;    // is disabled

    // Number of jobs for metapopulation maintenance such as merging
    // candidates to the metapopulation.
    unsigned jobs;

    // parameters to control diversity
    diversity_parameters diversity;

    // the set of perceptions of an optional interactive agent
    const combo_tree_ns_set* perceptions;
    // the set of actions of an optional interactive agent
    const combo_tree_ns_set* actions;

    bool (*merge_callback)(bscored_combo_tree_set&, void*);
    void *callback_user_data;

    const feature_selector* fstor;

    // Build only linear expressions involving contin features.
    // This can greatly decrease the number of knobs created during
    // representation building, resulting in much smaller field sets,
    // and instances that can be searched more quickly. However, in
    // order to fit the data, linear expressions may not be as good,
    // and thus may require more time overall to find...
    bool linear_contin;

    // Defines how many pairs of literals constituting subtrees op(l1
    // l2) are considered while creating the prototype of an
    // exemplar. It ranges from 0 to 1, 0 means arity positive
    // literals and arity pairs of literals, 1 means arity positive
    // literals and arity*(arity-1) pairs of literals
    float perm_ratio;
};

void print_stats_header (optim_stats *os, bool diversity_enabled);

struct deme_expander
{
    typedef deme_t::iterator deme_it;
    typedef deme_t::const_iterator deme_cit;

    deme_expander(const type_tree& type_signature,
                  const reduct::rule& si_ca,
                  const reduct::rule& si_kb,
                  const cscore_base& sc,
                  optimizer_base& opt,
                  const metapop_parameters& pa = metapop_parameters()) :
        _rep(NULL), _deme(NULL),
        _optimize(opt),
        _type_sig(type_signature),
        simplify_candidate(si_ca),
        simplify_knob_building(si_kb),
        _cscorer(sc),
        _params(pa)
    {}

    ~deme_expander()
    {
        if (_rep) delete _rep;
        if (_deme) delete _deme;
    }

    /**
     * Create the deme
     *
     * @return return true if it creates deme successfully,otherwise false.
     */
    // bool create_deme(bscored_combo_tree_set::const_iterator exemplar)
    bool create_deme(const combo_tree& exemplar);

    /**
     * Do some optimization according to the scoring function.
     *
     * @param max_evals the maximum number of evaluations of the scoring
     *                  function to perform.
     * @param max_time the maximum elapsed (wall-clock) time to allow.
     *
     * @return return the number of evaluations actually performed,
     */
    int optimize_deme(int max_evals, time_t max_time);

    void free_deme();

    // Structures related to the current deme
    representation* _rep; // representation of the current deme
    deme_t* _deme; // current deme
    optimizer_base &_optimize;

protected:
    const combo::type_tree& _type_sig;    // type signature of the exemplar
    const reduct::rule& simplify_candidate; // to simplify candidates
    const reduct::rule& simplify_knob_building; // during knob building
    const cscore_base& _cscorer; // composite score
    metapop_parameters _params;

    // exemplar of the current deme; a copy, not a reference.
    combo_tree _exemplar;
};

/**
 * The metapopulation will store the expressions (as scored trees)
 * that were encountered during the learning process.  Only the
 * highest-scoring trees are typically kept.
 *
 * The metapopulation is updated in iterations. In each iteration, one
 * of its elements is selected as an exemplar. The exemplar is then
 * decorated with knobs and optimized, to create a new deme.  Members
 * of the deme are then folded back into the metapopulation.
 *
 * NOTE:
 *   cscore_base = scoring function (output composite scores)
 *   bscore_base = behavioral scoring function (output behaviors)
 */
struct metapopulation : bscored_combo_tree_ptr_set
{
    typedef deme_t::iterator deme_it;
    typedef deme_t::const_iterator deme_cit;

    // The goal of using unordered_set here is to have O(1) access time
    // to see if a combo tree is in the set, or not.
    typedef boost::unordered_set<combo_tree,
                                 boost::hash<combo_tree> > combo_tree_hash_set;

    // Init the metapopulation with the following set of exemplars.
    void init(const std::vector<combo_tree>& exemplars,
              const reduct::rule& simplify_candidate,
              const cscore_base& cscorer);

    /**
     *  Constuctor for the class metapopulation
     *
     * @param bases   Exemplars used to initialize the metapopulation
     * @param tt      Type signature of expression to be learned.
     *                That is, the expression must have this signature,
     *                relating the argument variables, and the output type.
     * @param si_ca   Reduct rule for reducing candidate combo trees.
     * @param si_kb   Reduct rule for reducing trees decorated with
     *                knobs.
     * @param sc      Function for scoring combo trees.
     * @param bsc     Behavior scoring function
     * @param opt     Algorithm that find best knob settings for a given
     *                exemplar decorated with knobs.
     * @param pa      Control parameters for this class.
     */
    metapopulation(const std::vector<combo_tree>& bases,
                   const type_tree& type_signature,
                   const reduct::rule& si_ca,
                   const reduct::rule& si_kb,
                   const cscore_base& sc,
                   const bscore_base& bsc,
                   optimizer_base& opt,
                   const metapop_parameters& pa = metapop_parameters()) :
        _dex(type_signature, si_ca, si_kb, sc, opt, pa),
        _bscorer(bsc), params(pa),
        _merge_count(0),
        _best_cscore(worst_composite_score),
        _cached_dst(pa.diversity)
    {
        init(bases, si_ca, sc);
    }

    // Like above but using a single base, and a single reduction rule.
    /// @todo use C++11 redirection
    metapopulation(const combo_tree& base,
                   const type_tree& type_signature,
                   const reduct::rule& si,
                   const cscore_base& sc, const bscore_base& bsc,
                   optimizer_base& opt,
                   const metapop_parameters& pa = metapop_parameters()) :
        _dex(type_signature, si, si, sc, opt, pa),
        _bscorer(bsc), params(pa),
        _merge_count(0),
        _best_cscore(worst_composite_score),
        _cached_dst(pa.diversity)
    {
        std::vector<combo_tree> bases(1, base);
        init(bases, si, sc);
    }

    ~metapopulation() {}

    /**
     * Return the best composite score.
     */
    const composite_score& best_composite_score() const
    {
        return _best_cscore;
    }

    /**
     * Return the best score.
     */
    score_t best_score() const
    {
        return get_score(_best_cscore);
    }

    /**
     * Return the set of candidates with the highest composite
     * scores.  These will all have the the same "best_composite_score".
     */
    const metapop_candidates& best_candidates() const
    {
        return _best_candidates;
    }

    /**
     * Return the best combo tree (shortest best candidate).
     */
    const combo_tree& best_tree() const
    {
        return _best_candidates.begin()->first;
    }

    typedef diversity_parameters::dp_t dp_t;  // diversity_penalty type

    /**
     * Distort a diversity penalty component between 2
     * candidates. (actually not used apart from a comment of
     * aggregated_dps)
     */
    dp_t distort_dp(dp_t dp) const {
        return pow(dp, params.diversity.exponent);
    }
    /**
     * The inverse function of distort_dp normalized by the vector
     * size. Basically
     *
     * aggregated_dps(sum_i distort_dp(x_i), N) == generalized_mean(x)
     * where N is the size of x.
     */
    dp_t aggregated_dps(dp_t ddp_sum, unsigned N) const {
        return pow(ddp_sum / N, 1.0 / params.diversity.exponent);
    }

    /**
     * Compute the diversity penalty for all models of the metapopulation.
     *
     * If the diversity penalty is enabled, then punish the scores of
     * those exemplars that are too similar to the previous ones.
     * This may not make any difference for the first dozen exemplars
     * choosen, but starts getting important once the metapopulation
     * gets large, and the search bogs down.
     *
     * XXX The implementation here results in a lot of copying of
     * behavioral scores and combo trees, and thus could hurt
     * performance by quite a bit.  To avoid this, we'd need to change
     * the use of bscored_combo_tree_set in this class. This would be
     * a fairly big task, and it's currently not clear that its worth
     * the effort, as diversity_penalty is not yet showing promising
     * results...
     */
    void set_diversity();

    void log_selected_exemplar(const_iterator exemplar_it);

    /**
     * Select the exemplar from the population. An exemplar is choosen
     * from the pool of candidates using a Boltzmann distribution
     * exp (-score / temperature).  Thus, they choosen exemplar will
     * typically be high-scoring, but not necessarily the highest-scoring.
     * This allows a range of reasonably-competitive candidates to be
     * explored, and, in practice, prooves to be much more effective
     * than a greedy algo which only selects the highest-scoring candidate.
     *
     * Current experimental evidence shows that temperatures in the
     * range of 6-12 work best for most problems, both discrete
     * (e.g. 4-parity) and continuous.
     *
     * @return the iterator of the selected exemplar, if no such
     *         exemplar exists then return end()
     */
    bscored_combo_tree_ptr_set::const_iterator select_exemplar();

    /// Given the current complexity temp, return the range of scores that
    /// are likely to be selected by the select_exemplar routine. Due to
    /// exponential decay of scores in select_exemplar(), this is fairly
    /// narrow: viz: e^30 = 1e13 ... We could probably get by with
    /// e^14 = 1.2e6
    //
    score_t useful_score_range() const
    {
        return params.complexity_temperature * 30.0 / 100.0;
    }

    /// Merge candidates in to the metapopulation.
    ///
    /// If the include-dominated flag is not set, the set of candidates
    /// might be changed during merge, with the dominated candidates
    /// removed during the merge. XXX Really?  It looks like the code
    /// does this culling *before* this method is called ...
    ///
    /// Safe to call in a multi-threaded context.
    ///
    /// @todo it would probably be more efficient to use
    /// bscored_combo_tree_ptr_set and not having to copy and
    /// reallocate candidates onces they are selected. It might be
    /// minor though in terms of performance gain.
    void merge_candidates(bscored_combo_tree_set& candidates);

    /**
     * merge deme -- convert instances to trees, and save them.
     *
     * 1) cull the poorest scoring instances.
     * 2) convert set of instances to trees
     * 3) merge trees into the metapopulation, possibly using domination
     *    as the merge criterion.
     *
     * Return true if further deme exploration should be halted.
     *
     * This is almost but not quite thread-safe.  The use of
     * _visited_exemplars is not yet protected. There may be other
     * things.
     */
    bool merge_deme(deme_t* __deme, representation* __rep, size_t evals);

    /**
     * Weed out excessively bad scores. The select_exemplar() routine
     * picks an exemplar out of the metapopulation, using an
     * exponential distribution of the score. Scores that are much
     * worse than the best scores are extremely unlikely to be
     * choosen, so discard these from the metapopulation.  Keeping the
     * metapop small brings huge benefits to the mem usage and runtime
     * performance.
     *
     * However, lets not get over-zelous; if the metapop is too small,
     * then we have the nasty situation where none of the best-scoring
     * individuals lead to a solution.  Fix the minimum metapop size
     * to, oh, say, 250.
     *
     * But if the population starts exploding, this is also bad, as it
     * chews up RAM with unlikely exemplars. Keep it in check by
     * applying more and more stringent bounds on the allowable
     * scores.  The current implementation of useful_score_range()
     * returns a value a bit on the large size, by a factor of 2 or
     * so, so its quite OK to cut back on this value.
     */
    void resize_metapop();

    // Return the set of candidates not present in the metapopulation.
    // This makes merging faster because it decreases the number of
    // calls of dominates.
    bscored_combo_tree_set get_new_candidates(const metapop_candidates& mcs);

    typedef pair<bscored_combo_tree_set,
                 bscored_combo_tree_set> bscored_combo_tree_set_pair;

    typedef std::vector<const bscored_combo_tree*> bscored_combo_tree_ptr_vec;
    typedef bscored_combo_tree_ptr_vec::iterator bscored_combo_tree_ptr_vec_it;
    typedef bscored_combo_tree_ptr_vec::const_iterator bscored_combo_tree_ptr_vec_cit;
    typedef pair<bscored_combo_tree_ptr_vec,
                 bscored_combo_tree_ptr_vec> bscored_combo_tree_ptr_vec_pair;

    // reciprocal of random_access_view
    static bscored_combo_tree_set
    to_set(const bscored_combo_tree_ptr_vec& bcv);

    void remove_dominated(bscored_combo_tree_set& bcs, unsigned jobs = 1);

    static bscored_combo_tree_set
    get_nondominated_iter(const bscored_combo_tree_set& bcs);

    // split in 2 of equal size
    static bscored_combo_tree_ptr_vec_pair
    inline split(const bscored_combo_tree_ptr_vec& bcv)
    {
        bscored_combo_tree_ptr_vec_cit middle = bcv.begin() + bcv.size() / 2;
        return make_pair(bscored_combo_tree_ptr_vec(bcv.begin(), middle),
                         bscored_combo_tree_ptr_vec(middle, bcv.end()));
    }

    bscored_combo_tree_ptr_vec
    get_nondominated_rec(const bscored_combo_tree_ptr_vec& bcv,
                         unsigned jobs = 1);

    // return a pair of sets of nondominated candidates between bcs1
    // and bcs2, assuming none contain dominated candidates. Contrary
    // to what the name of function says, the 2 sets do not need be
    // disjoint, however there are indeed disjoint according to the way
    // they are used in the code. The first (resp. second) element of
    // the pair corresponds to the nondominated candidates of bcs1
    // (resp. bcs2)
    bscored_combo_tree_set_pair
    get_nondominated_disjoint(const bscored_combo_tree_set& bcs1,
                              const bscored_combo_tree_set& bcs2,
                              unsigned jobs = 1);

    bscored_combo_tree_ptr_vec_pair
    get_nondominated_disjoint_rec(const bscored_combo_tree_ptr_vec& bcv1,
                                  const bscored_combo_tree_ptr_vec& bcv2,
                                  unsigned jobs = 1);

    // merge nondominated candidate to the metapopulation assuming
    // that bcs contains no dominated candidates within itself
    void merge_nondominated(bscored_combo_tree_set& bcs, unsigned jobs = 1);

    // Iterative version of merge_nondominated
    void merge_nondominated_iter(bscored_combo_tree_set& bcs);

    /**
     * x dominates y if
     *
     * for all i x_i >= y_i and there exists i such that x_i > y_i
     *
     * this function returns true if x dominates y
     *                       false if y dominates x
     *                       indeterminate otherwise
     */
    static inline tribool dominates(const behavioral_score& x,
                                    const behavioral_score& y)
    {
        // everything dominates an empty vector
        if (x.empty()) {
            if (y.empty())
                return indeterminate;
            return false;
        } else if (y.empty()) {
            return true;
        }

        tribool res = indeterminate;
        for (behavioral_score::const_iterator xit = x.begin(), yit = y.begin();
             xit != x.end(); ++xit, ++yit)
        {
            if (*xit > *yit) {
                if (!res)
                    return indeterminate;
                else
                    res = true;
            } else if (*yit > *xit) {
                if (res)
                    return indeterminate;
                else
                    res = false;
            }
        }
        return res;
    }

    /// Update the record of the best score seen, and the associated tree.
    /// Safe to call in a multi-threaded context.
    void update_best_candidates(const bscored_combo_tree_set& candidates);

    // log the best candidates
    void log_best_candidates() const;

    /**
     * stream out the metapopulation in decreasing order of their
     * score along with their scores (optionally complexity and
     * bscore).  If n is negative, then stream them all out.  Note
     * that the default sort order for the metapop is a penalized
     * scores and the smallest complexities, so that the best-ranked
     * candidates are not necessarily those with the best raw score.
     */
    template<typename Out, typename In>
    Out& ostream(Out& out, In from, In to, long n = -1,
                 bool output_score = true,
                 bool output_penalty = false,
                 bool output_bscore = false,
                 bool output_visited = false,
                 bool output_only_best = false,
                 bool output_python = false)
    {
        if (!output_only_best) {
            for (; from != to && n != 0; ++from, n--) {
                ostream_bscored_combo_tree(out, *from, output_score,
                                           output_penalty, output_bscore,
                                           output_python);
                if (output_visited)
                    out << "visited: " << has_been_visited(*from) << std::endl;
            }
            return out;
        }

        // Else, search for the top score...
        score_t best_score = very_worst_score;

        for (In f = from; f != to; ++f) {
            const bscored_combo_tree& bt = *f;
            score_t sc = get_score(bt);
            if (best_score < sc) best_score = sc;
        }

        // And print only the top scorers.
        // The problem here is that the highest scorers are not
        // necessarily ranked highest, as the ranking is a linear combo
        // of both score and complexity.
        for (In f = from; f != to && n != 0; ++f, n--) {
            const bscored_combo_tree& bt = *f;
            if (best_score <= get_score(bt)) {
                ostream_bscored_combo_tree(out, bt, output_score,
                                           output_penalty, output_bscore,
                                           output_python);
                if (output_visited)
                    out << "visited:" << has_been_visited(*from) << std::endl;
            }
        }
        return out;
    }

    // Like above, but assumes that from = begin() and to = end().
    template<typename Out>
    Out& ostream(Out& out, long n = -1,
                 bool output_score = true,
                 bool output_penalty = false,
                 bool output_bscore = false,
                 bool output_visited = false,
                 bool output_only_best = false,
                 bool output_python = false)
    {
        return ostream(out, begin(), end(),
                       n, output_score, output_penalty, output_bscore,
                       output_visited, output_only_best, output_python);
    }

    // Like above, but using std::cout.
    void print(long n = -1,
               bool output_score = true,
               bool output_penalty = false,
               bool output_bscore = false,
               bool output_visited = false,
               bool output_only_best = false);

    deme_expander _dex;

    // Structure holding stats about diversity
    struct diversity_stats {
        float count,            // number of pairs of candidates considered
            mean,               // average bscore distance between all candidates
            std,                // std dev bscore distance between all candidates
            min,                // min bscore distance between all candidates
            max;                // max bscore distance between all candidates
    };

    /**
     * Gather statistics about the diversity of the n best candidates
     * (if n is negative then all candidates are included)
     */
    diversity_stats gather_diversity_stats(int n) {
        using namespace boost::accumulators;
        typedef accumulator_set<float, stats<tag::count,
                                     tag::mean,
                                     tag::variance,
                                     tag::min,
                                     tag::max>> accumulator_t;
        if (n < 0)
            return _cached_dst.gather_stats();
        else {
            using namespace boost::accumulators;
            typedef accumulator_set<float, stats<tag::count,
                                     tag::mean,
                                     tag::variance,
                                     tag::min,
                                     tag::max>> accumulator_t;

            // compute the statistics
            accumulator_t acc;
            auto from_i = cbegin(),
                to = std::next(cbegin(), std::min(n, (int)size()));
            for (; from_i != to; ++from_i) {
                for (auto from_j = cbegin(); from_j != from_i; ++from_j) {
#ifdef ENABLE_DST_CACHE
                    cached_dst::ptr_pair cts = {&*from_j, &*from_i};
                    auto it = _cached_dst.cache.find(cts);
                    OC_ASSERT(it != _cached_dst.cache.cend(),
                              "Candidate isn't in the cache that must be a bug");
                    acc(it->second);
#else
                    acc(_cached_dst(&*from_j, &*from_i));
#endif
                }
            }

            // gather stats
            diversity_stats ds;
            ds.count = boost::accumulators::count(acc);
            ds.mean = boost::accumulators::mean(acc);
            ds.std = sqrt(boost::accumulators::variance(acc));
            ds.min = boost::accumulators::min(acc);
            ds.max = boost::accumulators::max(acc);

            return ds;
        }
    }

protected:
    static const unsigned min_pool_size = 250;

    const bscore_base& _bscorer; // behavioral score

public:
    const metapop_parameters& params;

protected:
    size_t _merge_count;

    // The best score ever found during search.
    composite_score _best_cscore;

    // Trees with composite score equal to _best_cscore.
    metapop_candidates _best_candidates;

    // contains the exemplars of demes that have been searched so far
    combo_tree_hash_set _visited_exemplars;

    // return true iff tr has already been visited
    bool has_been_visited(const combo_tree& tr) const;

    // lock to enable thread-safe deme merging.
    std::mutex _merge_mutex;

    /**
     * Cache for bscore distance between (for diversity penalty). Maps
     * a std::set<bscored_combo_tree*> (only 2 elements to represent
     * an unordered pair) to a a bscore distance. We don't use
     * {lru,prr}_cache because
     *
     * 1) we don't need a limit on the cache.
     *
     * 2) we need to remove the pairs containing deleted pointers
     */
    struct cached_dst {
        // ctor
        cached_dst(const diversity_parameters& _dparams)
            : dparams(_dparams), misses(0), hits(0) {}

        // We use a std::set instead of a std::pair, little
        // optimization to deal with the symmetry of the distance
        typedef std::set<const bscored_combo_tree*> ptr_pair;
        dp_t operator()(const bscored_combo_tree* cl, const bscored_combo_tree* cr)
        {
#ifdef ENABLE_DST_CACHE
            ptr_pair cts = {cl, cr};
            // hit
            {
                shared_lock lock(mutex);
                auto it = cache.find(cts);
                if (it != cache.end()) {
                    ++hits;
                    return it->second;
                }
            }
            // miss
            dp_t dst = dparams.dst(get_bscore(*cl), get_bscore(*cr));

            // // debug
            // logger().fine("&cl = %p, &cr = %p, dst = %f", cl, cr, dst);
            // // ~debug

            ++misses;
            {
                unique_lock lock(mutex);
                return cache[cts] = dst;
            }
#else
            return dparams.dst(get_bscore(*cl), get_bscore(*cr));
#endif
        }

        /**
         * Remove all keys containing any element of ptr_seq
         */
        void erase_ptr_seq(std::vector<bscored_combo_tree*> ptr_seq) {
#ifdef ENABLE_DST_CACHE
            for (Cache::iterator it = cache.begin(); it != cache.end();) {
                if (!is_disjoint(ptr_seq, it->first))
                    it = cache.erase(it);
                else
                    ++it;
            }
#endif
        }

        /**
         * Gather some statistics about the diversity of the
         * population, such as mean, std, min, max of the distances.
         */
        diversity_stats gather_stats() const {
            using namespace boost::accumulators;
            typedef accumulator_set<float, stats<tag::count,
                                     tag::mean,
                                     tag::variance,
                                     tag::min,
                                     tag::max>> accumulator_t;

            // compute the statistics
            accumulator_t acc;
            for (const auto& v : cache) acc(v.second);

            // gather stats
            diversity_stats ds;
            ds.count = boost::accumulators::count(acc);
            ds.mean = boost::accumulators::mean(acc);
            ds.std = sqrt(boost::accumulators::variance(acc));
            ds.min = boost::accumulators::min(acc);
            ds.max = boost::accumulators::max(acc);

            return ds;
        }

        // cache
        typedef boost::shared_mutex cache_mutex;
        typedef boost::shared_lock<cache_mutex> shared_lock;
        typedef boost::unique_lock<cache_mutex> unique_lock;
        typedef boost::unordered_map<ptr_pair, dp_t, boost::hash<ptr_pair>> Cache;
        cache_mutex mutex;

        const diversity_parameters& dparams;
        std::atomic<unsigned> misses, hits;
        Cache cache;
    };

    cached_dst _cached_dst;

public:
    const cached_dst& get_cached_dst() const {
        return _cached_dst;
    }

};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_METAPOPULATION_H
