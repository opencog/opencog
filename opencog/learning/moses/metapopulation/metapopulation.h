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
#include <unordered_map>
#include <unordered_set>

#include <boost/logic/tribool.hpp>

#include "../optimization/optimization.h"
#include "metapop_params.h"
#include "deme_expander.h"

#define EVALUATED_ALL_AVAILABLE 1234567

// Dst cache is temporarily disabled because it creates some
// indeterminism
// #define ENABLE_DST_CACHE

namespace opencog {
namespace moses {

void print_stats_header (optim_stats *os, bool diversity_enabled);

/**
 * The metapopulation will store the expressions (as scored trees)
 * that were encountered during the learning process.  Only the
 * highest-scoring trees are typically kept.
 *
 * The metapopulation is updated in iterations. In each iteration, one
 * of its elements is selected as an exemplar. The exemplar is then
 * decorated with knobs and optimized, to create a new deme.  Suitably
 * high-scoring members of the deme are then folded back into the
 * metapopulation.  At this point, the metapopulation may be pruned,
 * to keep it's size manageable.
 *
 * NOTE:
 *   cscore_base = scoring function (output composite (combined) scores)
 *   bscore_base = behavioral scoring function (output behaviors)
 */
struct metapopulation
{
    // XXX shouldn't this bbe scored_combo_tree ??
    typedef std::unordered_map<combo_tree, unsigned,
                               boost::hash<combo_tree> > combo_tree_hash_counter;

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
        params(pa),
        _dex(type_signature, si_ca, si_kb, sc, opt, pa),
        _bscorer(bsc),
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
        params(pa),
        _dex(type_signature, si, si, sc, opt, pa),
        _bscorer(bsc),
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
        return _best_cscore.get_score();
    }

    /**
     * Return the set of candidates with the highest composite
     * scores.  These will all have the the same "best_composite_score".
     */
    const scored_combo_tree_set& best_candidates() const
    {
        return _best_candidates;
    }

    /**
     * Return the best combo tree (shortest best candidate).
     */
    const combo_tree& best_tree() const
    {
        return _best_candidates.begin()->get_tree();
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
     * the use of scored_combo_tree_set in this class. This would be
     * a fairly big task, and it's currently not clear that its worth
     * the effort, as diversity_penalty is not yet showing promising
     * results...
     */
    void set_diversity();

    void log_selected_exemplar(scored_combo_tree_ptr_set::const_iterator);

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
    scored_combo_tree_ptr_set::const_iterator select_exemplar();
    scored_combo_tree_ptr_set::const_iterator end() const { return _scored_trees.end(); }
    scored_combo_tree_ptr_set::const_iterator begin() const { return _scored_trees.begin(); }
    bool empty() const { return _scored_trees.empty(); } 
    size_t size() const { return _scored_trees.size(); }
    void clear() { _scored_trees.clear(); }

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
    /// scored_combo_tree_ptr_set and not having to copy and
    /// reallocate candidates onces they are selected. It might be
    /// minor though in terms of performance gain.
    void merge_candidates(scored_combo_tree_set& candidates);

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
    bool merge_demes(boost::ptr_vector<deme_t>& demes,
                     const boost::ptr_vector<representation>& reps,
                     const std::vector<unsigned>& evals_seq);

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
    scored_combo_tree_set get_new_candidates(const scored_combo_tree_set&);

    typedef pair<scored_combo_tree_set,
                 scored_combo_tree_set> scored_combo_tree_set_pair;

    typedef std::vector<const scored_combo_tree*> scored_combo_tree_ptr_vec;
    typedef scored_combo_tree_ptr_vec::iterator scored_combo_tree_ptr_vec_it;
    typedef scored_combo_tree_ptr_vec::const_iterator scored_combo_tree_ptr_vec_cit;
    typedef pair<scored_combo_tree_ptr_vec,
                 scored_combo_tree_ptr_vec> scored_combo_tree_ptr_vec_pair;

    // reciprocal of random_access_view
    static scored_combo_tree_set
    to_set(const scored_combo_tree_ptr_vec& bcv);

    void remove_dominated(scored_combo_tree_set& bcs, unsigned jobs = 1);

    static scored_combo_tree_set
    get_nondominated_iter(const scored_combo_tree_set& bcs);

    // split in 2 of equal size
    static scored_combo_tree_ptr_vec_pair
    inline split(const scored_combo_tree_ptr_vec& bcv)
    {
        scored_combo_tree_ptr_vec_cit middle = bcv.begin() + bcv.size() / 2;
        return make_pair(scored_combo_tree_ptr_vec(bcv.begin(), middle),
                         scored_combo_tree_ptr_vec(middle, bcv.end()));
    }

    scored_combo_tree_ptr_vec
    get_nondominated_rec(const scored_combo_tree_ptr_vec& bcv,
                         unsigned jobs = 1);

    // return a pair of sets of nondominated candidates between bcs1
    // and bcs2, assuming none contain dominated candidates. Contrary
    // to what the name of function says, the 2 sets do not need be
    // disjoint, however there are indeed disjoint according to the way
    // they are used in the code. The first (resp. second) element of
    // the pair corresponds to the nondominated candidates of bcs1
    // (resp. bcs2)
    scored_combo_tree_set_pair
    get_nondominated_disjoint(const scored_combo_tree_set& bcs1,
                              const scored_combo_tree_set& bcs2,
                              unsigned jobs = 1);

    scored_combo_tree_ptr_vec_pair
    get_nondominated_disjoint_rec(const scored_combo_tree_ptr_vec& bcv1,
                                  const scored_combo_tree_ptr_vec& bcv2,
                                  unsigned jobs = 1);

    // merge nondominated candidate to the metapopulation assuming
    // that bcs contains no dominated candidates within itself
    void merge_nondominated(const scored_combo_tree_set& bcs, unsigned jobs = 1);

    /**
     * x dominates y if
     *
     * for all i x_i >= y_i and there exists i such that x_i > y_i
     *
     * this function returns true if x dominates y
     *                       false if y dominates x
     *                       indeterminate otherwise
     */
    static inline boost::logic::tribool dominates(const behavioral_score& x,
                                    const behavioral_score& y)
    {
        // everything dominates an empty vector
        if (x.empty()) {
            if (y.empty())
                return boost::logic::indeterminate;
            return false;
        } else if (y.empty()) {
            return true;
        }

        boost::logic::tribool res = boost::logic::indeterminate;
        for (behavioral_score::const_iterator xit = x.begin(), yit = y.begin();
             xit != x.end(); ++xit, ++yit)
        {
            if (*xit > *yit) {
                if (!res)
                    return boost::logic::indeterminate;
                else
                    res = true;
            } else if (*yit > *xit) {
                if (res)
                    return boost::logic::indeterminate;
                else
                    res = false;
            }
        }
        return res;
    }

    // Trim down demes before merging based the scores
    void trim_down_demes(boost::ptr_vector<deme_t>& demes) const;
    
    /// Update the record of the best score seen, and the associated tree.
    /// Safe to call in a multi-threaded context.
    void update_best_candidates(const scored_combo_tree_set& candidates);

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
                ostream_scored_combo_tree(out, *from, output_score,
                                          output_penalty, output_bscore,
                                          output_python);
                if (output_visited)
                    out << "visited: " << has_been_visited(from->get_tree())
                        << std::endl;
            }
            return out;
        }

        // Else, search for the top score...
        score_t best_score = very_worst_score;

        for (In f = from; f != to; ++f) {
            const scored_combo_tree& bt = *f;
            score_t sc = bt.get_score();
            if (best_score < sc) best_score = sc;
        }

        // And print only the top scorers.
        // The problem here is that the highest scorers are not
        // necessarily ranked highest, as the ranking is a linear combo
        // of both score and complexity.
        for (In f = from; f != to && n != 0; ++f, n--) {
            const scored_combo_tree& bt = *f;
            if (best_score <= bt.get_score()) {
                ostream_scored_combo_tree(out, bt, output_score,
                                          output_penalty, output_bscore,
                                          output_python);
                if (output_visited)
                    out << "visited:" << has_been_visited(from->get_tree())
                        << std::endl;
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
        return ostream(out, _scored_trees.begin(), _scored_trees.end(),
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

    // Structure holding stats about diversity
    struct diversity_stats
    {
        double count;    // number of pairs of candidates considered
        double mean;     // average bscore distance between all candidates
        double std;      // std dev bscore distance between all candidates
        double min;      // min bscore distance between all candidates
        double max;      // max bscore distance between all candidates
    };

    /**
     * Gather statistics about the diversity of the n best candidates
     * (if n is negative then all candidates are included)
     */
    diversity_stats gather_diversity_stats(int n);

public:
    const metapop_parameters& params;

// protected:
    deme_expander _dex;
 
protected:
    scored_combo_tree_ptr_set _scored_trees;

    static const unsigned min_pool_size = 250;

    const bscore_base& _bscorer; // behavioral score

    size_t _merge_count;

    // The best score ever found during search.
    composite_score _best_cscore;

    // Trees with composite score equal to _best_cscore.
    scored_combo_tree_set _best_candidates;

    // contains the exemplars of demes that have been searched so far
    // (and the number of times they have been searched)
    combo_tree_hash_counter _visited_exemplars;

    // return true iff tr has already been visited
    bool has_been_visited(const combo_tree& tr) const;

    // lock to enable thread-safe deme merging.
    std::mutex _merge_mutex;

    /**
     * Cache for bscore distance between (for diversity penalty). Maps
     * a std::set<scored_combo_tree*> (only 2 elements to represent
     * an unordered pair) to a a bscore distance. We don't use
     * {lru,prr}_cache because
     *
     * 1) we don't need a limit on the cache.
     *
     * 2) we need to remove the pairs containing deleted pointers
     */
    struct cached_dst
    {
        // ctor
        cached_dst(const diversity_parameters& _dparams)
            : dparams(_dparams), misses(0), hits(0) {}

        // We use a std::set instead of a std::pair, little
        // optimization to deal with the symmetry of the distance
        typedef std::set<const scored_combo_tree*> ptr_pair;
        dp_t operator()(const scored_combo_tree* cl,
                        const scored_combo_tree* cr);

        /**
         * Remove all keys containing any element of ptr_seq
         */
        void erase_ptr_seq(std::vector<scored_combo_tree*> ptr_seq);

        /**
         * Gather some statistics about the diversity of the
         * population, such as mean, std, min, max of the distances.
         */
        diversity_stats gather_stats() const;

        // cache
        boost::shared_mutex mutex;

        const diversity_parameters& dparams;
        std::atomic<unsigned> misses, hits;
        std::unordered_map<ptr_pair, dp_t, boost::hash<ptr_pair>> cache;
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
