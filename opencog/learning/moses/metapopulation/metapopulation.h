/** metapopulation.h ---
 *
 * Copyright (C) 2010 Novemente LLC
 * Copyright (C) 2012 Poulin Holdings
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


#ifndef _OPENCOG_METAPOPULATION_H
#define _OPENCOG_METAPOPULATION_H

#include <atomic>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/logic/tribool.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <opencog/util/boost_ext/accumulators/statistics/geometric_mean_mirror.h>

#include <opencog/comboreduct/combo/combo.h>
#include "../optimization/optimization.h"
#include "../scoring/behave_cscore.h"
#include "metapop_params.h"
#include "ensemble.h"

#define EVALUATED_ALL_AVAILABLE 1234567

// Dst cache is temporarily disabled because it creates some
// indeterminism
// #define ENABLE_DST_CACHE

class metapopulationUTest;

namespace opencog {
namespace moses {

/**
 * The metapopulation stores a pool of combo trees that are drawn from
 * during the learning process.  The pool is expanded by adding the
 * expressions (the combo trees) that were generated during the deme
 * expansion and optimization step.
 *
 * The metapopulation is updated in iterations. In each iteration, one
 * of its elements is selected as an exemplar. The exemplar is then
 * decorated with knobs and optimized, to create a new deme.  Suitably
 * high-scoring members of the deme (instances) are then explicitly
 * converted into trees, and folded back into the metapopulation.  At
 * this point, the metapopulation may be pruned, to keep it's size
 * manageable.
 *
 * Again, the primary purpose of the metapopulation is to provide a
 * well-balanced, evenly-distributed collection of exemplars for future
 * expansion into demes.  The metapopulation does NOT have to consist
 * of the very best possible combo trees; rather, it must consist of
 * the kinds of combo trees that will generate the fittest offspring,
 * when expanded. It can be thought of as a kind of "breeding stock".
 *
 * The metapopulation should be contrasted with the ensemble, which
 * also holds a colection of scored combo trees.  The explicit goal of
 * the ensemble is to gather together the fittest combo trees (the most
 * predictive, accurate, precise trees). Although, in general, the two
 * are likely to hold similar sets of trees, the management policy for
 * the ensemble and the metapopulation differ; the ensemble will be used
 * for inference, the metapopulation for breeding.
 *
 * XXX FIXME: right now, the ensemble is attached to the metapop, its
 * kind-of coming along for the ride, because that's easier for now.
 * Someday, it should have an independent existance.
 *
 * A number of different approaches are taken to maintain a well-balanced,
 * evenly-distributed collection of exemplars.  One of these is the
 * "diversity" mechanism, which seeks to ensure that the distribution
 * stays as diverse as possible, by means of a "diversity penalty",
 * which enables low-scoring combo trees to be kept around, without being
 * crowded out by higher-scoring rivals. The point here is that it is
 * often the case that low-scoring, unfit combo trees can generate very
 * high-fitness offspring.
 */
using combo::combo_tree;

class metapopulation
{
    // Init the metapopulation with the following set of exemplars.
    void init(const std::vector<combo_tree>& exemplars);

public:
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
                   behave_cscore& sc,
                   const metapop_parameters& pa = metapop_parameters(),
                   const subsample_deme_filter_parameters& subp = subsample_deme_filter_parameters());

    // Like above but using a single base, and a single reduction rule.
    /// @todo use C++11 redirection
    metapopulation(const combo_tree& base,
                   behave_cscore& sc,
                   const metapop_parameters& pa = metapop_parameters(),
                   const subsample_deme_filter_parameters& subp = subsample_deme_filter_parameters());

    ~metapopulation() {}

    const scored_combo_tree_set& best_candidates() const;
    const ensemble& get_ensemble() const { return _ensemble; }
    composite_score best_composite_score() const;
    const combo_tree& best_tree() const;

    /**
     * Return the best model score (either the score of the
     * highest-scoring tree in the metapop, or the ensemble score).
     */
    score_t best_score() const {
        return best_composite_score().get_score();
    }

    behave_cscore& get_cscorer() const { return  _cscorer; }

    // ---------------- Deme-expansion-related -----------------------
public:
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

    const scored_combo_tree_ptr_set& get_trees() const { return _scored_trees; }
    scored_combo_tree_ptr_set::const_iterator begin() const { return _scored_trees.begin(); }
    scored_combo_tree_ptr_set::const_iterator end() const { return _scored_trees.end(); }
    bool empty() const { return _scored_trees.empty(); }
    size_t size() const { return _scored_trees.size(); }
    void clear() { _scored_trees.clear(); }

    // -------------------------- Merge related ----------------------
public:
    /// Merge candidates in to the metapopulation. If the param
    /// discard_dominated flag is not set, then no culling or scoring
    /// is performed; it is assumed that previous stages have already
    /// determined that the candidates are suitable for merging.
    ///
    /// Safe to call in a multi-threaded context.
    ///
    /// @todo it would probably be more efficient to use
    /// scored_combo_tree_ptr_set and not having to copy and
    /// reallocate candidates once they are selected. It might be
    /// minor though in terms of performance gain. FIXME.
    void merge_candidates(scored_combo_tree_set& candidates);

    /**
     * merge demes -- convert instances to trees, and merge them
     * back into the metapopulation.
     *
     * 1) cull the poorest scoring instances.
     * 2) convert instances to trees
     * 3) merge trees into the metapopulation, possibly using domination
     *    as the merge criterion.
     *
     * Return true if further deme exploration should be halted.
     *
     * 'demes' is a list of demes
     * 'reps' is a list of reps, each rep corresponding to a deme.
     * 'evals' is a list of counts, indicating how many scoring
     *         function evaluations were made for each deme/rep.
     *
     * Although this is now thread safe, it itself uses many threads
     * to get stuff done, and so prallelizing calls to this don't
     * obviously make sense.
     */
    bool merge_demes(std::vector<std::vector<deme_t>>& demes,
                     const boost::ptr_vector<representation>& reps);

    /// Update the record of the best score seen, and the associated tree.
    /// Safe to call in a multi-threaded context.
    void update_best_candidates(const scored_combo_tree_set& candidates);

private:
    /**
     * Recompute the composite scores for the entire metapopulation.
     * This rouine is used only during boosting. When boosting is
     * enabled, the relative weights of each item in the bscore will be
     * dynamically changing; thus, the composite score associated with
     * a given combo tree needs to change as well.  This method loops
     * over the metapop, and recomputes the the composite score from
     * each bscore.
     */
    void rescore();

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
     * to, oh, say, 250 (aka _min_pool_size).
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

    // Trim down deme before merging based the scores
    void trim_down_deme(deme_t& deme) const;

    // convert instances in deme to trees
    void deme_to_trees(deme_t&, const representation&,
                       scored_combo_tree_set&);

    /// Given the current complexity temp, return the range of scores that
    /// are likely to be selected by the select_exemplar routine. Due to
    /// exponential decay of scores in select_exemplar(), this is fairly
    /// narrow: viz: e^30 = 1e13 ... We could probably get by with
    /// e^14 = 1.2e6
    //
    score_t useful_score_range() const
    {
        return _params.complexity_temperature * 30.0 / 100.0;
    }

    // ------------------- Diversity-realted parts --------------------
private:
    typedef diversity_parameters::dp_t dp_t;  // diversity_penalty type

    /**
     * Distort a diversity penalty component between 2
     * candidates. (actually not used apart from a comment of
     * aggregated_dps)
     */
    dp_t distort_dp(dp_t dp) const {
        return pow(dp, _params.diversity.exponent);
    }
    /**
     * The inverse function of distort_dp normalized by the vector
     * size. Basically
     *
     * aggregated_dps(sum_i distort_dp(x_i), N) == generalized_mean(x)
     * where N is the size of x.
     */
    dp_t aggregated_dps(dp_t ddp_sum, unsigned N) const {
        return pow(ddp_sum / N, 1.0 / _params.diversity.exponent);
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

public:
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

    /**
     * Return true if the diversity mechanism is enabled. This mechanism
     * tries to make sure that the metapopulation is as diverse as posible,
     * thus, in principle, speeding learning.
     */
    bool diversity_enabled() const {
        return _params.diversity.pressure > 0.0;
    }
private:

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
        cached_dst(const diversity_parameters& dparams)
            : _dparams(dparams), misses(0), hits(0) {}

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

        const diversity_parameters& _dparams;
        std::atomic<unsigned> misses, hits;
        std::unordered_map<ptr_pair, dp_t, boost::hash<ptr_pair>> cache;
    };

    cached_dst _cached_dst;

public:
    const cached_dst& get_cached_dst() const {
        return _cached_dst;
    }

    // --------------------- Domination-related stuff --------------------
private:
    friend class ::metapopulationUTest;  // the tester tests the domination code...

    typedef std::pair<scored_combo_tree_set,
                      scored_combo_tree_set> scored_combo_tree_set_pair;
    typedef std::vector<const scored_combo_tree*> scored_combo_tree_ptr_vec;
    typedef scored_combo_tree_ptr_vec::iterator scored_combo_tree_ptr_vec_it;
    typedef scored_combo_tree_ptr_vec::const_iterator scored_combo_tree_ptr_vec_cit;
    typedef std::pair<scored_combo_tree_ptr_vec,
                      scored_combo_tree_ptr_vec> scored_combo_tree_ptr_vec_pair;

    // reciprocal of random_access_view
    static scored_combo_tree_set to_set(const scored_combo_tree_ptr_vec& bcv);

    void remove_dominated(scored_combo_tree_set& bcs, unsigned jobs = 1);

    // split in 2 of equal size
    static scored_combo_tree_ptr_vec_pair
    inline split(const scored_combo_tree_ptr_vec& bcv)
    {
        scored_combo_tree_ptr_vec_cit middle = bcv.begin() + bcv.size() / 2;
        return make_pair(scored_combo_tree_ptr_vec(bcv.begin(), middle),
                         scored_combo_tree_ptr_vec(middle, bcv.end()));
    }

    static scored_combo_tree_set
    get_nondominated_iter(const scored_combo_tree_set& bcs);

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

    static boost::logic::tribool dominates(const behavioral_score& x,
                                           const behavioral_score& y);

private:
    // Sort all demes by decreasing penalized score
    void sort_demes(std::vector<std::vector<deme_t>>& ss_demes);

    // Remove duplicate candidates across all demes. It assumes that
    // the demes are sorted, that way it only needs to compare
    // candidates with identical scores (which saves a lot of
    // computation)
    void keep_top_unique_candidates(
        std::vector<std::vector<deme_t>>& all_demes,
        const boost::ptr_vector<representation>& reps);

    // ------------------ Subsampling-related -----------------------

    // Subsample filter. Return true if it passes the subsample
    // filter. I.e. if the variance of the score across demes pass the
    // filter. Note that ss_filter is not a const method because it
    // modifies the SS-demes scores (the candidates re-evaluated
    // will be on the whole dataset).
    bool ss_score_dev_filter(const representation& rep,
                             const std::vector<deme_t>& ss_demes) const;

    // Given the top candidates of each deme, what is the average
    // agreement between those candidates, for each data point, across
    // all ss-demes.
    float ss_average_agreement(const representation& rep,
                               std::vector<deme_t>& ss_demes);

    // Accumulator used to collect stats for the subsampling tanimoto
    // filter
    typedef boost::accumulators::accumulator_set
    <double,
     boost::accumulators::stats<boost::accumulators::tag::count,
                                boost::accumulators::tag::mean,
                                boost::accumulators::tag::geometric_mean_mirror,
                                boost::accumulators::tag::max>> tanimoto_acc_t;

    // In case of ss filter, the candidates during optimization will
    // not have their scores computed over the whole dataset, that
    // method corrects that.
    void recompute_scores_over_whole_dataset(
        std::vector<std::vector<deme_t>>& ss_demes,
        const boost::ptr_vector<representation>& reps);

    // For each breadth first deme compute whether it passes the
    // subsample filter
    std::vector<bool> ss_filter(
        const std::vector<std::vector<deme_t>>& all_demes,
        const boost::ptr_vector<representation>& reps) const;

    // Return stats about the Tanimoto distances of a vector of combo trees
    void ss_tanimoto_stats(const std::vector<combo_tree>& trs,
                           tanimoto_acc_t& acc) const;

    // Return stats about the Tanimoto distances of the top candidates
    // of a certain breadth first deme
    void ss_tanimoto_stats(const representation& rep,
                           const std::vector<deme_t>& ss_demes,
                           tanimoto_acc_t& acc) const;

    // Return true if the top candidates of the subsampled demes have
    // sufficiently low Tanimoto distance (mean or max). The Tanimoto
    // distance is used because it is a generalization of the Jaccard
    // distance over multisets. Here we are dealing with multisets
    // because rows of ctables are weighted by their uncompressed
    // count.
    bool ss_tanimoto_filter(const representation& rep,
                            const std::vector<deme_t>& ss_demes) const;

    // --------------------- Printing/Logging functions --------------------
public:
    // log the best candidates
    void log_best_candidates() const;

    std::ostream& ostream_metapop(std::ostream&, int n = INT_MAX) const;

private:
    void log_selected_exemplar(scored_combo_tree_ptr_set::const_iterator);

protected:
    // --------------------- Internal state -----------------------
    const metapop_parameters& _params;

    const subsample_deme_filter_parameters& _filter_params;

    behave_cscore& _cscorer;

    scored_combo_tree_ptr_set _scored_trees;

    static const unsigned _min_pool_size = 250;

    size_t _merge_count;

    // The best score ever found during search.
    composite_score _best_cscore;

    // Trees with composite score equal to _best_cscore.
    scored_combo_tree_set _best_candidates;

    /// _visited_exemplars contains the exemplars of demes that have
    ///  been previously expanded. The count indicated the number of
    /// times that they've been expanded.
    typedef std::unordered_map<scored_combo_tree, unsigned,
                               scored_combo_tree_hash,
                               scored_combo_tree_equal> scored_tree_counter;

    scored_tree_counter _visited_exemplars;

    /// Return true iff the tree has already been visited; that is, if
    /// its in _visited_exemplars
    bool has_been_visited(const scored_combo_tree&) const;

    // lock to enable thread-safe deme merging.
    std::mutex _merge_mutex;

    // For now, the ensemble is along for the ride.  Someday, perhaps
    // it should enjoy life ndependently of the metapop.
    ensemble _ensemble;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_METAPOPULATION_H
