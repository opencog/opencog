/** metapopulation.h --- 
 *
 * Copyright (C) 2010 Novemente LLC
 *
 * Authors: Nil Geisweiller, Moshe Looks
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

#include <opencog/util/selection.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/numeric.h>

#include <opencog/learning/moses/eda/instance_set.h>

#include <opencog/comboreduct/reduct/reduct.h>

#include "representation.h"
#include "scoring.h"
#include "types.h"

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

#define EVALUATED_ALL_AVAILABLE 1234567

namespace moses
{

using opencog::pow2;

typedef std::set<combo::vertex> operator_set;
typedef std::set<combo::combo_tree,
                 opencog::size_tree_order<combo::vertex> > combo_tree_ns_set;

/**
 * parameters about deme management
 */
struct metapop_parameters {
    metapop_parameters(int _max_candidates = -1,
                       bool _reduce_all = true,
                       bool _revisit = false,
                       bool _ignore_bscore = false) :
        selection_max_range(28),
        max_candidates(_max_candidates),
        reduce_all(_reduce_all),
        revisit(_revisit),
        ignore_bscore(_ignore_bscore)
    { }
    
    // when doing selection of examplars according to 2^-n, where n is
    // complexity, only examplars with p>=2^-selection_max_range will
    // be considered
    double selection_max_range;
    // the max number of candidates considered to be added to the
    // metapopulation, if negative then all candidates are considered
    int max_candidates;
    // if true then all candidates are reduced before evaluation
    bool reduce_all;
    // when true then visited exemplars can be revisited
    bool revisit;
    // ignore the behavioral score when merging candidates in the population
    bool ignore_bscore;
};

/**
 * greater_than operator for bscored_combo_tree.  The order is as
 * follow 1 the score matter, then complexity, then the combo_tree
 * itself. This is done (formerly replacing
 * std::greater<bscored_combo_tree>) so that candidates of same score
 * and same complexity can be added in the metapopulation.
 */
struct bscored_combo_tree_greater : public binary_function<bscored_combo_tree,
                                                           bscored_combo_tree,
                                                           bool> {
    bool operator()(const bscored_combo_tree& bs_tr1,
                    const bscored_combo_tree& bs_tr2) const {
        score_t sc1 = get_score(bs_tr1);
        score_t sc2 = get_score(bs_tr2);
        if(sc1 > sc2)
            return true;
        if(sc1 < sc2)
            return false;
        complexity_t c1 = get_complexity(bs_tr1);
        complexity_t c2 = get_complexity(bs_tr2);
        if(c1 < c2)
            return true;
        if(c1 > c2)
            return false;
        return opencog::size_tree_order<combo::vertex>()(get_tree(bs_tr1),
                                                         get_tree(bs_tr2));
    }
};
    
/**
 * The metapopulation will store the expressions (as scored trees)
 * that were encountered during the learning process (which some of
 * them, dominated by exsiting ones, might be skipped as
 * non-promising)
 * 
 * The metapopulation is updated in iterations. In each iteration, one
 * of its elements is selected as an exemplar. The exemplar is then
 * used for building a new deme (that will, further, extend the
 * metapopulation)
 * 
 * NOTE:
 *   BScoring = behavioral scoring function (output behaviors), we use std::greater
 *   because we are maximizing
 *
 */
template<typename Scoring, typename BScoring, typename Optimization>
struct metapopulation : public set < bscored_combo_tree,
                                     bscored_combo_tree_greater > {
    typedef boost::unordered_set<combo_tree,
                                 boost::hash<combo_tree> > combo_tree_hash_set;

    // init the metapopulation with the following set of exemplars
    void init(const std::vector<combo_tree>& exemplars) {
        metapop_candidates candidates;
        foreach(const combo_tree& base, exemplars) {
            combo_tree si_base(base);
            (*simplify_candidate)(si_base);
            composite_score base_sc = make_pair(score(si_base),
                                                complexity(si_base));
            bscored_combo_tree base_bsc = 
                make_pair(si_base, 
                          composite_behavioral_score(bscore(si_base), base_sc));
            candidates.insert(base_bsc);

        }
        update_best_candidates(candidates);
        merge_candidates(candidates);
    }

    /**
     *  Constuctor for the class metapopulation
     *  
     * @param _rng    rand number 
     * @param bases   exemplars used to initialize the metapopulation
     * @param tt      type of expression to be learned
     * @param si      reduct rule for reducting
     * @param sc      scoring function for scoring
     * @param bsc     behavior scoring function
     * @param opt     optimization should be providing for the learning
     * @param pa      parameter for selecting the deme 
     */
    metapopulation(opencog::RandGen& _rng,
                   const std::vector<combo_tree>& bases,
                   const combo::type_tree& tt,
                   const reduct::rule& si_ca,
                   const reduct::rule& si_kb,
                   const Scoring& sc, const BScoring& bsc,
                   const Optimization& opt = Optimization(),
                   const metapop_parameters& pa = metapop_parameters()) :
        rng(_rng), type(tt), simplify_candidate(&si_ca),
        simplify_knob_building(&si_kb), score(sc),
        bscore(bsc), optimize(opt), params(pa), _n_evals(0),
        _best_score(worst_possible_score), _rep(NULL), _deme(NULL)
    {
        init(bases);
    }

    // like above but using a single base, and a single reduction rule
    // this constructor is used for back compatibility and should be
    // eventually removed
    metapopulation(opencog::RandGen& _rng,
                   const combo_tree& base,
                   const combo::type_tree& tt,
                   const reduct::rule& si,
                   const Scoring& sc, const BScoring& bsc,
                   const Optimization& opt = Optimization(),
                   const metapop_parameters& pa = metapop_parameters()) :
        rng(_rng), type(tt), simplify_candidate(&si),
        simplify_knob_building(&si), score(sc),
        bscore(bsc), optimize(opt), params(pa), _n_evals(0),
        _best_score(worst_possible_score), _rep(NULL), _deme(NULL)
    {
        std::vector<combo_tree> bases(1, base);
        init(bases);
    }

    ~metapopulation() {
        delete _rep;
        delete _deme;
    }
    /**
     * return the n_evals
     */
    const int& n_evals() const {
        return _n_evals;
    }
    int& n_evals() {
        return _n_evals;
    }

    /**
     * return the best composite score
     */
    const composite_score& best_composite_score() const {
        return _best_score;
    }

    /**
     * return the best score
     */
    score_t best_score() const {
        return get_score(_best_score);
    }

    /**
     * return the best candidates (with _best_score)
     */
    const metapop_candidates& best_candidates() const {
        return _best_candidates;
    }

    /**
     * return the best combo tree (shortest best candidate)
     */
    const combo_tree& best_tree() const {
        return _best_candidates.begin()->first;
    }

    const combo_tree_hash_set& visited() const {
        return _visited_exemplars;
    }
    combo_tree_hash_set& visited() {
        return _visited_exemplars;
    }

    /**
     * Select the exemplar from the population. All candidates with
     * the best score (if more that one) are distributed according to
     * a Solomonoff-like distribution (2^{-complexity}) and the
     * exemplar is selected accordingly.
     * 
     * @return the iterator of the selected exemplar, if no such
     *         exemplar exists then return end()
     */
    const_iterator select_exemplar() const {
        OC_ASSERT(!empty(), "Empty metapopulation in select_exemplar().");
        
        //compute the probs for all candidates with best score
        score_t score = get_score(*begin());
        complexity_t cmin = get_complexity(*begin());

        vector<complexity_t> probs;
        // set to true when a potential exemplar to be selected is
        // found
        bool exist_exemplar = false;

        for(const_iterator it = begin(); it != end(); ++it) {
            // if no exemplar has been found for that score then look
            // at the next lower score
            if(get_score(*it) != score) {
                if(!exist_exemplar) {
                    score = get_score(*it);
                    cmin = get_complexity(*it);
                }
                else break;
            }

            complexity_t c = get_complexity(*it);

            // this to not consider too complex exemplar
            if (c - cmin > params.selection_max_range) 
                break;
            const combo_tree& tr = get_tree(*it);
            if(_visited_exemplars.find(tr) == _visited_exemplars.end()) {
                probs.push_back(c);
                exist_exemplar = true;
            } else // hack: if the tree is visited then put a complexity positive
                   // so we know it must be ignored
                probs.push_back(-1);
        }
        
        if(!exist_exemplar) {
            return end(); // there is no exemplar to select
        }

        complexity_t sum = 0;
        complexity_t max_comp = *max_element(probs.begin(), probs.end());
        // convert complexities into (non-normalized) probabilities
        foreach(complexity_t& p, probs) {
            // in case p has the max complexity (already visited) then
            // the probability is set to null
            p = (p < 0? 0 : pow2(max_comp - p));
            sum += p;
        }

        OC_ASSERT(sum > 0, "There is an internal bug, please fix it");

        const_iterator exemplar = begin();
        advance(exemplar, distance(probs.begin(),
                                   opencog::roulette_select(probs.begin(),
                                                            probs.end(), 
                                                            sum, rng)));
        return exemplar;
    }

    void merge_candidates(const metapop_candidates& candidates) {
        if(params.ignore_bscore)
            insert(candidates.begin(), candidates.end());
        else
            merge_nondominating(candidates.begin(), candidates.end());
    }

    /**
     * expand do representation-building and create a deme first, and
     * then do some optimization according to the scoring function,
     * and add all unique non-dominated trees in the final deme as
     * potential exemplars for future demes.
     *
     * @param max_evals    the max evals
     * @param ignore_ops   the operator set to ignore
     * @param perceptions  set of perceptions of an interactive agent
     * @param actions      set of actions of an interactive agent
     *
     * @return return true if expansion has succeeded, false otherwise
     *
     */
    bool expand(int max_evals,
                const operator_set& ignore_ops = operator_set(),
                const combo_tree_ns_set* perceptions = NULL,
                const combo_tree_ns_set* actions = NULL)  {
        if(!create_deme(ignore_ops, perceptions, actions))
            return false;
            
        _n_evals += optimize_deme(max_evals);

        close_deme();

        // Logger
        stringstream ss;
        ss << "Total number of evaluations so far: " << _n_evals;
        logger().info(ss.str());
        log_best_candidates();
        // ~Logger

        //this may happens for instance if the eval fails and throws an exception
        return !empty();
    }

    /**
     * Create the deme
     *
     * @param ignore_ops   the operators to ignore
     * @param perceptions  a set of perceptions of an interactive agent
     * @param actions      a set of actions of an interactive agent
     *
     * @return return true if it creates deme successfully,otherwise false.
     */
    bool create_deme(const operator_set& ignore_ops = operator_set(),
                     const combo_tree_ns_set* perceptions = NULL,
                     const combo_tree_ns_set* actions = NULL)  {

        using namespace reduct;

        if (_rep != NULL || _deme != NULL)
            return false;

        if (empty())
            return false;

        do { // attempt to create a non-empty representation
            _exemplar = select_exemplar();
            if(_exemplar == end()) {
                // Logger
                logger().info("There is no more exemplar in the"
                              " metapopulation that has not been visited");
                // ~Logger
                if(params.revisit) {
                    _visited_exemplars.clear();
                    // Logger
                    logger().info("All visited exemplar has been untagged"
                                  " and can be visited again");
                    // ~Logger
                    continue;
                } else {
                    // Logger
                    logger().info("To revisit already visited exemplar you may"
                                " use option --revisit or -R");
                    // ~Logger
                    return false;
                }
            }

            combo_tree tr(get_tree(*_exemplar));

            // Logger
            { 
                stringstream ss; 
                ss << "Attempt to expand with exemplar: " << tr; 
                logger().debug(ss.str()); 
            }
            { 
                stringstream ss; 
                ss << "Scored: " << score(tr); 
                logger().debug(ss.str()); 
            }
            // ~Logger

            //do representation-building and create a deme (initially empty)
            _rep = new representation(*simplify_candidate,
                                      *simplify_knob_building,
                                      _exemplar->first, type,
                                      rng, ignore_ops, perceptions, actions);

            // if the representation is empty try another exemplar
            if(_rep->fields().empty()) {
                delete(_rep);
                _rep = NULL;
                _visited_exemplars.insert(get_tree(*_exemplar));
                // Logger
                logger().debug("The representation is empty, perhaps the reduct"
                               " effort for knob building is too high");
                // ~Logger
            }
        } while(!_rep);

        // create an empty deme
        _deme = new eda::instance_set<composite_score>(_rep->fields());

        _evals_before_this_deme = n_evals();

        return true;
    }

    /**
     * Do some optimization according to the scoring function.
     *
     * sliced version.
     *
     * @param max_evals the max evals
     * @param max_for_slice the max for slice 
     * @param max_score the max score 
     * 
     * @return return the number of evaluations actually performed,
     *         return -1 if all available is evaluated.
     */
    int optimize_deme(int max_evals, int max_for_slice,
                      score_t max_score) {

        if (_rep == NULL || _deme == NULL)
            return -1;

        //do some optimization according to the scoring function
        optimize.set_evals_per_slice(max_for_slice);
        int n;
        complexity_based_scorer<Scoring> scorer =
            complexity_based_scorer<Scoring>(score, *_rep, params.reduce_all);
        n = optimize(*_deme, scorer, max_evals);                

        // This is very ugly, but saves the old MOSES' architecture
        // The only return value of the operator is used for two
        // sorts of information - how many new evaluations were made
        // and if the building should be restarted (PJ)
        if (n == EVALUATED_ALL_AVAILABLE)
            return -1;

        if (n < 0)
            _n_evals += -n;
        else
            _n_evals += n;

        return n;
    }

    /**
     * Do some optimization according to the scoring function.
     *
     * non-sliced version.
     *
     * @todo: there should be no sliced or non-sliced version, instead
     * the optimization API should be simple and allow slice for any
     * optimizer
     *
     * @param max_evals the max evals
     * 
     * @return return the number of evaluations actually performed,
     */
    int optimize_deme(int max_evals) {
        // Logger
        {
            logger().debug("Optimize deme");
            stringstream ss;
            ss << "Maximum evaluations during that expansion: " 
               << max_evals;
            logger().debug(ss.str());
        }
        // ~Logger

        complexity_based_scorer<Scoring> scorer =
            complexity_based_scorer<Scoring>(score, *_rep, params.reduce_all);
        return optimize(*_deme, scorer, max_evals);
    }

    /**
     * close deme
     * 1) mark the current deme exemplar to not explore it again,
     * 2) merge non-dominated candidates in the metapopulation,
     * 3) delete the deme instance from memory.
     */
    void close_deme() {
        if (_rep == NULL || _deme == NULL)
            return;

        int eval_during_this_deme = n_evals() - _evals_before_this_deme;

        // Logger
        {
            logger().debug("Close deme");
            logger().debug("Actual number of evaluations during that expansion: %d",
                           eval_during_this_deme);
        }
        // ~Logger

        //mark the exemplar so we won't expand it again
        _visited_exemplars.insert(get_tree(*_exemplar));

        //add (as potential exemplars for future demes) all unique non-dominated
        //trees in the final deme
        metapop_candidates candidates;

        int i = 0;

        // Logger
        logger().debug("Sort the deme");
        // ~Logger

        // sort the deme according to composite_score (descending order)
        std::sort(_deme->begin(), _deme->end(),
                  std::greater<eda::scored_instance<composite_score> >());

        // Logger
        logger().debug("Compute the behavioral score of all candidates to merge");
        // ~Logger

        foreach(const eda::scored_instance<composite_score>& inst, *_deme) {
            // this is in case the deme is closed before the entire
            // deme (or rather the current sample of it) has been
            // explored
            if (i++ == eval_during_this_deme)
                break;

            // if it's really bad just skip it and all that follow
            if (get_score(inst.second) == get_score(worst_possible_score))
                break;

            // generate the tree coded by inst
            // turn the knobs of rep._exemplar as stored in inst
            _rep->transform(inst);

            // get the combo_tree associated to inst, cleaned and reduced
            // @todo: here the canidate is possibly reduced for the second time
            // this could probability be avoid with some clever cache or something
            combo_tree tr = _rep->get_clean_exemplar(true);

            // update the set of potential exemplars
            if (_visited_exemplars.find(tr) == _visited_exemplars.end()
                && candidates.find(tr) == candidates.end()) {

                // only add up to max_candidates
                if(params.max_candidates < 0
                   || (int)candidates.size() < params.max_candidates) {
                    // recompute the complexity if the candidate has
                    // not been previously reduced
                    composite_score csc = params.reduce_all?
                        inst.second : make_pair(get_score(inst.second),
                                                complexity(tr));
                    bscored_combo_tree candidate = 
                        make_pair(tr, composite_behavioral_score(bscore(tr), csc));
                    candidates.insert(candidate);
                } else 
                    break;
            }
        }
        // update the record of the best-seen score & trees
        update_best_candidates(candidates);

        //Logger
        logger().debug("Merge %u candidates with the metapopulation",
                       candidates.size());
        if(logger().getLevel() >= opencog::Logger::FINE) {
            logger().fine("Candidates with their bscores to merge with"
                          " the metapopulation:");
            stringstream ss;
            logger().fine(ostream(ss, candidates.begin(), candidates.end(),
                                  -1, true, true).str());
        }
        // ~Logger

        merge_candidates(candidates);

        //Logger
        logger().debug("Metapopulation size is %u", size());
        if(logger().getLevel() >= opencog::Logger::FINE) {
            stringstream ss;
            ss << "Metapopulation after merging:" << std::endl;
            logger().fine(ostream(ss, -1, true, true).str());
        }
        // ~Logger

        delete _deme;
        delete _rep;
        _deme = NULL;
        _rep = NULL;
    }

    /**
     * return true if x dominates y
     *        false if y dominates x
     *        indeterminate otherwise
     */
    tribool dominates(const behavioral_score& x, const behavioral_score& y)
    {
        //everything dominates an empty vector
        if (x.empty()) {
            if (y.empty())
                return indeterminate;
            return false;
        } else if (y.empty()) {
            return true;
        }
        
        tribool res = indeterminate;
        for (behavioral_score::const_iterator xit = x.begin(), yit = y.begin();
             xit != x.end();++xit, ++yit) {
            if (*xit < *yit) { //individual elements are assumed to represent error
                if (!res)
                    return indeterminate;
                else
                    res = true;
            } else if (*yit < *xit) {
                if (res)
                    return indeterminate;
                else
                    res = false;
            }
        }
        return res;
    }

    /**
     * For all candidates c in [from, to), insert c in dst iff 
     * no element of (dst - _visited_exemplars) dominates c.
     */
    //this may turn out to be too slow...
    template<typename It>
    void merge_nondominating(It from, It to) {
        for(;from != to;++from) {
            bool nondominated = true;
            for(iterator it = begin(); it != end();) {
                tribool dom = dominates(from->second, it->second);
                if (dom) {
                    erase(it++);
                } else if (!dom) {
                    nondominated = false;
                    break;
                } else {
                    ++it;
                }
            }
            if (nondominated)
                insert(*from);
        }
    }

    // update the record of the best-seen score & trees
    void update_best_candidates(const metapop_candidates& candidates) {
        foreach(const bscored_combo_tree& candidate, candidates) {
            const composite_score& sc = get_composite_score(candidate);
            if (sc >= _best_score) {
                if (sc > _best_score) {
                    _best_score = sc;
                    _best_candidates.clear();
                }
                _best_candidates.insert(candidate);
            }
        }
    }

    // log the best candidates
    void log_best_candidates() const {
        if(best_candidates().empty())
            logger().info("Only worst scored candidates");
        else {
            stringstream ss;
            ss << "The following candidate(s) have the best score " 
               << best_score();
            logger().info(ss.str());
            foreach(const bscored_combo_tree& cand, best_candidates()) {
                stringstream ss_tr;
                ss_tr << get_tree(cand);
                logger().info(ss_tr.str());
            }
        }
    }

    /**
     * stream out the n best non dominated candidates along with their
     * scores (optionally complexity and bscore), if n is negative
     * stream them all out.
     */
    template<typename Out, typename In>
    Out& ostream(Out& out, In from, In to, long n = -1,
                 bool output_score = true,
                 bool output_complexity = false,
                 bool output_score_complexity_old_moses = false,
                 bool output_bscore = false) {
        for(; from != to && n != 0; from++, n--) {
            ostream_bscored_combo_tree(out, *from, output_score,
                                       output_complexity,
                                       output_score_complexity_old_moses,
                                       output_bscore);
        }
        return out;
    }
    // like above but assumes that from = begin() and to = end()
    template<typename Out>
    Out& ostream(Out& out, long n = -1,
                 bool output_score = true,
                 bool output_complexity = false,
                 bool output_score_complexity_old_moses = false,
                 bool output_bscore = false) {
        return ostream(out, begin(), end(),
                       n, output_score, output_complexity,
                       output_score_complexity_old_moses,
                       output_bscore);
    }
    // like above but using std::cout
    void print(long n = -1,
               bool output_score = true,
               bool output_complexity = false,
               bool output_score_complexity_old_moses = false,
               bool output_bscore = false) {
        ostream(std::cout, n, output_score, output_complexity,
                output_score_complexity_old_moses,
                output_bscore);
    }

    opencog::RandGen& rng;
    combo::type_tree type;
    const reduct::rule* simplify_candidate; // to simplify candidates
    const reduct::rule* simplify_knob_building; // during knob building
    const Scoring& score;
    const BScoring& bscore; // behavioral score
    Optimization optimize;
    metapop_parameters params;

protected:
    int _n_evals;
    int _evals_before_this_deme;

    // the best score ever
    composite_score _best_score; 

    // trees with score _best_score
    metapop_candidates _best_candidates;

    // contains the exemplars of demes that have been searched so far
    combo_tree_hash_set _visited_exemplars;
    
    representation* _rep; // representation of the current deme
    eda::instance_set<composite_score>* _deme; // current deme
    const_iterator _exemplar; // exemplar of the current deme
};

} // ~namespace moses

#endif // _OPENCOG_METAPOPULATION_H
