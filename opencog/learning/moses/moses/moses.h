/*
 * opencog/learning/moses/moses/moses.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _MOSES_MOSES_H
#define _MOSES_MOSES_H

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/reduct/meta_rules.h>

#include <opencog/util/lru_cache.h>
#include <opencog/util/selection.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/hash_set.h>

#include <opencog/learning/moses/eda/instance_set.h>

#include "representation.h"
#include "scoring.h"

#define EVALUATED_ALL_AVAILABLE 1234567

namespace moses
{

typedef std::set<combo::vertex> operator_set;
typedef std::set<combo::combo_tree,
                 opencog::size_tree_order<combo::vertex> > combo_tree_ns_set;
    
typedef std::multimap<fitness_t,
                      combo_tree,
                      std::greater<fitness_t> > ordered_programs;

typedef ordered_programs::iterator ordered_programs_it;


//BScoring = behavioral scoring function (output behaviors)
template<typename Scoring, typename BScoring, typename Optimization>
struct metapopulation : public set < behavioral_scored_combo_tree,
                                     std::greater<behavioral_scored_combo_tree> > {
    //greater because we are maximizing

    struct parameters {
        parameters() :
            local_cache(false),      //are instance scores cached at the deme level?
            global_cache(false),     //at the global (program) level across demes?
            simplify_all(false),     //are all programs generated simplified?
            
            selection_max_range(11), //when doing selection of examplars according to
            //2^-n, where n is complexity, only examplars with
            //p>=2^-selection_max_range will be considered

            scoring_epsilon(0.01)    //scores are considered equal when within epsilon
        { }

        bool local_cache;
        bool global_cache;
        bool simplify_all;

        double selection_max_range;
        double scoring_epsilon;
    };

    metapopulation(opencog::RandGen& _rng, const combo_tree& base,
                   const combo::type_tree& t, const reduct::rule& si,
                   const Scoring& sc, const BScoring& bsc,
                   const Optimization& opt = Optimization(),
                   const parameters& pa = parameters()) :
        rng(_rng), type(t), simplify(&si), score(sc),
        bscore(bsc), optimize(opt), params(pa),
        _n_evals(0), _best_score(worst_possible_score),
        _rep(NULL), _deme(NULL)

    {
        insert(behavioral_scored_combo_tree
               (base, combo_tree_behavioral_score(behavioral_score(),
                                            combo_tree_score(_best_score.first, 0))));
    }

    ~metapopulation() {
        delete _rep;
        delete _deme;
    }

    int n_evals() const {
        return _n_evals;
    }
    const combo_tree_score& best_score() const {
        return _best_score;
    }
    const std::vector<combo_tree>& best_trees() const {
        return _best_trees;
    }

    const_iterator select_exemplar() const {
        opencog::cassert(TRACE_INFO, !empty(),
                         "Empty metapopulation in function select_exemplar().");
        
        cout << "METAPOPULATION size : " << size() << endl << endl;

        //compute the probs for all candidates
        score_t score = get_score(*begin());
        complexity_t cmin = get_complexity(*begin());
        vector<complexity_t> probs;

        for (const_iterator it = begin();
             it != end() && get_score(*it) == score;++it) {
            complexity_t c = get_complexity(*it);
            if (cmin - c > params.selection_max_range)
                break;
            probs.push_back(c);
        }

        complexity_t sum = 0;
        foreach(complexity_t& p, probs) {
            p = (1 << (*max_element(probs.begin(), probs.end()) - p)); // Moshe's fix; 18.02.2008.
            sum += p;
        }

        const_iterator exemplar = begin();
        advance(exemplar, distance(probs.begin(), opencog::roulette_select
                                   (probs.begin(), probs.end(), sum, rng)));
        return exemplar;
    }


    //it returns true if expansion has succeeded, false otherwise
    bool expand(int max_evals,
                const combo_tree_score& max_score,
                const operator_set* os = NULL,
                const combo_tree_ns_set* perceptions = NULL,
                const combo_tree_ns_set* actions = NULL)  {

        using namespace reduct;

        // cassert(TRACE_INFO, !empty(), "Empty metapopulation in function expand(..).");
        if (empty())
            return false;

        iterator exemplar = select_exemplar();

        combo_tree tr(exemplar->first);
        cout << endl << "Current exemplar:" << tr << endl;
        //clean_and_full_reduce(tr);
        //cout << "Exemplar after reduction:" << tr << endl;
        cout << "Exemplar's score: " << score(tr) << endl;
        cout << "max evals in this expand " << max_evals << endl << endl;

        //do representation-building and create a deme (initially empty)
        representation rep(*simplify, exemplar->first, type,
                           rng, os, perceptions, actions);

        eda::instance_set<combo_tree_score> deme(rep.fields());

        //remove the examplar and mark it so we won't expand it again
        _visited_exemplars.insert(exemplar->first);
        erase(exemplar);

        //do some optimization according to the scoring function
        /*_n_evals+=optimize(deme,complexity_based_scorer<Scoring>(score,rep),
        max_evals);*/
        _n_evals += optimize(deme, count_based_scorer<Scoring>(score, rep,
                             get_complexity(*exemplar), rng),
                             max_evals);

        //add (as potential exemplars for future demes) all unique non-dominated
        //trees in the final deme
        opencog::hash_map<combo_tree, combo_tree_behavioral_score, boost::hash<combo_tree> > candidates;
        foreach(const eda::scored_instance<combo_tree_score>& inst, deme) {

#ifdef DEBUG_INFO
            cout << "Instance: " << inst.second << endl;
            cout << "Instance score : " << get_score(inst.second) << endl;
            cout << "Worst score: " << worst_possible_score << endl;
#endif

            //if its really bad just skip it
            if (get_score(inst.second) == get_score(worst_possible_score))
                continue;

            //generate the tree
            combo_tree tr;
            //cout << "inst : " << rep.fields().stream(inst) << endl;
            rep.transform(inst);

            //tr=rep.exemplar();        // otherwise dangling junctors and
            rep.get_clean_exemplar(tr);  // the scoring are not in accordance with each other
            //apply_rule(sequential(*simplify),tr,tr.begin());

#ifdef DEBUG_INFO
            cout << "Instance explored: " << inst.second << endl;
            cout << tr << endl;
#endif

            //update the set of potential exemplars
            if (_visited_exemplars.find(tr) == _visited_exemplars.end() &&
                    candidates.find(tr) == candidates.end()) {
                candidates.insert(make_pair(tr,
                                            combo_tree_behavioral_score(bscore(tr),
                                            inst.second)));
                // also update the record of the best-seen score & trees
                if (inst.second >= _best_score) {
                    if (inst.second > _best_score) {
                        _best_score = inst.second;
                        _best_trees.clear();
                    }
                    _best_trees.push_back(tr);
                }
            }
        }

#ifdef DEBUG_INFO
        cout << "Candidates size: " << candidates.size() << endl;
        cout << "Dc: " << distance(candidates.begin(), candidates.end()) << endl << endl;
#endif

        merge_nondominating(candidates.begin(), candidates.end(), *this);

        //log some exemplars
        for (const_iterator it = begin();
             it != end() && distance(begin(), it) < 3;++it)
            cout << "exemplar #" << distance(begin(), it)
                 << " " << get_tree(*it) << " "
                 << get_score(*it) << " "
                 << get_complexity(*it) << endl;

        cout << endl << "Number of evals performed: " << n_evals() << endl;

        //in case all candidates can't get a better score than the worst
        //this may happens for instance if the eval fails and throws an exception
        return !empty();
    }



    bool create_deme(const operator_set* os = NULL,
                     const combo_tree_ns_set* perceptions = NULL,
                     const combo_tree_ns_set* actions = NULL)  {

        using namespace reduct;

        if (_rep != NULL || _deme != NULL)
            return false;

        if (empty())
            return false;
        // cassert(TRACE_INFO, !empty(), "Empty metapopulation in function expand(..).");

        _exemplar = select_exemplar();

        cout << "Current exemplar:" << (_exemplar->first) << endl;

        combo_tree tr(_exemplar->first);
        //cout << endl << "Current exemplar:" << tr << endl;
        //clean_and_full_reduce(tr);
        //cout << "Exemplar after reduction:" << tr << endl;
        cout << "Exemplar's score: " << score(tr) << endl;
        cout << "exemplar complexity: " << get_complexity(*_exemplar) << endl;
        //cout << "Exemplar before going to rep:" << exemplar->first << endl;

        exemplar_complexity = get_complexity(*_exemplar);

        //do representation-building and create a deme (initially empty)
        _rep = new representation(*simplify, _exemplar->first, type,
                                  rng, os, perceptions, actions);
        _deme = new eda::instance_set<combo_tree_score>(_rep->fields());
        // _n_evals = 0;

        //remove the examplar and mark it so we won't expand it again
        _visited_exemplars.insert(_exemplar->first);
        erase(_exemplar);

        _evals_before_this_deme = n_evals();

        return true;
    }


    int optimize_deme(int max_evals, int max_for_slice,
                      const combo_tree_score& max_score) {

        if (_rep == NULL || _deme == NULL)
            return -1;

        cout << "Max evals in this expand " << max_evals << endl;
        cout << "Number of evals " << n_evals() << endl;
        cout << "Max per slice " << max_for_slice << endl;

        //do some optimization according to the scoring function
        optimize.set_evals_per_slice(max_for_slice);
        int n = optimize(*_deme,
                         count_based_scorer<Scoring>(score,
                                                     *_rep,
                                                     exemplar_complexity,
                                                     rng),
                         max_evals - n_evals());

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



    void close_deme(ordered_programs& op) {
        if (_rep == NULL || _deme == NULL)
            return;

        //add (as potential exemplars for future demes) all unique non-dominated
        //trees in the final deme
        opencog::hash_map<combo_tree, combo_tree_behavioral_score, boost::hash<combo_tree> > candidates;


        int i = 0;

        foreach(const eda::scored_instance<combo_tree_score>& inst, *_deme) {
            if (i++ == (n_evals() - _evals_before_this_deme))
                break;

            //if its really bad just skip it
            if (get_score(inst.second) == get_score(worst_possible_score))
                continue;

            //generate the tree
            combo_tree tr;
            //cout << "inst : " << rep.fields().stream(inst) << endl;
            _rep->transform(inst);

            //tr=_rep->exemplar();        // otherwise dangling junctors and
            _rep->get_clean_exemplar(tr);  // the scoring are not in accordance with each other
            //apply_rule(sequential(*simplify),tr,tr.begin());

#ifdef DEBUG_INFO
            cout << "Instance explored: " << inst.second << endl;
            cout << tr << endl;
#endif

            //update the set of potential exemplars
            if (_visited_exemplars.find(tr) == _visited_exemplars.end() &&
                    candidates.find(tr) == candidates.end()) {
                candidates.insert(make_pair(tr, combo_tree_behavioral_score(bscore(tr),
                                            inst.second)));
                // also update the record of the best-seen score & trees
                if (inst.second >= _best_score) {
                    if (inst.second > _best_score) {
                        _best_score = inst.second;
                        _best_trees.clear();
                    }
                    _best_trees.push_back(tr);
                }
            }
        }

#ifdef DEBUG_INFO
        cout << "Candidates " << candidates.size() << endl;
        cout << "DC: " << distance(candidates.begin(), candidates.end()) << endl << endl;
#endif

        merge_nondominating(candidates.begin(), candidates.end(), *this);

        //log some exemplars
        for (const_iterator it = begin();
             it != end() && distance(begin(), it) < 3;++it)
            cout << "exemplar #" << distance(begin(), it)
                 << " " << get_tree(*it) << " "
                 << get_score(*it) << " "
                 << get_complexity(*it) << endl;

        cout << endl << "Number of evals performed: " << n_evals() << endl;
        cout << "Metapopulation size : " << size() << endl << endl;

        // add some elements of the metapopulation to the given list of programs
        for (const_iterator it = begin();it != end();++it)  {
            combo_tree tr(get_tree(*it));

            (*simplify)(tr, tr.begin());
            // reduct::hillclimbing_full_reduce(tr);
            fitness_t s = get_score(*it);

            ordered_programs_it oi = op.find(s);
            if (oi == op.end() || ((oi->second).size() > tr.size())) {
                std::pair<fitness_t, combo_tree> p(s, tr);
                op.insert(p);
            }
        }

        delete _deme;
        delete _rep;
        _deme = NULL;
        _rep = NULL;
    }


    opencog::RandGen& rng;
    combo::type_tree type;
    const reduct::rule* simplify;
    Scoring score;
    BScoring bscore; //behavioral score
    Optimization optimize;
    parameters params;

protected:
    int _n_evals;
    int _evals_before_this_deme;
    combo_tree_score _best_score;
    std::vector<combo_tree> _best_trees;

    opencog::hash_set<combo_tree, boost::hash<combo_tree> > _visited_exemplars;

    // introduced for time-slicing version
    representation* _rep;
    eda::instance_set<combo_tree_score>* _deme;
    iterator _exemplar;
    complexity_t exemplar_complexity;
};


typedef std::set<combo::vertex> operator_set;
typedef std::set<combo::combo_tree, opencog::size_tree_order<combo::vertex> >
combo_tree_ns_set;




template<typename Scoring, typename Domination, typename Optimization>
void moses(metapopulation<Scoring, Domination, Optimization>& mp,
           int max_evals,
           const combo_tree_score& max_score)
{
    clock_t start, end;
    start = clock ();

    while (mp.n_evals() < max_evals) {
        cout << "mp size " << mp.size() << endl;

        //run a generation
        if (mp.expand(max_evals - mp.n_evals(), max_score)) {
            //print the generation number and a best solution
            std::cout << "sampled " << mp.n_evals()
                      << " best " << mp.best_score().first
                      << " " << mp.best_trees().front() << std::endl;
        } else // In iterative hillclimbing it is possible (but not likely) that the metapop gets empty and expand return false
            break;

        if (mp.best_score() >= max_score || mp.empty())
            break;
    }

    end = clock ();

    cout << endl << "Time elapsed  ======================== " << (end - start) / CLOCKS_PER_SEC << "seconds" << endl << endl;

}

template<typename Scoring, typename Domination, typename Optimization>
void moses(metapopulation<Scoring, Domination, Optimization>& mp,
           int max_evals, score_t max_score)
{
    moses(mp, max_evals, combo_tree_score(max_score, worst_possible_score.second));
}


// Lists of relevant operators, perceptions, and actions may or may not be
// provided. The initial design assumed fixed lists, this version
// has a constructor including these parameters, specific for actions
template<typename Scoring, typename Domination, typename Optimization>
void moses(metapopulation<Scoring, Domination, Optimization>& mp,
           int max_evals,
           const combo_tree_score& max_score,
           const operator_set* os,
           const combo_tree_ns_set* perceptions,
           const combo_tree_ns_set* actions,
           ordered_programs& op)
{
    clock_t start, end;
    start = clock ();

    while (mp.n_evals() < max_evals) {
        cout << "mp size " << mp.size() << endl;

        //run a generation
        int max_for_generation;
        max_for_generation = max_evals - mp.n_evals();

        if (mp.expand(max_for_generation, max_score, os, perceptions, actions)) {
            //print the generation number and a best solution
            std::cout << "sampled " << mp.n_evals()
                      << " best " << mp.best_score().first << " "
                      << mp.best_trees().front() << std::endl;
        } else // In iterative hillclimbing it is possible (but not likely) that the metapop gets empty and expand return false
            break;

        if (mp.best_score() >= max_score || mp.empty())
            break;
    }

    end = clock ();
    cout << endl << "Time elapsed  ======================== " << (end - start) / CLOCKS_PER_SEC << "seconds" << endl << endl;
}

template<typename Scoring, typename Domination, typename Optimization>
void moses(metapopulation<Scoring, Domination, Optimization>& mp,
           int max_evals, score_t max_score,
           const operator_set* os,
           const combo_tree_ns_set* perceptions,
           const combo_tree_ns_set* actions,
           ordered_programs& op)
{
    moses(mp, max_evals,
          combo_tree_score(max_score, worst_possible_score.second),
          os, perceptions, actions, op);
}




// Lists of relevant operators, perceptions, and actions may or may not be
// provided. The initial design assumed fixed lists, this version
// has a constructor including these parameters, specific for actions
template<typename Scoring, typename Domination, typename Optimization>
void moses_sliced(metapopulation<Scoring, Domination, Optimization>& mp,
                  int max_evals,
                  const combo_tree_score& max_score,
                  const operator_set* os,
                  const combo_tree_ns_set* perceptions,
                  const combo_tree_ns_set* actions,
                  ordered_programs& op)
{
    clock_t start, end;
    start = clock ();
    int o;
    int max_for_slice = 20;

    ordered_programs _ordered_best_estimates;

    while (mp.n_evals() < max_evals) {
        o = 0;
        if (mp.create_deme(os, perceptions, actions)) {
            while (o >= 0)
                o = mp.optimize_deme(max_evals, max_for_slice, max_score);

            mp.close_deme(_ordered_best_estimates);

            // ordered_programs_it oi;
            // cout << endl << "--------- BEST CANDIDATES SO FAR: ------------- " << endl;
            // for(oi = _ordered_best_estimates.begin();oi != _ordered_best_estimates.end();oi++)
            //  cout << oi->second << " score: " << oi->first << endl;
            // cout << "--------- BEST CANDIDATES SO FAR: ------------- " << endl << endl;
        } else
            break;
    }

    // print the best solution
    std::cout << "sampled " << mp.n_evals()
              << " best " << mp.best_score().first << endl
              << mp.best_trees().front() << std::endl;

    end = clock ();
    cout << "Time elapsed  ======================== " << (end - start) / CLOCKS_PER_SEC << "seconds" << endl;
}

template<typename Scoring, typename Domination, typename Optimization>
void moses_sliced(metapopulation<Scoring, Domination, Optimization>& mp,
                  int max_evals, score_t max_score,
                  const operator_set* os,
                  const combo_tree_ns_set* perceptions,
                  const combo_tree_ns_set* actions,
                  ordered_programs& op)
{
    moses_sliced(mp,
                 max_evals,
                 combo_tree_score(max_score, worst_possible_score.second),
                 os, perceptions, actions, op);
}






} //~namespace moses

#endif
