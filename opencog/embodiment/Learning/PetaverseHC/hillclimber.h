/*
 * opencog/embodiment/Learning/PetaverseHC/hillclimber.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _HILLCLIMBING_HILLCLIMBER_H
#define _HILLCLIMBING_HILLCLIMBER_H

#include <opencog/util/selection.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/numeric.h>
#include <opencog/util/RandGen.h>

#include <opencog/comboreduct/combo/vertex.h>
#include "NeighborhoodGenerator.h"
#include <ctime>
#include <vector>
#include <iostream>
#include <map>
#include <set>
#include <boost/unordered_set.hpp>

//uncomment those 2 lines if want cache
//don't forget to comment COUNT_NUMBER_OF_FITNESS as well
//#define IS_HC_CACHE
//#define ESTIMATOR_CACHE_SIZE 500000

//info for debug
//#define COUNT_NUMBER_OF_FITNESS

#define MIN_FITNESS -1.0e10
#define INIT_FITNESS 0.0 //supposed to be the average, neither good nor bad

//commant if want a probabilistic neighborhood re-expansion
#define DETERMINISTIC_REEXPANSION
//if probabilistic re-expansion then NARROWNESS determines how centered
//around the best candidates the re-expansion occurs
#define NARROWNESS 5.0

namespace opencog { namespace hillclimbing {

using namespace combo;

typedef double fitness_t;

template < typename FE, typename Combo_TreeComp = size_tree_order<vertex> >
struct hillclimber {
    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;

    typedef boost::unordered_set<combo_tree, boost::hash<combo_tree> > combo_tree_hash_set;
    typedef combo_tree_hash_set::iterator combo_tree_hash_set_it;
    typedef combo_tree_hash_set::const_iterator combo_tree_hash_set_const_it;

    typedef std::set<combo_tree, Combo_TreeComp> combo_tree_set;
    typedef typename combo_tree_set::iterator combo_tree_set_it;
    typedef typename combo_tree_set::const_iterator combo_tree_set_const_it;

    typedef combo_tree_set neighborhood;
    typedef combo_tree_set_it neighborhood_it;
    typedef combo_tree_set_const_it neighborhood_const_it;

    typedef std::multimap < fitness_t, combo_tree,
    std::greater<fitness_t> > ordered_neighborhood;
    typedef ordered_neighborhood::iterator ordered_neighborhood_it;
    typedef ordered_neighborhood::const_iterator ordered_neighborhood_const_it;

    typedef enum {
        HC_INIT,
        HC_REWARD,
        HC_START_ITERATION,
        HC_BUILD_CANDIDATES,
        HC_ESTIMATE_CANDIDATES,
        HC_IDLE
    } HCState;


    //- abibb stands for action_boolean_if_both_branches and is true
    //  the neighborhood generates both branches of a conditional
    //- neic stands for new_exemplar_initializes_center
    //  and if is true then everytime a new exemplar comes
    //  the center is initialized with the empty program instead of the best one
    hillclimber(const FE& fe, int fepc,
                const operator_set& os,
                const combo_tree_ns_set& conditions,
                const combo_tree_ns_set& actions,
                const Combo_TreeComp& comp,
                const rule& action_reduction,
                const rule& full_reduction,
                bool abibb,
                bool neic,
                bool reduct_enabled = true)
            : _fitnessEstimator(fe),
            _fitnessEstimationPerCycle(fepc),
#ifdef IS_HC_CACHE
            _estimator_cache(ESTIMATOR_CACHE_SIZE, _fitnessEstimator),
#endif
            _current_fitness(INIT_FITNESS),
            _current_fitness_estimated(MIN_FITNESS),
            _best_fitness(MIN_FITNESS),
            _best_fitness_estimated(MIN_FITNESS),
            _neighborhoodGenerator(os, conditions, actions,
                                   action_reduction, full_reduction,
                                   0, abibb, reduct_enabled),
            _neighborhood(comp),
            _hcState(HC_INIT),
            _number_of_fitness(0),
            _new_exemplar_initializes_center(neic)
    {
        //initialize neighborhood generator
        _neighborhoodGenerator.precomputeCompositePerceptions();
        _neighborhoodGenerator.precomputeCompositeActions();
    }

    ~hillclimber() { }

    //suggest the next best candidate, either by populating
    //a neighborhood and choosing the best of by suggesting
    //the next best neighbor among the set of untested neighbors
    void operator()() {
        switch (_hcState) {
        case HC_IDLE:
            sleep(1);
            break;
        case HC_REWARD: {
            bool must_choose_center;
            if (_current_fitness >= _best_fitness) {
                _best_fitness = _current_fitness;
                _best_program = _current_program;
                must_choose_center = has_been_used_as_center(_current_program);
            } else must_choose_center = true;
            if (must_choose_center) {
                if (choose_center(_center))
                    _hcState = HC_START_ITERATION;
                else //if there is no more center available
                    //then the entire candidate space
                    //has been explored
                    //and hillclimbing goes in idle state
                    _hcState = HC_IDLE;
            } else {
                _center = _current_program;
                _hcState = HC_START_ITERATION;
            }
            break;
        }
        case HC_INIT:
            if (_current_program.empty()) {
                OC_ASSERT(_center.empty(),
                        "_center must be empty, or maybe not?? Anyway fix that bug Nil!");
            }
            _hcState = HC_START_ITERATION;
            //no break
        case HC_START_ITERATION:
            _current_fitness_estimated = MIN_FITNESS;
            //no break
        case HC_BUILD_CANDIDATES:
            //populate neighborhood from the center
            //debug log
        {
            std::stringstream ss_center;
            ss_center << _center;
            std::string s_center = ss_center.str();
            logger().debug("hillclimber - Build candidates from center: %s", s_center.c_str());
        }
        //~debug log
        populate_neighborhood_from_center();
        _hcState = HC_ESTIMATE_CANDIDATES;
        //the break is commented to guaranty that the first few candidates
        //will be computed until the end of the cycle
        //so that if a try or stop learning message is received
        //a candidate solution will be suggested
        //break;
        case HC_ESTIMATE_CANDIDATES: {
            //estimate the fitness of the neighborhood
            //and fill _ordered_neighborhood using fitness as order
            estimate_fitness_at_most(_fitnessEstimationPerCycle);
            //update _best_fitness_estimated
            ordered_neighborhood_const_it oni = _ordered_best_estimates.begin();
            OC_ASSERT(!_ordered_best_estimates.empty(),
                    "_ordered_best_estimates should not be empty, if it may indeed be  empty then it is a bug, ask Nil to fix it");
            fitness_t fit = oni->first;
            if (fit >= _best_fitness_estimated) {
                _best_fitness_estimated = fit;
                _best_program_estimated = oni->second;
            }
            //if the neighborhood is empty then choose the center
            //for the next iteration
            if (_neighborhood.empty()) {
                if (choose_center(_center))
                    _hcState = HC_BUILD_CANDIDATES;
                else _hcState = HC_IDLE;
            }
        }
        break;
        default:
            OC_ASSERT(false, "Hillclimber operator unknown case");
            break;
        }
    }

    //the best fitness level and program observed (undefined if no calls made)
    //best_program returns the smallest tree if multiple trees have the
    //same observed fitness
    fitness_t best_fitness() const {
        return _best_fitness;
    }
    const combo_tree& best_program() const {
        return _best_program;
    }

    //the best fitness estimate and program observed (undefined if no
    //calls made)
    //best_program returns the smallest tree if multiple trees have the
    //same estmated fitness
    fitness_t best_fitness_estimated() const {
        return _best_fitness_estimated;
    }
    const combo_tree& best_program_estimated() const {
        return _best_program_estimated;
    }

    //the current program and it's fitness (undefined if no calls made)
    fitness_t current_fitness() const {
        return _current_fitness;
    }
    //set the fitness of the current program and set _hcState to HC_REWARD
    void set_current_fitness(fitness_t f) {
        _current_fitness = f;
        _hcState = HC_REWARD;
        _neighborhood.clear();
    }

    //return the best estimated that has not been sent to the user before
    //WARNING : current_program() has a side effect since it informs
    //that that program is delivered to the user and therefor will not
    //be sent again
    const combo_tree& current_program() {

        //returns the best program which has never been sent to the owner
        ordered_neighborhood_it oni = random_not_sent_best_program();
        if (oni == _ordered_best_estimates.end()) { //no such best program
            _current_program.clear();
        } else {
            _current_program = oni->second;
            _used_for_owner.insert(_current_program);

            fitness_t cur_est_fit = oni->first;
            _ordered_best_estimates.erase(oni);

            if (cur_est_fit >= _best_fitness_estimated) {
                _best_fitness_estimated = cur_est_fit;
                _best_program_estimated = _current_program;
            }
        }
        return _current_program;
    }

    void reset_estimator() {
#ifdef IS_HC_CACHE
        _estimator_cache.clear();
#endif
        _ordered_best_estimates.clear(); //because its content is out of date
        _used_as_center.clear();

        //depending on the option
        //we either get restarted from the best program so far
        //or from the start (the empty combo_tree)
        if (_new_exemplar_initializes_center)
            _center = combo_tree();
        else _center = (_best_program.empty() ?
                            _best_program_estimated : _best_program);

        _neighborhood.clear();

        _best_fitness_estimated = MIN_FITNESS;
        _best_program_estimated.clear();

        //rebuild composite actions
        _neighborhoodGenerator.clearCompositeActions();
        _neighborhoodGenerator.clearCompositePerceptions();
        _neighborhoodGenerator.precomputeCompositePerceptions();
        _neighborhoodGenerator.precomputeCompositeActions();

        _hcState = HC_START_ITERATION;
    }

private:
    const FE& _fitnessEstimator;

    int _fitnessEstimationPerCycle;

#ifdef IS_HC_CACHE
    lru_cache<FE> _estimator_cache;
#endif

    //Attributes
    //current program represent the current best program estimated during
    //the current iteration
    fitness_t _current_fitness; //real fitness of current program
    fitness_t _current_fitness_estimated;//estimated fitness of current program
    combo_tree _current_program;
    fitness_t _best_fitness;
    combo_tree _best_program;
    fitness_t _best_fitness_estimated;
    combo_tree _best_program_estimated;

    combo_tree _center; //contains the seed of the neighborhood

    NeighborhoodGenerator<Combo_TreeComp> _neighborhoodGenerator;
    neighborhood _neighborhood;

    ordered_neighborhood _ordered_best_estimates; //_ordered_neighborhood;

    HCState _hcState;

    int _number_of_fitness;

    bool _new_exemplar_initializes_center;

    combo_tree_hash_set _used_as_center;
    combo_tree_hash_set _used_for_owner;

    //ordered_neighborhood _ordered_center_set;

    //Methods

    //---------------------------------------------------------------------
    // fitness estimation of n candidates at most
    //---------------------------------------------------------------------

    void estimate_fitness_at_most(int n) {
        OC_ASSERT(n > 0);
        for (int i = 0; i < n && !_neighborhood.empty(); ++i) {
#ifdef COUNT_NUMBER_OF_FITNESS
            _number_of_fitness++;
#endif
            neighborhood_it tmp_it = _neighborhood.begin();
#ifdef IS_HC_CACHE
            std::pair<fitness_t, combo_tree> p(_estimator_cache(*tmp_it), *tmp_it);
#else
            std::pair<fitness_t, combo_tree> p(_fitnessEstimator(*tmp_it), *tmp_it);
#endif

            _ordered_best_estimates.insert(p);
            _neighborhood.erase(tmp_it);
        }
        if (_neighborhood.empty()) {
            _used_as_center.insert(_center);
        }
#ifdef COUNT_NUMBER_OF_FITNESS
        //debug log
        logger().debug("hillclimber - Total number of fitness estimations : %d", _number_of_fitness);
        logger().debug("hillclimber - Total number of non-cached estimations : %d", _estimator_cache.get_number_of_evaluations());
        //~debug log
#endif
    }

    //---------------------------------------------------------------------
    // Populate methods
    //---------------------------------------------------------------------

    void populate_neighborhood_from_center() {
        combo_tree center;
        if (_center.empty())
            center.set_head(id::sequential_and);
        else center = _center;
        pre_it center_head = center.begin();
        _neighborhoodGenerator.populate_neighborhood(_neighborhood,
                center, center_head);
    }

    //--------------------------------------------------------------------
    // other useful methods
    //--------------------------------------------------------------------

    bool has_been_used_as_center(const combo_tree& tr) const {
        return _used_as_center.find(tr) != _used_as_center.end();
    }

    bool has_been_sent_to_user(const combo_tree& tr) const {
        return _used_for_owner.find(tr) != _used_for_owner.end();
    }

    //return the nth+1 the best (fitness, cadidate)
    ordered_neighborhood_it nth_best_candidate(int n) {
        OC_ASSERT(n >= 0 && n < (int)_ordered_best_estimates.size());
        ordered_neighborhood_it res = _ordered_best_estimates.begin();
        for (int i_co = 0; i_co < n; ++i_co)
            ++res;
        return res;
    }

    //return the best candidate so far that has not been sent yet for trial.
    //If there is no such candidate (all has been sent or no candidates
    //have been produced), then it returns the end interator
    ordered_neighborhood_it random_not_sent_best_program() {
        ordered_neighborhood_it oni = _ordered_best_estimates.end();
        if (!_ordered_best_estimates.empty()) {
            oni = random_best_candidate();
            while (!_ordered_best_estimates.empty()
                    && has_been_sent_to_user(oni->second)) {
                _ordered_best_estimates.erase(oni);
                if (!_ordered_best_estimates.empty())
                    oni = random_best_candidate();
            }
        }
        return oni;
    }

    //return randomly one of the best (fitness, cadidate)
    //from ordered_neighborhood_it
    ordered_neighborhood_it random_best_candidate() {
        ordered_neighborhood_it res = _ordered_best_estimates.begin();
        OC_ASSERT(!_ordered_best_estimates.empty(),
                "_ordered_best_estimates should not be empty, if it may indeed be  empty then it is a bug, ask Nil to fix it");
        fitness_t fit = res->first;
        //choose randomly among programs with the same fitness
        int plateau_size = _ordered_best_estimates.count(fit);
        if (plateau_size > 1) {
            int chosen_one = randGen().randint(plateau_size);
            for (int i_co = 0; i_co < chosen_one; ++i_co)
                ++res;
        }
        OC_ASSERT(fit == res->first);
        return res;
    }

    //choose the candidate with the best fitness or randomly normal distributed
    //that hasn't been used as center yet
    //return true if such center exists, false if not
    //this situation with no center can happen if for instance
    //the only actions are idempotent, sit or drop for instance
    //and nothing happen on the scene no condition exists
    //therefore the candidate space is finite
    bool choose_center(combo_tree& tr) {
#ifdef DETERMINISTIC_REEXPANSION
        ordered_neighborhood_const_it oni = _ordered_best_estimates.begin();
        while (has_been_used_as_center(oni->second)) {
            ++oni;
            if (oni == _ordered_best_estimates.end()) //that is all candidates
                //have been tried as center
                return false;
        }
        tr = oni->second;
        return true;
#else
        tr = choose_rand_tree_from_order_neighborhood();
        OC_ASSERT(false, "Not sure that piece of code is working well, if you see that assert tell Nil about it");
        return true;
#endif
    }

    //return a combo_tree randomly choosen from _order_neighborhood
    //the higher the fitness the higher the chance to be choosen
    const combo_tree& choose_rand_tree_from_order_neighborhood() {

        RandGen &rng = randGen();
        ordered_neighborhood_it oni;
        do {
            //generate 2 gaussian numbers with variance 1 and mean 0
            double x1, x2, w, y1, y2, y;
            do {
                x1 = 2.0 * rng.randdouble() - 1.0;
                x2 = 2.0 * rng.randdouble() - 1.0;
                w = x1 * x1 + x2 * x2;
            } while ( w >= 1.0 );

            w = sqrt( (-2.0 * log( w ) ) / w );
            y1 = x1 * w;
            y2 = x2 * w;

            y1 = std::abs(y1);
            y2 = std::abs(y2);

            //take the smallest (that is with higher fitness)
            //then divide by NARROWNESS get a narrower variance
            y = std::min(y1, y2) / NARROWNESS;

            int i = (int)((double)_ordered_best_estimates.size() * y);
            i = i % _ordered_best_estimates.size();
            OC_ASSERT(i >= 0 && i < (int)_ordered_best_estimates.size());
            oni = nth_best_candidate(i);
        } while (has_been_used_as_center(oni->second)); //to not use a center
        //that has
        //already been developed
        return oni->second;
    }

};

}// ~namespace opencog
}// ~namespace hillclimbing

#endif
