/*
 * opencog/embodiment/Learning/PetaverseMOSES/moses-learning.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Predrag Janicic
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
#include <ctime>

#include <opencog/comboreduct/reduct/reduct.h>

#include <opencog/learning/moses/moses/optimization.h>
#include <opencog/learning/moses/moses/scoring.h>

#include <opencog/embodiment/Learning/RewritingRules/RewritingRules.h>

#include "moses-learning.h"

namespace opencog { namespace moses {

typedef std::set<combo::vertex> operator_set;
typedef operator_set::iterator operator_set_it;

typedef std::set<combo::combo_tree, size_tree_order<combo::vertex> >
combo_tree_ns_set;
typedef combo_tree_ns_set::iterator combo_tree_ns_set_it;


using namespace reduct;

//constructor, generate/filter conditions and actions
moses_learning::moses_learning(int nepc,
                               const FE& fitness_estimator,
                               const definite_object_set& dos,
                               const operator_set& ignore_ops,
                               const combo_tree_ns_set& perceptions,
                               const combo_tree_ns_set& actions,
                               RandGen& rng)
        : _fitness_estimator(fitness_estimator), _comp(dos), _dos(dos), 
          _ignore_ops(ignore_ops),
          _perceptions(perceptions), _actions(actions), 
          _rng(rng), max_score(0)
{

    _nepc = nepc;

    std::cout << "NEPC : " << _nepc << std::endl;

    _current_fitness = INIT_FITNESS;
    _current_fitness_estimated = MIN_FITNESS ;
    _best_fitness = MIN_FITNESS;
    _best_fitness_estimated = MIN_FITNESS;

    combo_tree t(id::sequential_and);
    _center = t;

    std::cout << "MOSES LEARNING; ACTION POINTER : " << (long)&_actions
              << std::endl;

    for (combo_tree_ns_set_it i = actions.begin(); i != actions.end(); ++i)
        std::cout << "action: " << *i << endl;

    score = new petaverse_score(_fitness_estimator);
    bscore = new petaverse_bscore(_fitness_estimator);

    metapop = NULL;

    _hcState = HC_INIT;

    _number_of_calls = 0;

}


moses_learning::~moses_learning()
{
    delete score;
    delete bscore;
    if (metapop)
        delete metapop;
}



void moses_learning::operator()()
{
    std::cout << " ########################### OPERATOR : CASE: " << _hcState << std::endl;
    using namespace reduct;
    int max_for_generation;

    switch (_hcState) {
    case HC_IDLE:

        sleep(1);
        break;
        // ------------------------------------------------------------------------

    case HC_REWARD:

        if (_current_fitness >= _best_fitness) {
            _best_fitness = _current_fitness;
            _best_program = _current_program;
        }

        _hcState = HC_START_ITERATION;
        //if there is no more center available
        //then the entire candidate space
        //has been explored
        //and hillclimbing goes in idle state
        //  _hcState = HC_IDLE;
        break;
        // ------------------------------------------------------------------------


    case HC_INIT:
        _hcState = HC_START_ITERATION;

        break;
        // ------------------------------------------------------------------------

    case HC_START_ITERATION:  {

        std::cout << "START_ITERATION" << std::endl;
        _current_fitness_estimated = MIN_FITNESS;

        type_tree tt(id::lambda_type);
        tt.append_children(tt.begin(), id::action_result_type, 1);

        if (metapop)
            delete metapop;

        metapop = new metapopulation<petaverse_score, petaverse_bscore,
                                     sliced_iterative_hillclimbing>
            (_rng, _center, tt, action_reduction(),
             *score, *bscore, sliced_iterative_hillclimbing(_rng));

        _hcState = HC_BUILD_CANDIDATES;
        break;
    }
    // ------------------------------------------------------------------------

    case HC_BUILD_CANDIDATES:  {

        std::cout << "BUILD" << std::endl;
        if (metapop->create_deme(_ignore_ops, &_perceptions, &_actions))
            _hcState = HC_ESTIMATE_CANDIDATES;
        else
            _hcState = HC_IDLE;

        break;
    }
    // ------------------------------------------------------------------------

    case HC_ESTIMATE_CANDIDATES:  {

        std::cout << "ESTIMATE: " << std::endl;
        max_for_generation = _nepc;
        std::cout << "NEPC : " << _nepc << std::endl;
        std::cout << "MFG : " << max_for_generation << std::endl;

        int o = metapop->optimize_deme(max_for_generation - metapop->n_evals(),
                                       _nepc, max_score);
        std::cout << "opt returned : " << o << std::endl;

        if (o < 0)
            _hcState = HC_FINISH_CANDIDATES;

        //print the generation number and a best solution
//          std::cout << "EST sampled " << metapop->n_evals()
        //                  << " best " << metapop->best_score().first << " "
        //     << metapop->best_trees().front() << std::endl;

        break;
    }
    // ------------------------------------------------------------------------

    case HC_FINISH_CANDIDATES:  {

        metapop->close_deme();

        //print the generation number and a best solution
        std::cout << "sampled " << metapop->n_evals()
                  << " best " << metapop->best_score() << " "
                  << metapop->best_tree() << std::endl;

        std::cout << "sampled " << metapop->n_evals() << std::endl;;

        if (metapop->best_score() >= _best_fitness_estimated) {
            _best_fitness_estimated = metapop->best_score();
            _best_program_estimated = metapop->best_tree();
        }

        std::cout << "best program in this iter: " << metapop->best_tree() << std::endl;
        std::cout << "best score for this prog: " << metapop->best_score() << std::endl;

        std::cout << "best program total: " << _best_program_estimated << std::endl;
        std::cout << "best score total: " << _best_fitness_estimated << std::endl;

        metapop_t::const_iterator exemplar = metapop->select_exemplar();
        if(exemplar == metapop->end())
            _hcState = HC_IDLE;
        else {
            _center = exemplar->first;
            _hcState = HC_INIT;
        }

        // sleep(20);
        break;
    }
    // ------------------------------------------------------------------------


    default:
        std::cout << "OPERATOR : UNKNOWN CASE: " << _hcState << std::endl;
        break;
    }
}



const combo_tree& moses_learning::best_program()
{
    reduct::post_learning_rewrite(_best_program);
    std::cout << "########################### LS BEST PROGRAM : " << _best_program << std::endl;
    return _best_program;
}


const combo_tree& moses_learning::best_program_estimated()
{
    reduct::post_learning_rewrite(_best_program_estimated);
    std::cout << "########################### LS BEST PROGRAM EST: " << _best_program_estimated << std::endl;
    return _best_program_estimated;
}


const combo_tree& moses_learning::current_program()
{
    //returns the best program which has never been sent to the owner
    for(metapop_t::const_iterator mci = metapop->begin();
        mci != metapop->end(); ++mci)  {
        _current_program = get_tree(*mci);

        // if this one has already been sent, check the next one
        if(_used_for_owner.find(_current_program) != _used_for_owner.end())
            continue;

        _used_for_owner.insert(_current_program);

        fitness_t cur_est_fit = get_score(*mci);

        //DEBUG INFO
        std::cout << "CURRENT FITNESS : " << cur_est_fit << std::endl;
        std::cout << "CURRENT PROGRAM : " << _current_program << std::endl;
        //~DEBUG INFO
        
        if (cur_est_fit >= _best_fitness_estimated) {
            _best_fitness_estimated = cur_est_fit;
            _best_program_estimated = _current_program;
            //std::cout << "BEST PRO EST : " << _best_program_estimated
            //    << " FIT EST : " << _best_fitness_estimated << std::endl;
        }

        reduct::post_learning_rewrite(_current_program);
        std::cout << "########################### LS PROGRAM : " << _current_program << std::endl;
        return _current_program;
    }
    return _current_program;
}


void moses_learning::set_current_fitness(fitness_t f)
{
    std::cout << "########################### LS FITNESS : " << f << std::endl;

    _current_fitness = f;
    _hcState = HC_REWARD;
    if (_current_fitness >= _best_fitness) {
        _best_fitness = _current_fitness;
        _best_program = _current_program;
    }
//      _neighborhood.clear();
}


void moses_learning::reset_estimator()
{
    std::cout << "########################### LS RESET : " << _best_program << std::endl;

    delete score;
    delete bscore;

    score = new petaverse_score(_fitness_estimator);
    bscore = new petaverse_bscore(_fitness_estimator);

    //we get restarted from the best program so far
    _center = (_best_program.empty() ? _best_program_estimated : _best_program);

    _best_fitness_estimated = MIN_FITNESS;
    _best_program_estimated.clear();

    _hcState = HC_START_ITERATION;
}

} // ~namespace moses
} // ~namespace opencog
