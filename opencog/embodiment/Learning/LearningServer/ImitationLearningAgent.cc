/*
 * opencog/embodiment/Learning/LearningServer/ImitationLearningAgent.cc
 *
 * Copyright (C) 2007-2008 Nil Geisweiller
 * All Rights Reserved
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
#include <iostream>

#include "util/exceptions.h"
#include "util/mt19937ar.h"

#include "ActionFilter.h"
#include "LoggerFactory.h"
#include "NetworkElement.h"
#include "EntityRelevanceFilter.h"
#include "moses-learning.h"

#include "ImitationLearningAgent.h"

namespace MessagingSystem
{

using namespace opencog;

//constructor destructor
ImitationLearningAgent::ImitationLearningAgent() : _lts(LTS_IDLE),
        _entropyFilter(NULL),
        _fitnessEstimator(NULL),
        _PIL(NULL), _PVoc(NULL)
{
    //initialize the random generator
    unsigned long rand_seed;
    if (NetworkElement::parameters.get("AUTOMATED_SYSTEM_TESTS") == "1") {
        rand_seed = 0;
    } else {
        rand_seed = time(NULL);
    }
    _rng = new opencog::MT19937RandGen(rand_seed);
    logger().log(opencog::Logger::INFO, "ImitationLearningAgent - Created random number generator (%p) for ImitationLearningAgent with seed %lu", _rng, rand_seed);

    //instanciate the learning vocabulary provider
    const std::string ILALGO = MessagingSystem::NetworkElement::parameters.get("IMITATION_LEARNING_ALGORITHM");
    if (ILALGO == Control::ImitationLearningAlgo::HillClimbing)
        _PVoc = new HCPetaverseVocabularyProvider();
    else if (ILALGO == Control::ImitationLearningAlgo::MOSES)
        _PVoc = new HCPetaverseVocabularyProvider();
    else opencog::cassert(TRACE_INFO, false, "A valid learning algo must be selected");
}

ImitationLearningAgent::~ImitationLearningAgent()
{
    delete _entropyFilter;
    delete _PIL;
    delete _PVoc;
    delete _rng;
}

//run method
void ImitationLearningAgent::run(CogServer* server)
{

    //std::cout << "IMITATION LEARNING AGENT RUN" << std::endl;

    switch (_lts) {
    case LTS_LEARN:
        opencog::cassert(TRACE_INFO, _PIL != NULL);
        (*_PIL)();
        break;

    case LTS_IDLE:
        // this prevents the idle LS to get 100% of cpu usage
        usleep(200000);
        break;

    default:
        // do nothing
        break;
    }
}

//command methods
bool ImitationLearningAgent::initLearning(int nepc,
                                         WorldProvider* wp,
                                         const argument_list& al,
                                         const string& pet_id,
                                         const string& owner_id,
                                         const string& avatar_id,
                                         const string& trick_name)
{
    opencog::cassert(TRACE_INFO, wp, "The World Provider points to NULL");
    //retreive exemplars
    _BDCat.clear();
    _exemplarTemporals.clear();
    _all.clear();

    //the following line is commented so that when learning is initiated
    //with a trick already previously learned the exemplars
    //of the previous learning session are ignored
    //BDRetriever::retrieveAllExemplars(_BDCat, _exemplarTemporals,
    //*wp, trick_name);
    BDRetriever::addLastExemplar(_BDCat, _exemplarTemporals, *wp, trick_name);

    if (_BDCat.empty()) {
        logger().log(opencog::Logger::INFO, "ImitationLearningAgent - All exemplars are empty, learning will not start.");
        return false; //indicates that initLEarning has failed
    } else {
        _pet_id = pet_id;
        _owner_id = owner_id;
        _avatar_id = avatar_id;
        _trick_name = trick_name;

        //for now it is assumed that _BDCat constins only one BD
        unsigned int bdcs = _BDCat.getSize();
        opencog::cassert(TRACE_INFO, bdcs == 1,
                         "For now it is assumed that _BDCat contains only one BD");

        _arity = al.size();
        _all.push_back(al);

        opencog::cassert(TRACE_INFO,
                         bdcs == _all.size() && bdcs == _exemplarTemporals.size(),
                         "The number of behavior description, argument lists and exemplar temporals must be equal");

        //----------------------------------------
        //select the definite objects and messages
        //----------------------------------------
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - Getting definite objects from SpaceMap.");
        EntityRelevanceFilter filter;
        opencog::cassert(TRACE_INFO, _definite_objects.empty(),
                         "_definite_objects must be empty because it should have been cleared by stopLearning");
        //the filter transform owner id and avatar to imitate id into
        //"owner" and "self" respectively that's because the pet
        //should take the viewpoint of the avatar to imitate
        _definite_objects = filter.getEntities(*wp, _trick_name,
                                               _avatar_id, _owner_id);

        //the definite object corresponding to the pet is simply deleted to
        //not have the pet "petID" and the imaginary pet "self" in the same scene
        //although that's the current assumption it can change in the future...
        _definite_objects.erase(_pet_id);

        //debug log
        if (opencog::Logger::DEBUG <= logger().getLevel()) {
            logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of the definite objects used for learning");
            for (std::set<std::string>::iterator si = _definite_objects.begin();
                    si != _definite_objects.end(); ++si) {
                logger().log(opencog::Logger::DEBUG,
                             "ImitationLearningAgent - %s", si->c_str());
            }
        }
        //~debug log

        //get the set of messages
        _messages = filter.getMessages(*wp, _trick_name);

        //debug log
        if (opencog::Logger::DEBUG <= logger().getLevel()) {
            logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of the messages used for learning");
            for (message_set_const_it mi = _messages.begin();
                    mi != _messages.end(); ++mi) {
                logger().log(opencog::Logger::DEBUG,
                             "ImitationLearningAgent - %s",
                             mi->getContent().c_str());
            }
        }
        //~debug log

        //get the set of other agents' actions
        std::set<string> exclude_set;
        exclude_set.insert(_avatar_id);
        exclude_set.insert(_pet_id);
        _atas = filter.getAgentActions(*wp, _trick_name,
                                       _avatar_id, _owner_id,
                                       exclude_set);

        //debug log
        if (opencog::Logger::DEBUG <= logger().getLevel()) {
            logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of the agent and action definite objects used for learning");
            for (agent_to_actions_const_it atas_it = _atas.begin();
                    atas_it != _atas.end(); ++atas_it) {
                logger().log(opencog::Logger::DEBUG,
                             "ImitationLearningAgent - Agent : %s",
                             atas_it->first.c_str());
                const definite_object_vec_set& ados = atas_it->second;
                for (definite_object_vec_set_const_it ados_it = ados.begin();
                        ados_it != ados.end(); ++ados_it) {
                    std::string s;
                    for (definite_object_vec_const_it dci = ados_it->begin();
                            dci != ados_it->end(); ++dci)
                        s += *dci + std::string(" ");
                    logger().log(opencog::Logger::DEBUG,
                                 "ImitationLearningAgent - Actions and arguments : %s",
                                 s.c_str());
                }
            }
        }
        //~debug log

        //-----------------------------------------
        //generate filtered perceptions and actions
        //-----------------------------------------
        opencog::cassert(TRACE_INFO, _entropyFilter == NULL,
                         "_entropyFilter must be NULL because it should have been deleted by stopLearning");
        //get petaverse elementary actions, perceptions and indefinite objects
        //and elementary operators
        const operator_set& eo = _PVoc->get_elementary_operators();
        const builtin_action_set& ea = _PVoc->get_elementary_actions();
        const perception_set& ep = _PVoc->get_elementary_perceptions();
        const indefinite_object_set& io = _PVoc->get_indefinite_objects();
        //compute and filter atomic perceptions
        //self_id is avatar_id because the pet must take the view point
        //of the avatar to imitate

        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - EntropyFilter, start processing.");

        //for now it is assumed that all arguments of the program to learn
        //are definite_object
        argument_type_list atl(_arity, type_tree(id::definite_object_type));

        _entropyFilter = new EntropyFilter(_avatar_id, _owner_id,
                                           wp->getSpaceServer(),
                                           ep, io, _definite_objects, _messages,
                                           _atas,
                                           atl,
                                           *_rng);
        opencog::cassert(TRACE_INFO, _atomic_perceptions.empty(),
                         "_atomic_perceptions must be empty because stopLearning would have clear it");

        _entropyFilter->generateFilteredPerceptions(_atomic_perceptions, atoi(MessagingSystem::NetworkElement::parameters.get("ENTROPY_PERCEPTION_FILTER_THRESHOLD").c_str()), _BDCat, _exemplarTemporals, _all);

        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - EntropyFilter, stop processing.");

        //debug log
        if (opencog::Logger::DEBUG <= logger().getLevel()) {
            logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of perceptions filtered by EntropyFilter");
            for (combo_tree_ns_set_const_it pi = _atomic_perceptions.begin();
                    pi != _atomic_perceptions.end(); ++pi) {
                stringstream ss;
                ss << *pi;
                logger().log(opencog::Logger::DEBUG,
                             "ImitationLearningAgent - %s", ss.str().c_str());
            }
        }
        //~debug log

        //compute and filter actions
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - ActionFilter, start processing.");

        bool type_check_actions =
            NetworkElement::parameters.get("TYPE_CHECK_GENERATED_COMBO") == "1";

        Filter::ActionFilter af(_avatar_id, _owner_id, *wp, _definite_objects,
                                io, ea, _arity, type_check_actions, *_rng);

        opencog::cassert(TRACE_INFO, _atomic_actions.empty(),
                         "_atomic_actions must be empty because stopLearning would have clear it");
        af.insertActionSubseqs(_atomic_actions, _BDCat, _all,
                               atoi(MessagingSystem::NetworkElement::parameters.get("ACTION_FILTER_SEQ_MAX").c_str()));

        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - ActionFilter, stop processing.");

        if (_atomic_actions.size() > 0) {

            //debug log
            if (opencog::Logger::DEBUG <= logger().getLevel()) {
                logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of actions filtered by ActionFilter");
                for (combo_tree_ns_set_const_it ai = _atomic_actions.begin();
                        ai != _atomic_actions.end(); ++ai) {
                    stringstream ss;
                    ss << *ai;
                    logger().log(opencog::Logger::DEBUG,
                                 "ImitationLearningAgent - %s", ss.str().c_str());
                }
            }
            //~debug log

        } else {
            logger().log(opencog::Logger::DEBUG,
                         "ImitationLearningAgent - No action filtered by ActionFilter, learning will not start");
            return false;
        }

        //--------------------------------
        //instanciate the fitness function
        //--------------------------------
        opencog::cassert(TRACE_INFO, _fitnessEstimator == NULL,
                         "_fitnessEstimator must be NULL because it should have been deleted by stopLearning");
        _fitnessEstimator =
            new FE(wp, _pet_id, _owner_id, _avatar_id, _trick_name,
                   _definite_objects, _BDCat, _exemplarTemporals, _all, io.size(),
                   eo.size(),
                   _atomic_perceptions.size(), _atomic_actions.size(), *_rng);

        //----------------------------------
        //instanciate petaverse-hillclimbing
        //----------------------------------
        opencog::cassert(TRACE_INFO, _PIL == NULL,
                         "PIL must be NULL because it should have been deleted by stopLearning");

        //debug log for SPCTools
        logger().log(opencog::Logger::DEBUG,
                     "ImitationLearningAgent - SPCTools - New estimator");
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - definite object count : %d", _definite_objects.size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - indefinite object count : %d", io.size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - elementary operator count : %d", eo.size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - atomic perception count : %d", _atomic_perceptions.size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - atomic action count : %d", _atomic_actions.size());
        //~debug log for SPCTools

        //instanciate the learning algo
        const std::string ILALGO = MessagingSystem::NetworkElement::parameters.get("IMITATION_LEARNING_ALGORITHM");
        if (ILALGO == Control::ImitationLearningAlgo::HillClimbing) {
            bool abibb = MessagingSystem::NetworkElement::parameters.get("ACTION_BOOLEAN_IF_BOTH_BRANCHES_HC_EXPENSION") == "1";
            bool neic = MessagingSystem::NetworkElement::parameters.get("HC_NEW_EXEMPLAR_INITIALIZES_CENTER") == "1";
            _PIL = new petaverse_hillclimber(nepc, *_fitnessEstimator,
                                             _definite_objects, eo,
                                             _atomic_perceptions,
                                             _atomic_actions,
                                             abibb, neic,
                                             *_rng);
        } else if (ILALGO == Control::ImitationLearningAlgo::MOSES) {
            _PIL = new moses::moses_learning(nepc, *_fitnessEstimator,
                                             _definite_objects, eo,
                                             _atomic_perceptions,
                                             _atomic_actions, *_rng);
        } else opencog::cassert(TRACE_INFO, false,
                                    "A valid learning algo must be selected, see src/Control/SystemParameters.h for the various options");

        //initLearning succeeds
        _lts = LTS_LEARN;
        return true;
    }
}

void ImitationLearningAgent::addLearningExample(WorldProvider* wp,
                                               const argument_list& al)
{
    opencog::cassert(TRACE_INFO, wp, "The World Provider points to NULL");
    //add new exemplar
    int cat_count = _BDCat.getSize();
    BDRetriever::addLastExemplar(_BDCat, _exemplarTemporals, *wp, _trick_name);
    if (cat_count != _BDCat.getSize()) { //a new exemplar has been added

        opencog::cassert(TRACE_INFO, (int)al.size() == _arity,
                         "For now the arity must be the same for all exemplars");
        _all.push_back(al);

        //insert the new objects in _definite_objects
        //probablement not needed because dos would contain all objects

        //----------------------------------------------------
        //select and add the new definite objects and messages
        //----------------------------------------------------
        EntityRelevanceFilter filter;
        //the filter transform owner id and avatar to imitate id into
        //"owner" and "self" respectively that's because the pet
        //should take the viewpoint of the avatar to imitate
        _definite_objects = filter.getEntities(*wp, _trick_name,
                                               _avatar_id, _owner_id);

        //the definite object corresponding to the pet is simply deleted to
        //not have the pet "petID" and the imaginary pet "self" in the same scene
        //although that's the current assumption it can change in the future...
        _definite_objects.erase(_pet_id);

        //debug log
        if (opencog::Logger::DEBUG <= logger().getLevel()) {
            logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of definite objects used for learning");
            for (std::set<std::string>::const_iterator si =
                        _definite_objects.begin(); si != _definite_objects.end(); ++si) {
                logger().log(opencog::Logger::DEBUG,
                             "ImitationLearningAgent - %s", si->c_str());
            }
        }
        //~debug log

        logger().log(opencog::Logger::DEBUG,
                     "ImitationLearningAgent - Add Learning Example.");

        //get the set of messages
        _messages = filter.getMessages(*wp, _trick_name);
        //get the set of other agents' actions
        std::set<string> exclude_set;
        exclude_set.insert(_avatar_id);
        exclude_set.insert(_pet_id);
        _atas = filter.getAgentActions(*wp, _trick_name,
                                       _avatar_id, _owner_id,
                                       exclude_set);

        //-------------------------------------------
        //regenerate filtered perceptions and actions
        //-------------------------------------------

        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - EntropyFilter, start update processing.");

        _atomic_perceptions.clear();
        _entropyFilter->generateFilteredPerceptions(_atomic_perceptions, atoi(MessagingSystem::NetworkElement::parameters.get("ENTROPY_PERCEPTION_FILTER_THRESHOLD").c_str()), _BDCat.getEntries().back(), _exemplarTemporals.back(), al);

        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - EntropyFilter, stop update processing.");

        //debug log
        if (opencog::Logger::DEBUG <= logger().getLevel()) {
            logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of perceptions filtered by EntropyFilter");
            for (combo_tree_ns_set_const_it pi = _atomic_perceptions.begin();
                    pi != _atomic_perceptions.end(); ++pi) {
                stringstream ss;
                ss << *pi;
                logger().log(opencog::Logger::DEBUG,
                             "ImitationLearningAgent - %s", ss.str().c_str());
            }
        }
        //~debug log

        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - ActionFilter, start processing.");

        bool type_check_actions =
            NetworkElement::parameters.get("TYPE_CHECK_GENERATED_COMBO") == "1";

        Filter::ActionFilter af(_avatar_id, _owner_id, *wp, _definite_objects,
                                _PVoc->get_indefinite_objects(),
                                _PVoc->get_elementary_actions(),
                                _arity, type_check_actions,
                                *_rng);

        _atomic_actions.clear();
        af.insertActionSubseqs(_atomic_actions, _BDCat, _all,
                               atoi(MessagingSystem::NetworkElement::parameters.get("ACTION_FILTER_SEQ_MAX").c_str()));

        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - ActionFilter, stop processing.");

        //debug log
        if (opencog::Logger::DEBUG <= logger().getLevel()) {
            logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - List of actions filtered by ActionFilter");
            for (combo_tree_ns_set_const_it ai = _atomic_actions.begin();
                    ai != _atomic_actions.end(); ++ai) {
                stringstream ss;
                ss << *ai;
                logger().log(opencog::Logger::DEBUG,
                             "ImitationLearningAgent - %s", ss.str().c_str());
            }
        }
        //~debug log

        //-----------------------------------------
        //update fitness estimator and hillclimbing
        //-----------------------------------------

        //debug log for SPCTools
        logger().log(opencog::Logger::DEBUG,
                     "ImitationLearningAgent - SPCTools - New estimator");
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - definite object count : %d", _definite_objects.size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - indefinite object count : %d", _PVoc->get_indefinite_objects().size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - elementary operator count : %d", _PVoc->get_elementary_operators().size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - atomic perception count : %d", _atomic_perceptions.size());
        logger().log(opencog::Logger::DEBUG, "ImitationLearningAgent - SPCTools - atomic action count : %d", _atomic_actions.size());
        //~debug log for SPCTools

        //this is needed because when new exemplars arrive the fitness estimation
        //is different
        //update fitness estimator
        _fitnessEstimator->update(_PVoc->get_indefinite_objects().size(),
                                  _PVoc->get_elementary_operators().size(),
                                  _atomic_perceptions.size(),
                                  _atomic_actions.size());
        _PIL->reset_estimator();
    }
    _lts = LTS_LEARN; //even if no exemplar are added the algo keep searching
}

void ImitationLearningAgent::waitForReward()
{
    _lts = LTS_IDLE;
}

void ImitationLearningAgent::setFitness(fitness_t f)
{
    opencog::cassert(TRACE_INFO, _PIL != NULL);
    _PIL->set_current_fitness(f);
    _lts = LTS_LEARN;
}

void ImitationLearningAgent::stopLearning()
{
    opencog::cassert(TRACE_INFO, _PIL != NULL);

    _definite_objects.clear();
    _messages.clear();
    _BDCat.clear();
    _atomic_perceptions.clear();
    _atomic_actions.clear();

    _lts = LTS_IDLE;

    delete(_entropyFilter);
    _entropyFilter = NULL;
    delete(_fitnessEstimator);
    _fitnessEstimator = NULL;
    delete(_PIL);
    _PIL = NULL;
}

//return the best schema learned so far
const combo_tree& ImitationLearningAgent::getBestSchema()
{
    opencog::cassert(TRACE_INFO, _PIL != NULL);
    //if the bestSchema is empty, it means that the user didn't provide
    //any reward
    //then the best schema estimated is returned
    const combo_tree& tr = _PIL->best_program();
    if (tr.empty())
        return _PIL->best_program_estimated();
    else return tr;
}

//return the current best schema estimated so far since
//the iteration has started
const combo_tree& ImitationLearningAgent::getBestSchemaEstimated()
{
    opencog::cassert(TRACE_INFO, _PIL != NULL);
    const combo_tree& cp = _PIL->current_program();
    return cp;
}

}//~MessagingSystem
