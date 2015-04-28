/*
 * opencog/embodiment/Learning/FitnessEstimator/NoSpaceLifeFitnessEstimator.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#include "NoSpaceLifeFitnessEstimator.h"
#include <opencog/embodiment/WorldWrapper/NoSpaceLifeWorldWrapper.h>
#include <opencog/embodiment/Control/Procedure/ComboInterpreter.h>
#include <opencog/util/exceptions.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

//true if the timing is taken into account in the similarity measure
#define TIMING_PERTINANCE true

//the number of Monte Carlo simulation is number of definite objects
//times TRIAL_FACTOR
#define TRIAL_FACTOR 2

using namespace behavior;
using namespace Procedure;
using namespace AvatarCombo;
using namespace opencog;

namespace FitnessEstimator
{

/**
 * ctor, dtor
 */
NoSpaceLifeFitnessEstimator::NoSpaceLifeFitnessEstimator
(WorldProvider* wp, const std::string& petName, const std::string& ownerName,
 const std::string& avatarName, const std::string& trickName,
 const definite_object_set& dos, BehaviorCategory& BDCat,
 const std::vector<Temporal>& ets, const argument_list_list& all,
 int indefinite_object_count, int operator_count,
 int predicate_count, int action_count)
        : _wp(wp), _dos(dos), _petName(petName), _ownerName(ownerName),
        _avatarName(avatarName), _trickName(trickName),
        _BDCat(BDCat), _exemplarTemporals(ets), _all(all),
        _BDMatcher(&wp->getAtomSpace()),
        _sizePenalty(dos, indefinite_object_count, operator_count,
                     predicate_count, action_count)
#ifdef IS_FE_LRU_CACHE
        , _bd_cache(FE_LRU_CACHE_SIZE),
        _cache_success(0), _total_fitness_call(0)
#endif
{

    OC_ASSERT(BDCat.getSize() == (int)all.size(),
                     "There must be as many behavior category as argument lists");

    _randomOperatorOptimization = opencog::config().get_bool("RANDOM_OPERATOR_OPTIMIZATION");
    //if the random operator optimization is activated then
    //a plan is sent without evaluating indefinite objects
    _sendDefinitePlan = !_randomOperatorOptimization;
}

NoSpaceLifeFitnessEstimator::~NoSpaceLifeFitnessEstimator() {}

/**
 * operator
 */
fitness_t NoSpaceLifeFitnessEstimator::operator()(const combo::combo_tree& tr) const
{
    //debug log
    if (logger().isDebugEnabled()) {
        stringstream ss_tr;
        ss_tr << tr;
        string s_tr = ss_tr.str();
        opencog::logger().debug("NoSpaceLifeFitnessEstimator - Candidate being estimated : %s", s_tr.c_str());
    }
    //~debug log

    fitness_t score = 0; //score without size penalty
    //assess similarity of each composite BD obtained from combo executed
    //back into the context to each exemplar
    //then take the mean of their similarity
    const std::vector<CompositeBehaviorDescription>& bdce = _BDCat.getEntries();
    OC_ASSERT(!bdce.empty(), "Error : No exemplars");

    //debug log
    opencog::logger().debug("NoSpaceLifeFitnessEstimator - Loop over Behavior Category starts");
    //~debug log

    //determine the number of times to run the schema
    //for each behavior description
    int trial_count = 1;
    if (!_randomOperatorOptimization)
        trial_count = getTrialCount(tr); //in case the tree contains rands

#ifdef IS_FE_LRU_CACHE
    BDCache::map_iter mi = _bd_cache.find(tr);
    bool cache_failure = _bd_cache.is_cache_failure(mi);
    fitness_vec new_fv;
    fitness_vec& ref_fv = cache_failure ? new_fv : mi->second;
    fitness_vec_const_it fv_it = ref_fv.begin();
#endif

    std::vector<CompositeBehaviorDescription>::const_iterator
    cbd_it = bdce.begin();
    std::vector<Temporal>::const_iterator eti = _exemplarTemporals.begin();
    argument_list_list_const_it allci = _all.begin();
    for (; cbd_it != bdce.end(); ++cbd_it, ++allci, ++eti) {
        fitness_t bd_score = 0;
#ifdef IS_FE_LRU_CACHE
        if (fv_it == ref_fv.end()) {
#else
        {
#endif
            for (int i = 0; i < trial_count; i++) {
                //generate behavior description with NoSpaceLife
                world::NoSpaceLifeWorldWrapper nspww(_wp->getAtomSpace(),
                        _petName,
                        _ownerName,
                        _avatarName,
                        *cbd_it,
                        *eti);
                RunningComboProcedure rp(nspww, tr, *allci, _sendDefinitePlan);
                unsigned int cbd_size = cbd_it->size();
                for (unsigned int i = 0;
                        !rp.isFinished() &&
                        i < (cbd_size
                             + (unsigned int)(MAX_ADDITIONAL_CYCLE_COEF*(float)cbd_size));
                        i++) {

                    //debug log
                    logger().debug("NoSpaceLifeFitnessEstimator - Combo interpreter cycle : %d", i);
                    //~debug log

                    rp.cycle();
                }
                CompositeBehaviorDescription& genBD =
                    nspww.getNoSpaceLife().getGeneratedBD();
                //debug log
                if (logger().isDebugEnabled()) {
                    logger().debug("NoSpaceLifeFitnessEstimator - Composite Behavior Description to imitate : %s", cbd_it->toString().c_str());
                    logger().debug("NoSpaceLifeFitnessEstimator - Composite Behavior Description generated : %s", genBD.toString().c_str());
                }
                //~debug log

                bd_score = _BDMatcher.computePertinenceDegree(genBD,
                           *cbd_it,
                           TIMING_PERTINANCE);
            }
            bd_score /= trial_count;

            //debug log
            logger().debug("NoSpaceLifeFitnessEstimator - Score against the current Composite Behavior Description : %f", bd_score);
            //~debug log

#ifdef IS_FE_LRU_CACHE
            ref_fv.push_back(bd_score);
            fv_it = ref_fv.end();
        }
        else { //that bd_score has already been computed
            bd_score = *fv_it;
            ++fv_it;
#endif
        }
        score += bd_score; //non-normalized score
    }
    //debug log
    logger().debug("NoSpaceLifeFitnessEstimator - Loop over Behavior Category ends");
    //~debug log

#ifdef IS_FE_LRU_CACHE
    //if cache failure then insert ref_fv in the cache
    if (cache_failure)
        _bd_cache.insert_new(tr, ref_fv);
    else _cache_success++;
    _total_fitness_call++;

    //debug log
    logger().debug("NoSpaceLifeFitnessEstimator - Total fitness call = %u, Cache success = %u", _total_fitness_call, _cache_success);
    //~debug log
#endif

    //compute score
    score /= (float)(bdce.size()); //normalized score
    //debug log for SPCTools
    logger().debug("NoSpaceLifeFitnessEstimator - SPCTools - Score : %f", score);
    //~debug log for SPCTools

    //compute size penalty
    double sp = _sizePenalty.computeSizePenalty(tr);
    //compute fitness estimation
    double fit = score * sp;

    //debug log
    logger().debug("NoSpaceLifeFitnessEstimator - Size penalty : %f", sp);
    logger().debug("NoSpaceLifeFitnessEstimator - Fitness : %f", fit);
    //~debug log

    return fit;
}

/**
 * public methods
 */

void NoSpaceLifeFitnessEstimator::update(int indefinite_object_count,
        int operator_count,
        int condition_count,
        int action_count) {
    _sizePenalty.update(indefinite_object_count, operator_count,
                        condition_count, action_count);
    OC_ASSERT(_BDCat.getSize() == (int)_all.size(),
                     "There must be as many behavior category as argument lists");
}

/**
 * private methods
 */

int NoSpaceLifeFitnessEstimator::getTrialCount(const combo::combo_tree& tr) const {
    int res = 1;
    for (combo::combo_tree::iterator it = tr.begin(); it != tr.end(); ++it) {
        //TODO : improve the code so that it works for any random
        //also, the technic should certainly be different
        //checking whether it could be random rather than making Monte Carlos
        //simulation
        if (*it == get_instance(id::random_object))
            res *= _dos.size() * TRIAL_FACTOR;
        else if (*it == get_instance(id::random_step))
            res *= 4 * TRIAL_FACTOR;
    }
    return res;
}

}

