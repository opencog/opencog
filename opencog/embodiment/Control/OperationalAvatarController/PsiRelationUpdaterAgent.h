/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRelationUpdaterAgent.h
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-02-23
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

#ifndef PSIRELATIONUPDATERAGENT_H
#define PSIRELATIONUPDATERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>

namespace OperationalAvatarController
{

/**
 * @class
 *
 * @brief MindAgent for updating relations between the entities in the environment and the pet
 *
 * @note Each (relation, entity, pet) triple in AtomSpace is represented as follows:
 *       
 *       EvaluationLink
 *           PredicateNode "relationName"
 *           ListLink
 *               entityHandle
 *               petHandle
 */
class PsiRelationUpdaterAgent : public opencog::Agent
{
private:

    unsigned long cycleCount;

    bool bInitialized;  // Indicate whether the Agent has been Initialized

    std::vector<std::string> entityList;  // The names of entities within SpaceMap
    std::vector<std::string> relationList; // The names of relations

    int totalRemainSteps;   // Total steps (inference resource) remain for all the (entity, relation) pair
    int singleEntityRelationMaxSteps; // Maximum Steps (inference resource) for
                                      // a single (entity, relation) pair

    Btr<BITNodeRoot> Bstate;  // Root node of back inference tree, used by PLN backward chainer

    /**
     * Initialize entity, relation lists etc.
     */
    void init(opencog::CogServer * server);

    /**
     * Set inference steps (totalRemainSteps and singleEntityRelationMaxSteps)
     */
    void allocInferSteps();

    /**
     * Update the truth value of EvaluationLink holding the relation between the entity and the pet via PLN
     *
     * @param  server
     * @param  relationEvaluationLink  EvaluationLink holding the relation between the entity and the pet
     * @param  steps  Inference resource
     */
    void updateEntityRelation(opencog::CogServer * server, Handle relationEvaluationLink, int & steps); 

    /**
     * Get the EvaluationLink holding the relation between the pet and the entity.
     * If failed to find the EvaluationLink, it will create one automatically and return the EvaluationLink.
     *
     * @note  
     *     EvaluationLink
     *         PredicateNode "relationName"
     *         ListLink
     *             petHandle
     *             entityHandle
     */
    Handle getRelationEvaluationLink(opencog::CogServer * server, 
		                     const std::string & relationName, 
				     Handle petHandle, 
                                     Handle entityHandle
				    ); 
    /**
     * Get the Handle to entity given its name (id)
     */
    Handle getEntityHandle(opencog::CogServer * server, const std::string & entityName);

public:

    PsiRelationUpdaterAgent();
    virtual ~PsiRelationUpdaterAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PsiRelationUpdaterAgent");
        return _ci;
    }

    /**
     *  Entry of the Agent, CogServer will invoke this function during its cycle
     */
    void run(opencog::CogServer * server);

     /**
      * After calling this function, the Agent will invoke its "init" method firstly 
      * in "run" function during its next cycle
      */
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

}  // namespace

#endif
