/*
 * TODO: Once PLN is ready and can process GroundedPredicateNode automatically, we should then use 
 *       'PsiRelationUpdaterByPLNAgent.h|cc' instead. Of course, some modification are needed :-)
 *
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRelationUpdaterAgent.h
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-03-07
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

namespace opencog { namespace oac {

/**
 * @class
 *
 * @brief MindAgent for updating relations between the entities in the environment and the pet
 *
 * @note Each (relation, pet, entity) triple in AtomSpace is represented as follows:
 *       
 *       EvaluationLink
 *           PredicateNode "relationName"
 *           ListLink
 *               petHandle
 *               entityHandle
 */
class PsiRelationUpdaterAgent : public opencog::Agent
{
private:

    /**
     * @class Novelty
     *
     * @brief Inner class that stores the information of an entity's novelty
     */
    class Novelty
    {
    public:

        Novelty(float initLevel) : noveltyLevel(initLevel), resetTime(time(NULL)) {
        }

        inline float getNoveltyLevel() const {
            return this->noveltyLevel;
        }

        /**
         * Check if the entity is novel
         *
         * @param noveltyThreshold
         */
        inline bool isNovel(float noveltyThreshold) const {
            return (this->noveltyLevel >= noveltyThreshold);
        }

        /**
         * Check if the novelty level can be reset to initial value
         *
         * @param resetThreshold 
         */
        inline bool canBeReset (float resetThreshold) const {
            return (this->noveltyLevel <= resetThreshold);
        }

        inline void resetNoveltyLevel(float initLevel) {
            this->noveltyLevel = initLevel; 
            this->resetTime = time(NULL);
        }

        /**
         * Update the novelty level of the entity
         *
         * @param initLevel    Initial novelty level
         * @param decayFactor  The larger, the novelty level decrease more quickly by time
         */
        inline void updateNoveltyLevel(float initLevel, float decayFactor) {
            long timeElapse = time(NULL) - this->resetTime;
            this->noveltyLevel = initLevel*exp(-decayFactor*timeElapse);
        }

    private:
        float noveltyLevel;  
        long resetTime;      // When the novelty level is reset to initial level

    }; // class Novelty

    unsigned long cycleCount;

    bool bInitialized;  // Indicate whether the Agent has been Initialized

    std::vector<Handle> instantRelationRules; // Psi Rules (ImplicatonLinks) about Relations, 
                                              // 'instant' here means they have NULL_ACTION   

    std::map<std::string, Novelty> entityNovelty; // Contains (entityId, novelty level) pairs

    float noveltyInitLevel;      // Initial value of novelty level
    float noveltyThreshold;      // Entity with novelty level greater than the threshold is novel
    float noveltyResetThreshold; // Entity with novelty level less than the threshold should be 
                                 // reset or removed
    float noveltyDecayFactor;    // Larger the value, the novelty level decays more quickly

    /**
     * Initialize instantRelationRules etc.
     */
    void init();

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
    Handle getRelationEvaluationLink(AtomSpace & atomSpace, 
                                     const std::string & relationName, 
                                     Handle petHandle, 
                                     Handle entityHandle
                                    ); 
    /**
     * Get the Handle to entity given its id
     */
    Handle getEntityHandle(const AtomSpace & atomSpace, const std::string & entityName);

    /**
     * Update novel levels of all the entities.  
     *
     * @param server
     *
     * @note Each entity associates with a novelty level decreasing by time
     *       If the novelty level is higher than PSI_NOVELTY_THRESHOLD, 
     *       the entity would be considered as novel.
     *       If the novelty level is below or equals to PSI_NOVELTY_RESET_THRESHOLD, 
     *       it would be reset to PSI_NOVELTY_INIT_VALUE next time
     *       the pet encounters the corresponding entity. 
     */
    void updateEntityNovelty(opencog::CogServer * server);

    void updateEntityRelation(AtomSpace & atomSpace, 
                              Handle petHandle, 
                              Procedure::ProcedureInterpreter & procedureInterpreter, 
                              const Procedure::ProcedureRepository & procedureRepository);

public:

    PsiRelationUpdaterAgent(CogServer&);
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
    virtual void run();

     /**
      * After calling this function, the Agent will invoke its "init" method firstly 
      * in "run" function during its next cycle
      */
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

} } // namespace opencog::oac

#endif
