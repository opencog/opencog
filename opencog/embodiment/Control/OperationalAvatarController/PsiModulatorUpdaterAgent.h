/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.h
 *
 * @author Jinhua Chua <JinhuaChua@gmail.com>
 * @date   2011-11-22
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

#ifndef PSIMODULATORUPDATERAGENT_H
#define PSIMODULATORUPDATERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>

class PsiModulatorUpdaterAgentUTest;

namespace opencog { namespace oac {

/**
 * @class
 *
 * @brief MindAgent for updating modulators
 *
 * Controls the dynamic of the emotion of the agent (Perhaps how angry an
 * agent is. Note that "angry" should either correspond to a modulator, or,
 * more likely a configuration of modulators, but this is just an example).
 * 
 * The format of Modulator in AtomSpace is
 *
 * AtTimeLink
 *     TimeNode "timestamp"
 *     SimilarityLink (stv 1.0 1.0)
 *         NumberNode: "modulator_value"
 *         ExecutionOutputLink
 *             GroundedSchemaNode: xxxModulatorUpdater
 *             ListLink (empty)
 *
 * LatestLink
 *     AtTimeLink
 *         TimeNode "latestTimestamp"
 *         EvaluationLink (stv denotes the level of the modulator)
 *             PredicateNode "xxxModulator"
 *
*/
class PsiModulatorUpdaterAgent : public opencog::Agent
{
    friend class::PsiModulatorUpdaterAgentUTest; 

private:

    /**
     * Inner class for Modulator
     */
    class Modulator
    {
    public:

        Modulator(const std::string & modulatorName) : modulatorName(modulatorName)
        {};

        inline const std::string & getModulatorName() 
        {
            return this->modulatorName;
        }

        inline double getModulatorLevel()
        {
            return this->currentModulatorValue; 
        }
  
        /**
         * Update the Modulator Value
         *
         * @return return true if update successfully, otherwise false
         */
        bool runUpdater(AtomSpace & atomSpace);

        /**
         * Set the new modulator value to the AtomSpace
         *
         * @return return true if update successfully, otherwise false
         *
         * TODO: Only current (latest) modulator value is considered, also take previous modulator values in future
         *
         * @note This function woule create a new NumberNode and SimilarityLink to store the result, 
         *       and then time stamp the SimilarityLink.
         *
         *       Since OpenCog would forget (remove) those Nodes and Links gradually, 
         *       unless you create them to be permanent, don't worry about the overflow of memory. 
         *
         *       AtTimeLink
         *           TimeNode "timestamp"
         *           SimilarityLink (stv 1.0 1.0)
         *               NumberNode: "modulator_value"
         *               ExecutionOutputLink
         *                   GroundedSchemaNode: xxxModulatorUpdater
         *                   ListLink (empty)
         *
         *       LatestLink
         *           AtTimeLink
         *               TimeNode "latestTimestamp"
         *               EvaluationLink (stv denotes the level of the modulator)
         *                   PredicateNode "xxxModulator"
         */
        bool updateModulator(AtomSpace & atomSpace, const unsigned long timeStamp);

    private:        

        std::string modulatorName;    // The name of the Modulator
        double currentModulatorValue; // Store current (latest) value of Modulator

    };// class Modulator

    double pleasure; 

    unsigned long cycleCount;

    std::vector<Modulator> modulatorList;  // List of Modulators

#ifdef HAVE_ZMQ    
    std::string publishEndPoint; // Publish all the messages to this end point
    zmq::socket_t * publisher;   // ZeroMQ publisher

    /**
     * Publish updated modulator values via ZeroMQ
     */
    void publishUpdatedValue(Plaza & plaza, zmq::socket_t & publisher, const unsigned long timeStamp);
#endif    

    // Initialize modulatorList etc.
    void init();

    bool bInitialized; 

public:

    PsiModulatorUpdaterAgent(CogServer&);
    virtual ~PsiModulatorUpdaterAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PsiModulatorUpdaterAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    virtual void run();

    // After calling this function, the Agent will invoke its "init" method firstly 
    // in "run" function during its next cycle
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

} } // namespace opencog::oac

#endif
