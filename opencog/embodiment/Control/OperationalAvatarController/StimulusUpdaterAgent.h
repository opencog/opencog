/*
 * @file opencog/embodiment/Control/OperationalAvatarController/StimulusUpdaterAgent.h
 *
 * @author Troy Huang <huangdeheng@gmail.com>
 * @date   2011-12-19
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

#ifndef STIMULUS_UPDATER_AGENT_H
#define STIMULUS_UPDATER_AGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>

// TODO:
//class StimulusUpdaterAgentUTest;

namespace opencog { namespace oac {

/**
 * @class
 *
 * @brief MindAgent for updating external stimulus for modulators
 *
 * Affect the dynamic of the modulator level by adding the stimulus 
 * coming from external environment. (e.g. When punched by some one, 
 * the stimulus might amplify the emotion of anger, which is controlled
 * by ActivationModulator, ResolutionModulator and 
 * SelectionThresholdModulator)
 *
 * The format of stimulus in AtomSpace is
 *
 * AtTimeLink
 *     TimeNode "timestamp"
 *     SimilarityLink (stv 1.0 1.0)
 *         NumberNode: "stimulus_value"
 *         ExecutionOutputLink
 *             GroundedSchemaNode: xxxStimulusUpdater
 *             ListLink (empty)
 *
 * LatestLink
 *     AtTimeLink
 *         TimeNode "latestTimestamp"
 *         EvaluationLink (stv denotes the level of the stimulus)
 *             PredicateNode "xxxStimulus"
 *
*/
class StimulusUpdaterAgent : public opencog::Agent
{
    //friend class::StimulusUpdaterAgentUTest; 

private:

    /**
     * Inner class for Stimulus
     */
    class Stimulus
    {
    public:

        Stimulus(const std::string & stimulusName) : stimulusName(stimulusName)
        {};

        inline const std::string & getStimulusName() 
        {
            return this->stimulusName;
        }

        inline double getStimulusLevel()
        {
            return this->currentStimulusValue; 
        }
  
        /**
         * Update the stimulus value
         *
         * @return return true if update successfully, otherwise false
         */
        bool runUpdater(AtomSpace & atomSpace);

        /**
         * Set the new stimulus value to the AtomSpace
         *
         * @return return true if update successfully, otherwise false
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
         *               NumberNode: "stimulus_value"
         *               ExecutionOutputLink
         *                   GroundedSchemaNode: xxxStimulusUpdater
         *                   ListLink (empty)
         *
         *       LatestLink
         *           AtTimeLink
         *               TimeNode "latestTimestamp"
         *               EvaluationLink (stv denotes the level of the modulator)
         *                   PredicateNode "xxxStimulus"
         */
        bool updateStimulus(AtomSpace & atomSpace, const unsigned long timeStamp);
        
        /**
         * The original value range of stimulus should be [-1, 1].
         * But as there is no negative value for truth value, we scale the
         * interval to [0, 1]. Thus the initial mean for truth value is 0.5
         */
        void initStimulus(AtomSpace & atomSpace, const unsigned long timeStamp);

    private:        

        std::string stimulusName;    // The name of the Stimulus
        double currentStimulusValue; // Store current (latest) value of Stimulus

    };// class Modulator

    unsigned long cycleCount;

    unsigned long latestStimulusTimestamp;

    std::vector<Stimulus> stimulusList;  // List of Modulators

#ifdef HAVE_ZMQ    
    std::string publishEndPoint; // Publish all the messages to this end point
    zmq::socket_t * publisher;   // ZeroMQ publisher

    /**
     * Publish updated stimulus values via ZeroMQ
     */
    void publishUpdatedValue(Plaza & plaza, zmq::socket_t & publisher, const unsigned long timeStamp);
#endif    

    // Initialize stimulusList etc.
    void init();

    bool bInitialized; 

public:

    StimulusUpdaterAgent(CogServer&);
    virtual ~StimulusUpdaterAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::StimulusUpdaterAgent");
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

typedef std::shared_ptr<StimulusUpdaterAgent> StimulusUpdaterAgentPtr;

} } // namespace opencog::oac

#endif
