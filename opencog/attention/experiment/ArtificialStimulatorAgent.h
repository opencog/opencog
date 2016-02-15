/*
 * StimulusProviderAgent.h
 *
 *  Created on: 4 Aug, 2015
 *      Author: misgana
 */

#ifndef _ARTIFICIALSTIMULATORAGENT_H_
#define _ARTIFICIALSTIMULATORAGENT_H_

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{
/**
 * This agent stimulates set of atoms.
 */
class CogServer;
class ArtificialStimulatorAgent: public Agent {
private:
    AtomSpace * _as;
    const stim_t stimuli = 20;

public:
    ArtificialStimulatorAgent(CogServer&);
    virtual ~ArtificialStimulatorAgent();

    virtual const ClassInfo& classinfo() const
    {
        return info();
    }

    static const ClassInfo& info()
    {
        static const ClassInfo _ci("opencog::ArtificialStimulatorAgent");
        return _ci;
    }

    virtual void run();

};

} /* namespace opencog */

#endif /* _ARTIFICIALSTIMULATORAGENT_H_ */
