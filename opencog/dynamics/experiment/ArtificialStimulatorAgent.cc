/*
 * StimulusProviderAgent.cc
 *
 *  Created on: 4 Aug, 2015
 *      Author: misgana
 */
#include <opencog/dynamics/experiment/ArtificialStimulatorAgent.h>
#include <opencog/server/CogServer.h>


using namespace opencog;

ArtificialStimulatorAgent::ArtificialStimulatorAgent(CogServer& cs) :
        Agent(cs)
{
    _as = &cs.getAtomSpace();
}

ArtificialStimulatorAgent::~ArtificialStimulatorAgent()
{

}

void ArtificialStimulatorAgent::run()
{

}

