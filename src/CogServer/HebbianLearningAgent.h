/**
 * HebbianLearningAgent.h
 *
 * Simple Hebbian Learning, currently only updates existing Hebbian Links.
 *
 * Author: Joel Pitt 
 * Creation: Wed Apr 9 15:32:00 GMT+12 2008
 */

#ifndef _HEBBIAN_LEARNING_AGENT_H
#define _HEBBIAN_LEARNING_AGENT_H

#include <string>
#include <Logger.h>

#include <AtomSpace.h>
#include <MindAgent.h>
#include <AttentionValue.h>

namespace opencog {

class CogServer;

class HebbianLearningAgent : public MindAgent
{

    private:
	AtomSpace* a;

	float targetConjunction(std::vector<Handle> handles);
	float getNormSTI(AttentionValue::sti_t s);
	std::vector<Handle>& moveSourceToFront(std::vector<Handle> &outgoing);
    public:
	// Convert links to/from inverse as necessary.
	bool convertLinks;
	// Maximum LTI of a link that can be converted.
	AttentionValue::lti_t conversionThreshold;

	HebbianLearningAgent();
	virtual ~HebbianLearningAgent();
	virtual void run(CogServer *server);

	void hebbianLearningUpdate();

}; // class

}; // namespace
#endif // _HEBBIAN_LEARNING_AGENT_H

