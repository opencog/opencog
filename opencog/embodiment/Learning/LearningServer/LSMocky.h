/**
 * LSMocky.h
 *
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#ifndef LSMOCKY_H
#define LSMOCKY_H

#include "util/Logger.h"
#include <opencog/atomspace/AtomSpace.h>
#include <EmbodimentCogServer.h>
#include <SystemParameters.h>
#include "SleepAgent.h"

using namespace opencog;

namespace LearningServer{

class LSMocky : public MessagingSystem::EmbodimentCogServer {

	public:

		/**
		 * Constructor and Destructor
		 */
        static BaseServer* createInstance();
		LSMocky();
		void init(const std::string &myId, const std::string &ip, int portNumber,
		   Control::SystemParameters & parameters);
		~LSMocky();

		bool processNextMessage(MessagingSystem::Message *msg);

        Factory<SleepAgent, Agent> sleepAgentFactory;

	private:

		AtomSpace * atomSpace;			// store behavior descriptors and space server
										// with latest map

		std::string learningPet;		// the id of the pet using the LS
		std::string learningSchema;     // the trick being learned

}; // class
}  // namespace

#endif
