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
#include <NetworkElement.h>
#include <SystemParameters.h>

using namespace opencog;

namespace LearningServer{

class LSMocky : public MessagingSystem::NetworkElement {

	public:

		/**
		 * Constructor and Destructor
		 */
		LSMocky(const std::string &myId, const std::string &ip, int portNumber,
		   Control::SystemParameters & parameters);
		~LSMocky();

		bool processNextMessage(MessagingSystem::Message *msg);

		void setUp();

	private:

		AtomSpace * atomSpace;			// store behavior descriptors and space server
										// with latest map

		std::string learningPet;		// the id of the pet using the LS
		std::string learningSchema;     // the trick being learned

}; // class
}  // namespace

#endif
