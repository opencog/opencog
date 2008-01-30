/**
 * MindAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef MINDAGENT_H
#define MINDAGENT_H

namespace opencog {

class CogServer;

class MindAgent {

    public:
		
		virtual ~MindAgent() {}
		
        virtual void run(CogServer *server) = 0;

}; // class
}  // namespace

#endif
