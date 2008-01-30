/**
 * CogServerSetup.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef COGSERVERSETUP_H
#define COGSERVERSETUP_H

#include "CogServer.h"

namespace opencog {

class CogServer;

class CogServerSetup {

    public:
		
		virtual ~CogServerSetup();
		CogServerSetup();
        virtual void setUp(CogServer *server);

}; // class
}  // namespace

#endif
