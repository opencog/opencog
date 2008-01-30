/**
 * NetworkServer.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef NETWORKSERVER_H
#define NETWORKSERVER_H

namespace opencog {

class NetworkServer {

    public:
		
		virtual ~NetworkServer() {};
        virtual void start() = 0;

}; // class
}  // namespace

#endif
