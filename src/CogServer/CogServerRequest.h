/**
 * CogServerRequest.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef COGSERVERREQUEST_H
#define COGSERVERREQUEST_H

#include <string>

namespace opencog {

class CogServerRequest {

    public:
		
		virtual ~CogServerRequest() {}
        virtual std::string getType() = 0;
        virtual void callBack() = 0;

}; // class
}  // namespace

#endif
