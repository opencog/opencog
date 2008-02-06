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
#include "RequestProcessor.h"

namespace opencog {

class CogServerRequest
{
    public:
        virtual ~CogServerRequest() {}
        virtual void callBack() = 0;
        virtual RequestProcessor * getRequestProcessor() = 0;
        virtual void processRequest() {
	         getRequestProcessor()->processRequest(this);
        }

}; // class
}  // namespace

#endif
