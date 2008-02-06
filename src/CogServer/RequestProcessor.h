/**
 * RequestProcessor.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef REQUESTPROCESSOR_H
#define REQUESTPROCESSOR_H

namespace opencog {

class CogServerRequest;

class RequestProcessor
{
    public:
		
        virtual ~RequestProcessor() {}
        virtual void processRequest(CogServerRequest *request) = 0;

}; // class
}  // namespace

#endif
