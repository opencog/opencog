/**
 * CommandRequestProcessor.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef COMMANDREQUESTPROCESSOR_H
#define COMMANDREQUESTPROCESSOR_H

#include <string>
#include "RequestProcessor.h"

namespace opencog {

class CommandRequestProcessor : public RequestProcessor
{
    public:
        ~CommandRequestProcessor();
        CommandRequestProcessor();
        virtual void processRequest(CogServerRequest *request);

    private:
        std::string data(std::string buf);
        std::string load(std::string fileName);
        std::string ls();

}; // class
}  // namespace

#endif
