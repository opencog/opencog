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
#include "XMLBufferReader.h"

namespace opencog {

class CommandRequestProcessor : public RequestProcessor
{
    public:
        ~CommandRequestProcessor();
        CommandRequestProcessor(void);
        virtual void processRequest(CogServerRequest *);

    private:
        std::string loadXML(XMLBufferReader *);
        std::string data(std::string);
        std::string help(std::string);
        std::string load(std::string);
        std::string ls(void);

}; // class
}  // namespace

#endif
