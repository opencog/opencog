/**
 * CommandRequest.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef COMMANDREQUEST_H
#define COMMANDREQUEST_H

#include <string>
#include <queue>

#include "ServerSocket.h"
#include "RequestProcessor.h"
#include "CallBackInterface.h"

namespace opencog {

class CommandRequest : public CogServerRequest
{
    private:

        std::string answer;
        CallBackInterface *callBackRequestor;
        static RequestProcessor *requestProcessor;

    public:
		
        std::string command;
        std::queue<std::string> args;

        ~CommandRequest();
        CommandRequest(CallBackInterface *callback, 
                       std::string &command,
                       std::queue<std::string> &args);
        void callBack();
        RequestProcessor * getRequestProcessor();

        void setAnswer(std::string &cmdOutput);
        std::string getCommand();
        std::queue<std::string> getArgs();

}; // class
}  // namespace

#endif
