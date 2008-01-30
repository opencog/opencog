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
#include "CallBackInterface.h"

namespace opencog {

class CommandRequest : public CogServerRequest {

    private:

        std::string answer;
        CallBackInterface *callBackRequestor;

    public:
		
        std::string command;
        std::queue<std::string> args;

		~CommandRequest();
		CommandRequest(CallBackInterface *callback, std::string &command, std::queue<std::string> &args);
        std::string getType();
        void callBack();
        void setAnswer(std::string &cmdOutput);
        std::string getCommand();
        std::queue<std::string> getArgs();

}; // class
}  // namespace

#endif
