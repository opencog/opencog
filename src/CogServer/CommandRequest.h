/*
 * src/CogServer/CommandRequest.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef COMMANDREQUEST_H
#define COMMANDREQUEST_H

#include <string>
#include <queue>

#include "ServerSocket.h"
#include "RequestProcessor.h"
#include "CallBackInterface.h"

namespace opencog
{

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
