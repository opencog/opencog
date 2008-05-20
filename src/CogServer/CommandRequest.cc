/*
 * src/CogServer/CommandRequest.cc
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

#include "CommandRequest.h"
#include "CommandRequestProcessor.h"

using namespace opencog;

CommandRequest::~CommandRequest() {
}

RequestProcessor* CommandRequest::requestProcessor = NULL;

CommandRequest::CommandRequest(CallBackInterface *callBack, 
                               std::string &command,
                               std::queue<std::string> &args)
{
    this->callBackRequestor = callBack;
    this->command = command;
    this->args = args;
}

RequestProcessor * CommandRequest::getRequestProcessor()
{
    if (requestProcessor == NULL) requestProcessor = new CommandRequestProcessor();
    return requestProcessor;
}

void CommandRequest::setAnswer(std::string &commandOutput) {
    answer = commandOutput;
}

std::string CommandRequest::getCommand() {
    return command;
}

std::queue<std::string> CommandRequest::getArgs() {
    return args;
}

void CommandRequest::callBack() {
    callBackRequestor->callBack(answer);
}
