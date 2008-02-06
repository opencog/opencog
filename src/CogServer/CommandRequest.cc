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
