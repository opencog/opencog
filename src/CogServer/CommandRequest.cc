#include "CommandRequest.h"

using namespace opencog;

CommandRequest::~CommandRequest() {
}

CommandRequest::CommandRequest(CallBackInterface *callBack, std::string &command, std::queue<std::string> &args) {
    this->callBackRequestor = callBack;
    this->command = command;
    this->args = args;
}

std::string CommandRequest::getType() {
    return "COMMAND_LINE";
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
