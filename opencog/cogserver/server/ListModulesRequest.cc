#include "ListModulesRequest.h"
#include <opencog/cogserver/server/CogServer.h>

using namespace opencog;

ListModulesRequest::ListModulesRequest(CogServer& cs) : Request(cs)
{
}

ListModulesRequest::~ListModulesRequest()
{
    logger().debug("[ListModulesRequest] destructor");
}

bool ListModulesRequest::syntaxError()
{
    _error << "invalid syntax" << std::endl;
    sendError();
    return false;
}

bool ListModulesRequest::execute()
{
    std::string moduleList = _cogserver.listModules();
    send(moduleList);
    return true;
}

void ListModulesRequest::sendError()
{
    send(_error.str());
}
