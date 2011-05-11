#include "PyMindAgent.h"

using namespace opencog;

PyMindAgent::PyMindAgent()
{
    pyagent=NULL;
}

PyMindAgent::PyMindAgent(const std::string& moduleName, const std::string& className)
{
    pyagent=NULL;
}

PyMindAgent::~PyMindAgent()
{
}

void PyMindAgent::run(CogServer* server)
{
}

