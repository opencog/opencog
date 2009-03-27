/**
 * ProcedureInterpreterAgent.cc
 *
 * Author: Welter Luigi
 */

#include "OPC.h"
#include "ProcedureInterpreterAgent.h"

using namespace OperationalPetController;

ProcedureInterpreterAgent::~ProcedureInterpreterAgent() {
}

ProcedureInterpreterAgent::ProcedureInterpreterAgent() {
}

void ProcedureInterpreterAgent::setInterpreter(Procedure::ProcedureInterpreter* _interpreter) {
    interpreter = _interpreter;
}

void ProcedureInterpreterAgent::run(opencog::CogServer *server) {
    logger().log(opencog::Logger::FINE, "ProcedureInterpreterAgent::run()");
    interpreter->run(&(((OPC*)server)->getNetworkElement()));
}
