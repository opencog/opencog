#include "RunningBuiltInProcedure.h"
    
using namespace Procedure;
using namespace PerceptionActionInterface;

RunningBuiltInProcedure::RunningBuiltInProcedure(const PAI& _pai, const BuiltInProcedure& _p, const std::vector<combo::vertex>& _arguments) : pai(_pai), p(_p), arguments(_arguments) {
            finished = false;
            failed = false;
            result = combo::id::null_vertex; // TODO: perhaps there is a "undefined result" constant or something like that...
}
RunningBuiltInProcedure::~RunningBuiltInProcedure() {}
        
void RunningBuiltInProcedure::run() {
    if (finished) return; // must run only once.
    try {
        result = p.execute(arguments);
    } catch (...) {
        failed = true;
    }
    finished = true;
}
         
bool RunningBuiltInProcedure::isFinished() const { 
    if (!finished) return false;
    if (!p.isPetAction()) return true;
    const ActionPlanID* planId = boost::get<ActionPlanID>(&result);
    if (!planId) return true;
    return pai.isPlanFinished(*planId);
}

bool RunningBuiltInProcedure::isFailed() const { 
    if (failed) return true;
    if (p.isPetAction() && finished) {
        const ActionPlanID* planId = boost::get<ActionPlanID>(&result);
	if (!planId) return true;
	return pai.hasPlanFailed(*planId); 
    } else {
        return false;
    }
}

combo::vertex RunningBuiltInProcedure::getResult() const { 
    return result;
}

