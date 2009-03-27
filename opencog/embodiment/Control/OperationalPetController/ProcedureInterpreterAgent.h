/**
 * ProcedureInterpreterAgent.h
 *
 * Author: Welter Luigi
 */

#ifndef PROCEDUREINTERPRETERAGENT_H
#define PROCEDUREINTERPRETERAGENT_H

#include <opencog/server/Agent.h>
#include "ProcedureInterpreter.h"

namespace OperationalPetController {

class ProcedureInterpreterAgent : public opencog::Agent {

    private:

        Procedure::ProcedureInterpreter* interpreter;

    public:

        ProcedureInterpreterAgent();
        virtual ~ProcedureInterpreterAgent();
        void setInterpreter(Procedure::ProcedureInterpreter*);

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("OperationalPetController::ProcedureInterpreterAgent");
            return _ci;
        }

        void run(opencog::CogServer *server);

}; // class
}  // namespace

#endif
