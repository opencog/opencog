/**
 * ActionSelectionAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:46:02 BRT 2007
 */

#ifndef ACTIONSELECTIONAGENT_H
#define ACTIONSELECTIONAGENT_H

#include <opencog/server/Agent.h>
#include <time.h>

namespace OperationalPetController {

using namespace std;

class ActionSelectionAgent : public opencog::Agent {

    private:

        time_t lastTickTime;

    public:

        ~ActionSelectionAgent();
        ActionSelectionAgent();

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("OperationalPetController::ActionSelectionAgent");
            return _ci;
        }

        void run(opencog::CogServer *server);

}; // class
}  // namespace

#endif
