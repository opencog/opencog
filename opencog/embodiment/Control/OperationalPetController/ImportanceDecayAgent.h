/**
 * ImportanceDecayAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:30:53 BRT 2007
 */

#ifndef IMPORTANCEDECAYAGENT_H
#define IMPORTANCEDECAYAGENT_H

#include <opencog/server/Agent.h>
#include <time.h>

namespace OperationalPetController {

class ImportanceDecayAgent : public opencog::Agent {

    private:

        time_t lastTickTime;

    public:

        ImportanceDecayAgent();
        virtual ~ImportanceDecayAgent();

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("OperationalPetController::ImportanceDecayAgent");
            return _ci;
        }

        void run(opencog::CogServer *server);

}; // class
}  // namespace

#endif
