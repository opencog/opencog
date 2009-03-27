/**
 * SleepAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 12:36:43 BRT 2007
 */

#ifndef SLEEPAGENT_H
#define SLEEPAGENT_H

#include <opencog/server/Agent.h>

namespace OperationalPetController {

using namespace opencog;

class SleepAgent : public Agent {

    private:

    public:

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("OperationalPetController::SleepAgent");
            return _ci;
        }

        ~SleepAgent();
        SleepAgent();

        void run(CogServer *opc);

}; // class
}  // namespace

#endif
