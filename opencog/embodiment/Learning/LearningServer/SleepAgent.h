/**
 * SleepAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 12:36:43 BRT 2007
 */

#ifndef SLEEPAGENTLS_H
#define SLEEPAGENTLS_H

#include <opencog/server/Factory.h>
#include <opencog/server/Agent.h>

namespace LearningServer {

using namespace opencog;

class SleepAgent : public opencog::Agent {

    private:

    public:

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("LearningServer::SleepAgent");
            return _ci;
        }

        ~SleepAgent();
        SleepAgent();

        void run(opencog::CogServer *opc);

}; // class
}  // namespace

#endif
