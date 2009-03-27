/**
 * PetInterfaceUpdaterAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Thu Oct 18 12:46:07 BRDT 2007
 */

#ifndef PETINTERFACEUPDATERAGENT_H
#define PETINTERFACEUPDATERAGENT_H

#include <opencog/server/Agent.h>
#include <string>

namespace OperationalPetController {

class OPC; 

class PetInterfaceUpdaterAgent : public opencog::Agent {

    private:

        int interfacePortNumber;
        void sendLine(const std::string &line);

    public:

        // ***********************************************/
        // Constructors/destructors

        ~PetInterfaceUpdaterAgent();
        PetInterfaceUpdaterAgent();

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("OperationalPetController::PetInterfaceUpdaterAgent");
            return _ci;
        }

        void run(opencog::CogServer *server);
        void updateLocalMap(OPC *opc);
        void updateFeelings(OPC *opc);
        void updateLastSelectedAction(OPC *opc);

        static void startGUI();

}; // class
}  // namespace

#endif
