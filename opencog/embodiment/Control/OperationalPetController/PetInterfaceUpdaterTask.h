/**
 * PetInterfaceUpdaterTask.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Thu Oct 18 12:46:07 BRDT 2007
 */

#ifndef PETINTERFACEUPDATERTASK_H
#define PETINTERFACEUPDATERTASK_H

#include <NetworkElement.h>
#include <IdleTask.h>
#include <string>

namespace OperationalPetController {

class OPC; 

class PetInterfaceUpdaterTask : public MessagingSystem::IdleTask {

    private:

        int interfacePortNumber;
        void sendLine(const std::string &line);

    public:

        // ***********************************************/
        // Constructors/destructors

        ~PetInterfaceUpdaterTask();
        PetInterfaceUpdaterTask();

        void run(MessagingSystem::NetworkElement *ne);
        void updateLocalMap(OPC *opc);
        void updateFeelings(OPC *opc);
        void updateLastSelectedAction(OPC *opc);

        static void startGUI();

}; // class
}  // namespace

#endif
