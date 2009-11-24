/*
 * opencog/embodiment/Control/OperationalPetController/PetInterfaceUpdaterAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#ifndef PETINTERFACEUPDATERAGENT_H
#define PETINTERFACEUPDATERAGENT_H

#include <opencog/server/Agent.h>
#include <string>

namespace OperationalPetController
{

class OPC;


/**
 * Pet Interface is a tool created for helping debugging and testing OPC.
 * Two parts composes PetInterface, a C++ and a TCL/TK.
 * PetInterfaceUpdaterAgent is the C++ part. It collects information about
 * the OPC and delivers it to a Viewer Interface via network.
 * The Viewer was coded in TCL/TK. It is a console that can be used to send
 * commands to and receive data from the OPC. That IP and Port you've mentioned
 * are used to stablish a communication between the OPC and the Viewer
 * Interface. However, it is a legacy code. It was only useful during a
 * period where the Petaverse developers hadn't the Multiverse Proxy/Client.
 * That tool becomes obsolete and perhaps could be removed from the repository.
 *
 * It seems the Viewer TCL script was not included into the repo when
 * the Petaverse code was merged with the OpenCog one, that's why one cannot
 * find a second file which contains the term "LASTSELECTEDSCHEMA" for instance.
 **/
class PetInterfaceUpdaterAgent : public opencog::Agent
{

private:

    int interfacePortNumber;
    void sendLine(const std::string &line);

public:

    // ***********************************************/
    // Constructors/destructors

    ~PetInterfaceUpdaterAgent();
    PetInterfaceUpdaterAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }
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
