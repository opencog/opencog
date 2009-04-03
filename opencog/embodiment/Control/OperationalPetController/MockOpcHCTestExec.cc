/*
 * opencog/embodiment/Control/MessagingSystem/MockOpcHCTestExec.cc
 *
 * Copyleft (C) 2007-2008 Nil Geisweiller
 * All Wrongs Reserved
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
#include <SystemParameters.h>
#include <unistd.h>
#include "util/files.h"
#include "MockOpcHCTest.h"

using namespace OperationalPetController;

int main(int argc, char *argv[])
{

    opencog::cassert(TRACE_INFO, argc == 3);
    Control::SystemParameters parameters;

    // if exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.
    if (fileExists(parameters.get("CONFIG_FILE").c_str())) {
        parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }


    //char petName[256];
    //int petID = atoi(argv[1]);
    //int portNumber = 5100 + petID;
    int portNumber = atoi(argv[2]);
    //sprintf(petName, "%d", petID);

    server(MockOpcHCTest::createInstance);
    MockOpcHCTest& mOpcHcTest = static_cast<MockOpcHCTest&>(server());
    mOpcHcTest.init(argv[1], "127.0.0.1", portNumber, argv[1], parameters);
    mOpcHcTest.serverLoop();
    //delete mOpcHcTest;
    return 0;
}
