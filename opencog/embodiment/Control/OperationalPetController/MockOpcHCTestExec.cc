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
#include <EmbodimentConfig.h>
#include <unistd.h>
#include "util/files.h"
#include "MockOpcHCTest.h"

using namespace OperationalPetController;
using namespace opencog;

int main(int argc, char *argv[])
{

    cassert(TRACE_INFO, argc == 3);

    config(Control::EmbodimentConfig::embodimentCreateInstance, true);


    // if exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.
    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }


    //char petName[256];
    //int petID = atoi(argv[1]);
    //int portNumber = 5100 + petID;
    int portNumber = atoi(argv[2]);
    //sprintf(petName, "%d", petID);

    server(MockOpcHCTest::createInstance);
    MockOpcHCTest& mOpcHcTest = static_cast<MockOpcHCTest&>(server());
    mOpcHcTest.init(argv[1], "127.0.0.1", portNumber, argv[1]);
    mOpcHcTest.serverLoop();
    //delete mOpcHcTest;
    return 0;
}
