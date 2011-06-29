/*
 * opencog/embodiment/Control/OperationalAvatarController/MockOpcHCTestExec.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#include <boost/lexical_cast.hpp>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <unistd.h>
#include <opencog/util/files.h>
#include "MockOpcHCTest.h"

using namespace OperationalAvatarController;
using namespace opencog;
using namespace boost;

int main(int argc, char *argv[])
{
    //learning time in second for iteration 1 and 2 respectively
    unsigned int learning_time1 = 10, learning_time2 = 100;
    //maximum number of cycle to run
    unsigned long max_cycle = 10000;

    if(argc < 3 || argc > 6) {
        std::cout << "Usage: " << argv[0] << " " << "PetID " << "portNumber "
                  << "[learning_time1 = 10] "
                  << "[learning_time2 = 100] "
                  << "[max_cycle = 10000]" << std::endl;
        exit(1);
    }
    
    if(argc >= 4)
        learning_time1 = lexical_cast<unsigned int>(argv[3]);
    if(argc >= 5)
        learning_time2 = lexical_cast<unsigned int>(argv[4]);
    if(argc == 6)
        max_cycle = lexical_cast<unsigned long>(argv[5]);

    config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);


    // if exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.
    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }

    

    //char petName[256];
    //int petID = atoi(argv[1]);
    //int portNumber = 5100 + petID;
    int portNumber = lexical_cast<int>(argv[2]);
    //sprintf(petName, "%d", petID);

    server(MockOpcHCTest::createInstance);
    MockOpcHCTest& mOpcHcTest = static_cast<MockOpcHCTest&>(server());
    mOpcHcTest.init(argv[1], "127.0.0.1", portNumber, argv[1],
                    learning_time1, learning_time2, max_cycle);
    mOpcHcTest.serverLoop();
    //delete mOpcHcTest;
    return 0;
}
