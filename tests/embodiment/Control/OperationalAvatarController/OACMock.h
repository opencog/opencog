/*
 * tests/embodiment/Control/OperationalAvatarController/OACMock.h
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date   2011-06-14
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

#include <opencog/comboreduct/type_checker/type_tree.h>

#include <opencog/embodiment/Learning/LearningServerMessages/SchemaMessage.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

// For loading Scheme scripts by C++ code
#include <opencog/guile/load-file.h>

#include <opencog/util/files.h>
#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>

#include <boost/format.hpp>

#include <fstream>
#include <iostream>

#include <opencog/embodiment/Control/OperationalAvatarController/OAC.h>

using namespace opencog::oac;
using namespace Procedure;
using namespace AvatarCombo;
using namespace opencog;

/**
 * This class is used to set up the environment for testing all kinds of mind 
 * agents running inside OAC. 
 *
 * It tries to use the same set up of OAC in reality as much as possible:
 *
 * 1. The customized target 'DistributeOACStuff' in CMake will do the same 
 *    distribution before testing, as you do in running OAC manually, say 
 *    ./make_distribution bin Embodiment
 * 2. It copies the code in OACExecutable.cc as much as possible. 
 */
class OACMock {

private:
       
    /**
     * Arguments used by OAC::init
     * You can easily get these arguments by set MANUAL_OAC_LAUNCH true in 
     * config file, run the embodiment system and copy the command of 
     * running OAC in console. 
     */
    std::string agentBrainId; 
    std::string ownerId; 
    std::string agentType; 
    std::string agentTrait; 
    std::string networkElementPort; 
    std::string cogServerShellPort;
    std::string zmqPublishPort; 

    OAC *oac; 

public:

    /**
     * Getters
     */
    std::string & getAgentBrainId() {
        return this->agentBrainId; 
    }

    std::string & getOwnerId() {
        return this->ownerId; 
    }

    std::string & getAgentType() {
        return this->agentType; 
    }

    std::string & getAgentTrait() {
        return this->agentTrait; 
    }

    std::string & getNetworkElementPort() {
        return this->networkElementPort; 
    }

    std::string & getCogServerShellPort() {
        return this->cogServerShellPort; 
    }

    std::string & getZmqPublishPort() {
        return this->zmqPublishPort; 
    }

    /**
     * Set configurations before creating an instance of OAC
     */
    void setConfig(); 

    /**
     * Create an instance of OAC, load all the modules specified in the
     * config, and call OAC::init
     *
     * @return A reference of newly created OAC
     *
     * @note   If you want customized configurations for you test different from 
     *         the set up in setConfig function, set your configurations before 
     *         invoking createOAC. 
     */
    OAC & createOAC();

    /**
     * Return the reference of OAC
     */
    OAC & getOAC() {
        return *(this->oac);
    }

    /**
     * Generate a message from given file, which can be processed by
     * OAC::processNextMessage or PAI::processPVPMessage
     */
    Message * createMessageFromFile(const std::string & from, 
                                    const std::string & to,
                                    int msgType,
                                    const char * fileName
                                   ); 
};
