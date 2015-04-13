/*
 * tests/embodiment/Control/Procedure/ComboInterpreterUTest.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Elvys Borges
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


#include <opencog/embodiment/Control/Procedure/ComboInterpreter.h>
#include <opencog/embodiment/Control/Procedure/RunningProcedureId.h>
#include <tests/embodiment/Control/PerceptionActionInterface/ActionPlanSenderMock.h>
#include <opencog/util/files.h>
#include <tests/embodiment/Control/PerceptionActionInterface/AvatarInterfaceMock.h>
#include <opencog/embodiment/Control/Procedure/ComboProcedureRepository.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <tests/embodiment/Control/OperationalAvatarController/LSMessageSenderMock.h>
#include <opencog/embodiment/Control/PredicateUpdaters/PredicatesUpdater.h>
#include <opencog/embodiment/Control/OperationalAvatarController/Pet.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#ifndef COMBOINTERPRETERUTEST_H
#define COMBOINTERPRETERUTEST_H


#define NUMBER_OF_COMBO_TREES 2

using namespace opencog::pai;
using namespace combo;
using namespace opencog::control;
using namespace opencog;

class ComboInterpreterUTest
{

protected:

    AtomSpace* atomSpace;

    HandleSeq toUpdateHandles;
    std::list<ActionPlan> sentActionPlans;
    ResponsiveActionPlanSender *sender;
    AvatarInterfaceMock *avatarInterface;


    PAI* ppai;

    Procedure::ComboProcedureRepository cpr;

    std::string xmlFileName;

public:

    ComboInterpreterUTest() {
        config(EmbodimentConfig::embodimentCreateInstance, true);
	config().set("ENABLE_ACTION_COLLECT", "false"); 
        init(string("pvpMsg1.xml"), PAIUtils::getInternalId("32f56136-7973-4703-915b-6ec1bf5c67fa"));
//      init(string("pvpMsg1.xml"), string("Fido"));
    }

    ComboInterpreterUTest(std::string _xmlFileName, std::string petName) {
        config(EmbodimentConfig::embodimentCreateInstance, true);
	config().set("ENABLE_ACTION_COLLECT", "false"); 
        init(_xmlFileName, petName);
    }

    virtual ~ComboInterpreterUTest() {}

    void init(std::string _xmlFileName, std::string petName) {
        atomSpace = new AtomSpace();

        xmlFileName = _xmlFileName;

        sender = new ResponsiveActionPlanSender();
        avatarInterface = new AvatarInterfaceMock(petName, PAIUtils::getInternalId("Wynx"), string(""));
        ppai = new PAI(*atomSpace, *sender, *avatarInterface);
        avatarInterface->setPAI(ppai);

        sender->setPai(ppai);

        string pvpMsg;
        printf("Will open file %s\n", xmlFileName.c_str());
        if (!appendFileContent( string(PVP_XML_FILE_PATH + string("/") +  xmlFileName).c_str(), pvpMsg)) {
            printf("Could not read content of file %s\n",
                   string(PVP_XML_FILE_PATH + string("/") +  xmlFileName).c_str());
        }
        printf("Processing pvp message:\n");
        printf("%s", pvpMsg.c_str());
        ppai->processPVPMessage(pvpMsg, toUpdateHandles);

        PredicatesUpdater * updater;

        updater = new PredicatesUpdater(*atomSpace, avatarInterface->getPetId());

        updater->update(toUpdateHandles, ppai->getLatestSimWorldTimestamp());

    }

    void setUp() {

    }

    void tearDown() {
    }

    void runProcedureInCombo(std::string cmd, vertex vResult) {

        std::cout << "Start (" << cmd.c_str() << ")" << std::endl;

        combo::combo_tree tree;

        std::stringstream ss(cmd);
        // ss >> tree;
        AvatarCombo::operator>>(ss, tree);

        cpr.instantiateProcedureCalls(tree, true);

        std::vector<vertex> arguments ;
        Procedure::ComboInterpreter interpreter(*ppai);
        Procedure::RunningProcedureId runningId = interpreter.runProcedure(tree, arguments);
        while (! interpreter.isFinished(runningId)) {
            interpreter.run(NULL);
            sender->processSentMessage();
        }

        if (!interpreter.isFailed(runningId)) {
            vertex result = interpreter.getResult(runningId);
            std::cout << "Expected result (" << cmd.c_str() << ") = " << vResult << std::endl;
            std::cout << "Got result (" << cmd.c_str() << ") = " << result << std::endl;
            TS_ASSERT(true);
            //if (result != id::null_vertex && vResult != id::null_vertex) {

            if (is_builtin(result)) {
                TS_ASSERT(get_builtin(result) == vResult);
            }
            if (is_definite_object(result)) {
                TS_ASSERT(get_definite_object(result) == vResult);
            }
            if (is_action_result(result)) {
                TS_ASSERT(get_action(result) == vResult);
            }
            if (is_contin(result)) {
                TS_ASSERT_DELTA(get_contin(result), get_contin(vResult),  0.001);
            }
        } else {
            std::cout << "Failed (" << cmd.c_str() << ") = " << std::endl;
            TS_ASSERT(false);
        }
    }

    void runProcedureInComboRandom(std::string cmd, bool is_null_obj) {

        std::cout << "Start (" << cmd.c_str() << ")" << std::endl;

        combo::combo_tree tree;

        std::stringstream ss(cmd);
        ss >> tree;

        std::vector<vertex> arguments ;
        Procedure::ComboInterpreter interpreter(*ppai);
        Procedure::RunningProcedureId runningId = interpreter.runProcedure(tree, arguments);
        while (! interpreter.isFinished(runningId)) {
            interpreter.run(NULL);
            sender->processSentMessage();
        }

        if (!interpreter.isFailed(runningId)) {
            vertex result = interpreter.getResult(runningId);
            std::cout << "Result (" << cmd.c_str() << ") = " << result << std::endl;
            TS_ASSERT(true);
            //if (result != id::null_vertex && vResult != id::null_vertex) {

            if (is_null_obj) {

                if (is_definite_object(result)) {
                    TS_ASSERT(get_definite_object(result) == id::null_obj);
                } else {
                    TS_ASSERT(false);
                }

            } else {

                if (is_definite_object(result)) {
                    TS_ASSERT(get_definite_object(result) != id::null_obj);
                } else {
                    TS_ASSERT(false);
                }
            }
        } else {
            std::cout << "Failed (" << cmd.c_str() << ") = " << std::endl;
            TS_ASSERT(false);
        }
    }

};

#endif
