/*
 * tests/embodiment/Control/Procedure/ComboSelectInterpreterUTest.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#ifndef COMBOSELECTINTERPRETERUTEST_H
#define COMBOSELECTINTERPRETERUTEST_H

#include "ComboSelectInterpreter.h"
#include "ComboProcedure.h"
#include "RunningProcedureId.h"
#include "ActionPlanSenderMock.h"
#include "util/files.h"
#include "AvatarInterfaceMock.h"
#include "ComboProcedureRepository.h"
#include "SystemParameters.h"
#include "LSMessageSenderMock.h"
#include "PredicatesUpdater.h"
#include "Pet.h"
#include "AvatarComboVocabulary.h"



#define NUMBER_OF_COMBO_TREES 2

using namespace opencog;
using namespace opencog::pai;
using namespace combo;

class ComboSelectInterpreterUTest
{

protected:

    AtomSpace* atomSpace;
    control::SystemParameters parameters;
    HandleSeq toUpdateHandles;
    std::list<ActionPlan> sentActionPlans;
    ResponsiveActionPlanSender *sender;
    AvatarInterfaceMock *avatarInterface;


    PAI* ppai;

    Procedure::ComboProcedureRepository cpr;

    std::string xmlFileName;

public:

    ComboSelectInterpreterUTest() {
        init(string("pvpMsg1.xml"),
             PAIUtils::getInternalId("32f56136-7973-4703-915b-6ec1bf5c67fa"));
        //init(string("pvpMsg1.xml"), string("Fido"));
    }

    ComboSelectInterpreterUTest(std::string _xmlFileName,
                                std::string petName) {
        init(_xmlFileName, petName);
    }

    virtual ~ComboSelectInterpreterUTest() {}
    virtual opencog::RandGen& getRandGen() = 0;

    void init(std::string _xmlFileName, std::string petName) {
        xmlFileName = _xmlFileName;

        atomSpace = new AtomSpace();
        sender = new ResponsiveActionPlanSender();
        avatarInterface = new AvatarInterfaceMock(petName,
                                            PAIUtils::getInternalId("Wynx"),
                                            string(""));
        ppai = new PAI(*atomSpace, *sender, *avatarInterface, parameters);

        avatarInterface->setPAI(ppai);
        sender->setPai(ppai);

        string pvpMsg;
        //printf("Will open  file %s\n", xmlFileName.c_str());
        if (!appendFileContent( string(PVP_XML_FILE_PATH + string("/") +  xmlFileName).c_str(), pvpMsg)) {
            printf("Could not read content of file %s\n",
                   string(PVP_XML_FILE_PATH + string("/") +  xmlFileName).c_str());
        }
        //printf("Processing pvp message:\n"); //, pvpMsg.c_str());
        ppai->processPVPMessage(pvpMsg, toUpdateHandles);

        PredicatesUpdater * updater;
        updater = new PredicatesUpdater(*atomSpace, avatarInterface->getPetId());
        updater->update(toUpdateHandles, ppai->getLatestSimWorldTimestamp());
    }

    Procedure::ComboProcedure createComboProcedure(std::string name, int arity, std::string cmd) {
        std::cout << "Creating combo procedure (" << name << "-" << cmd << ")" << std::endl;

        combo::combo_tree tree;
        std::stringstream ss(cmd);
        ss >> tree;

        cpr.instantiateProcedureCalls(tree, true);
        Procedure::ComboProcedure cp(name, arity, tree);

        return cp;
    }

    void setUp() {
    }

    void tearDown() {
    }

    void runProcedureInComboSelect(Procedure::ComboProcedure& cp1, Procedure::ComboProcedure& cp2, vertex vResult) {

        stringstream ss1;
        ss1 << cp1.getComboTree();

        stringstream ss2;
        ss2 << cp2.getComboTree();

        std::cout << "Start " << std::endl << "(" << ss1.str() << ")" << std::endl << "(" << ss2.str() << ")" << std::endl;

        std::vector<vertex> arguments ;
        Procedure::ComboSelectInterpreter interpreter(*ppai, getRandGen());
        Procedure::RunningProcedureId runningId = interpreter.runProcedure(cp1, cp2, arguments);

        while (! interpreter.isFinished(runningId)) {
            interpreter.run(NULL);
        }

        if (!interpreter.isFailed(runningId)) {
            vertex result = interpreter.getResult(runningId);
            std::cout << "Expected result = " << vResult << std::endl;
            std::cout << "Got result = " << result << std::endl;

            if (is_builtin(result)) {
                TS_ASSERT(get_builtin(result) == vResult);
            } else if (is_action_result(result)) {
                TS_ASSERT(get_action(result) == vResult);
            } else {
                TS_ASSERT(false);
            }
        } else {

            std::cout << "Failed " << std::endl << "(" << ss1.str() << ")" << std::endl << "(" << ss2.str() << ")" << std::endl;
            TS_ASSERT(false);
        }
    }

    void runProcedureInComboSelectWildCard(Procedure::ComboProcedure& cp1, Procedure::ComboProcedure& cp2, combo::vertex vResult,
                                           combo::variable_unifier& vu, combo::variable_unifier& vu_result) {

        stringstream ss1;
        ss1 << cp1.getComboTree();

        stringstream ss2;
        ss2 << cp2.getComboTree();

        std::cout << "Start " << std::endl << "(" << ss1.str() << ")" << std::endl << "(" << ss2.str() << ")" << std::endl;

        std::vector<vertex> arguments ;
        Procedure::ComboSelectInterpreter interpreter(*ppai, getRandGen());
        Procedure::RunningProcedureId runningId = interpreter.runProcedure(cp1, cp2, arguments, vu);

        while (! interpreter.isFinished(runningId)) {
            interpreter.run(NULL);
            sender->proccessSentMessage();
        }

        if (!interpreter.isFailed(runningId)) {
            vertex result = interpreter.getResult(runningId);
            std::cout << "Expected result = " << vResult << std::endl;
            std::cout << "Got result = " << result << std::endl;

            combo::variable_unifier vu_result_local = interpreter.getUnifierResult(runningId);

            if (is_builtin(result)) {
                TS_ASSERT(get_builtin(result) == vResult);
                if (get_builtin(result) == id::logical_true) {
                    TS_ASSERT(!vu_result_local.isEmpty());
                    if (vu_result_local.isUpdated()) {
                        TS_ASSERT(vu_result_local.isOneVariableActive());
                    } else {
                        TS_ASSERT(!vu_result_local.isOneVariableActive());
                    }

                } else if (get_builtin(result) == id::logical_false) {
                    //TS_ASSERT(!vu_result_local.isEmpty());
                    // don't metter if is updated or not... in every case
                    // there should be no variable active
                    TS_ASSERT(!vu_result_local.isOneVariableActive());
                }

            } else if (is_action_result(result)) {
                TS_ASSERT(get_action(result) == vResult);
                if (get_action(result) == id::action_success ) {
                    TS_ASSERT(!vu_result_local.isEmpty());
                    if (vu_result_local.isUpdated()) {
                        TS_ASSERT(vu_result_local.isOneVariableActive());
                    } else {
                        TS_ASSERT(!vu_result_local.isOneVariableActive());
                    }

                } else if (get_action(result) == id::action_failure) {
                    //TS_ASSERT(!vu_result_local.isEmpty());
                    // don't metter if is updated or empty or not... in every case
                    // there should be no variable active
                    TS_ASSERT(!vu_result_local.isOneVariableActive());
                }

            } else {
                TS_ASSERT(false);
                return;
            }

            combo::UnifierIt it;
            combo::UnifierIt local_it;

            for (it = vu_result.begin(), local_it = vu_result_local.begin();
                    it != vu_result.end() && local_it != vu_result_local.end();
                    it++, local_it++) {
                printf("expectd %s - got %s\n", it->first.c_str(),
                       local_it->first.c_str());
                printf("expectd %s - got %s\n", (it->second ? "true" : "false"), ((*local_it).second ? "true" : "false"));
                TS_ASSERT((*it).first == local_it->first);
                TS_ASSERT((*it).second == local_it->second);
            }

        } else {

            std::cout << "Failed " << std::endl << "(" << ss1.str() << ")" << std::endl << "(" << ss2.str() << ")" << std::endl;
            TS_ASSERT(false);
        }
    }

};

#endif
