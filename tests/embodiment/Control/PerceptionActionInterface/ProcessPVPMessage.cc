/*
 * tests/embodiment/Control/PerceptionActionInterface/ProcessPVPMessage.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include "ActionPlanSenderMock.h"
#include "AvatarInterfaceMock.h"
#include <opencog/embodiment/Control/PredicateUpdaters/PredicatesUpdater.h>
#include <opencog/atomspace/SpaceServer.h>

#include <opencog/util/files.h>
#include <opencog/persist/xml/NMXmlExporter.h>


using namespace opencog::pai;
using namespace opencog::oac;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Wrong number of arguments\nUsage: %s <pvp_msg_xml_file1> [<pvp_msg_xml_file2>... [<pvp_msg_xml_fileN>]]\n", argv[0]);
        exit(-1);
    }
    config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);
    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }

    AtomSpace atomSpace;

    HandleSeq toUpdateHandles;
    std::list<ActionPlan> sentActionPlans;
    OKActionPlanSender sender(sentActionPlans);
    AvatarInterfaceMock avatarInterface;
    PAI pai(atomSpace, sender, avatarInterface);
    avatarInterface.setPAI(&pai);
    PredicatesUpdater predicatesUpdater(atomSpace, avatarInterface.getPetId());
    for (int i = 1; i < argc; i++) {
        const char* xmlFileName = argv[1];
        string pvpMsg;
        if (appendFileContent(xmlFileName, pvpMsg)) {
            printf("Could not read content of file %s\n", xmlFileName);
        }
        printf("Processing pvp message:\n%s\n", pvpMsg.c_str());
        if (pai.processPVPMessage(pvpMsg, toUpdateHandles)) {
            predicatesUpdater.update(toUpdateHandles, pai.getLatestSimWorldTimestamp());
        }
    }

    NMXmlExporter exporter(&atomSpace);
    HandleSeq handles;
    atomSpace.getHandleSet(back_inserter(handles), ATOM, true);
    std::string xml = exporter.toXML(handles);
    printf("%s\n", xml.c_str());
}

