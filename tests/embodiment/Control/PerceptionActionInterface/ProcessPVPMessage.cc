#include "PAI.h"
#include "SystemParameters.h"
#include "ActionPlanSenderMock.h"
#include "PetInterfaceMock.h"
#include "PredicatesUpdater.h"
#include <opencog/atomspace/SpaceServer.h>

#include "util/files.h"
#include <opencog/xml/NMXmlExporter.h>


using namespace PerceptionActionInterface;
using namespace OperationalPetController;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Wrong number of arguments\nUsage: %s <pvp_msg_xml_file1> [<pvp_msg_xml_file2>... [<pvp_msg_xml_fileN>]]\n", argv[0]);
        exit(-1);
    }
    Control::SystemParameters parameters;
    if (fileExists(parameters.get("CONFIG_FILE").c_str())) {
        parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    AtomSpace atomSpace;
    SpaceServer spaceServer(atomSpace);

    HandleSeq toUpdateHandles;
    std::list<ActionPlan> sentActionPlans;
    OKActionPlanSender sender(sentActionPlans);
    PetInterfaceMock petInterface;
    PAI pai(spaceServer, sender, petInterface, parameters);
    petInterface.setPAI(&pai);
    PredicatesUpdater predicatesUpdater(spaceServer, petInterface.getPetId());
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

    NMXmlExporter exporter;
    HandleEntry *handles = atomSpace.getAtomTable().getHandleSet(ATOM, true);
    std::string xml = exporter.toXML(handles);
    printf("%s\n", xml.c_str());
}

