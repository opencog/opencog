#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include "ActionPlanSenderMock.h"
#include "PetInterfaceMock.h"
#include <opencog/embodiment/Control/PredicateUpdaters/PredicatesUpdater.h>
#include <opencog/atomspace/SpaceServer.h>

#include <opencog/util/files.h>
#include <opencog/xml/NMXmlExporter.h>


using namespace PerceptionActionInterface;
using namespace OperationalPetController;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Wrong number of arguments\nUsage: %s <pvp_msg_xml_file1> [<pvp_msg_xml_file2>... [<pvp_msg_xml_fileN>]]\n", argv[0]);
        exit(-1);
    }
    config(Control::EmbodimentConfig::embodimentCreateInstance, true);
    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }

    AtomSpace atomSpace;

    HandleSeq toUpdateHandles;
    std::list<ActionPlan> sentActionPlans;
    OKActionPlanSender sender(sentActionPlans);
    PetInterfaceMock petInterface;
    PAI pai(atomSpace, sender, petInterface);
    petInterface.setPAI(&pai);
    PredicatesUpdater predicatesUpdater(atomSpace, petInterface.getPetId());
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

