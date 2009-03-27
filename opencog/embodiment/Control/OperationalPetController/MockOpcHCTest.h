/**
 * MockOpcHCTest.h
 *
 * Author: Nil Geisweiller
 * creation : Thu Sep 13 2007
 */
#ifndef MOCKOPCHCTEST_H
#define MOCKOPCHCTEST_H

#include <string>
#include <opencog/atomspace/AtomSpace.h>
#include <StringMessage.h>
#include <EmbodimentCogServer.h>

#include "Pet.h"
#include "PetMessageSender.h"
#include "LearnMessage.h"
#include "HCTestAgent.h"
#include "SpaceServer.h"

using namespace opencog;

namespace OperationalPetController
{

class MockOpcHCTest : public MessagingSystem::EmbodimentCogServer
{

private:
    Pet * pet;
    AtomSpace * atomSpace;
    SpaceServer * spaceServer;
    PetMessageSender * lsMessageSender;

    HCTestAgent* _HCTa;

    //Handle to keep in memory for the second BD
    //fill the atomSpace with the initial scene
    Handle owner_h;
    Handle pet_h;
    Handle obj_h;
    Handle speed_h;
    //add necessary nodes to represent BDs
    Handle behaved_h;
    //fill atomSpace with actions goto_obj grab and wag
    Handle goto_obj_h;
    Handle grab_h;
    Handle wag_h;
    //add the concept of the trick
    Handle trick_h;
    //add AtTimeLink to it
    Handle tt1_h;
    //add first behavior, goto to stick and wag the taile (subject : owner)
    //(yes the owner wag its taile)
    //for goto_obj
    Handle argl1_h;
    Handle eval_goto_obj_h;
    Handle eval_grab_obj_h;
    Handle eval_wag_h;
    //add atTimeLink to it
    Handle ebd1_h;
    //add atTimeLink to it
    Handle ebdg_h;

    bool first_try;

public:

    static BaseServer* createInstance();

    MockOpcHCTest();
    void init(const std::string &myId, const std::string & ip,
              int portNumber, const std::string & petId,
              Control::SystemParameters & parameters);
    ~MockOpcHCTest();

    Factory<HCTestAgent,Agent> HCTestAgentFactory;

    /**
     * @return The AtomSpace that represents the pet's short memory
     */
    AtomSpace & getAtomSpace();

    /**
     * Return the pet's coginitive component. This component is responsable
     * for updating the pet attentionValues and choose a schema to
     * be executed.
     *
     * @return The pet's cognitive component.
     */
    Pet & getPet();

    /**
     * Method inherited from network element
     */
    bool processNextMessage(MessagingSystem::Message *msg);

    /**
     * Method inherited from network element
     */
    void setUp();

}; // class
}  // namespace

#endif
