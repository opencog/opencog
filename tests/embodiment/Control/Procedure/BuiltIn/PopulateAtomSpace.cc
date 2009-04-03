#include "PopulateAtomSpace.h"
#include "PAITestUtil.h"

PopulateAtomSpace::PopulateAtomSpace()
{

    pet = new PetInterfaceMock(atomSpace);
    pet->setOwnerId(owner);

    ap  = new FailureActionPlanSender();
    pai = new PAI(atomSpace, *ap, *pet, parameters);

    updater = new PredicatesUpdater(atomSpace, pet->getPetId());

    addOnwerInfo();
}

PopulateAtomSpace::~PopulateAtomSpace()
{
    delete updater;
    delete pai;
    delete pet;
    delete ap;
}

void PopulateAtomSpace::addOnwerInfo()
{

    // adding ownership relation
    Handle petHandle = atomSpace.addNode(SL_PET_NODE, petId);
    Handle ownerHandle = atomSpace.addNode(SL_AVATAR_NODE, owner);

    HandleSeq seq0;
    seq0.push_back(ownerHandle);
    seq0.push_back(petHandle);

    HandleSeq seq;
    seq.push_back(atomSpace.addNode(PREDICATE_NODE, "owns"));
    seq.push_back(atomSpace.addLink(LIST_LINK, seq0));

    atomSpace.addLink(EVALUATION_LINK, seq);
}

void PopulateAtomSpace::addSpaceInfoMock(int baseX, int baseY)
{
    handles.clear();
    unsigned long timestamp = PerceptionActionInterface::PAITestUtil::getCurrentTimestamp();

    // adding pet
    Handle objs = atomSpace.addNode(SL_PET_NODE, petId);
    atomSpace.addSpaceInfo(objs, timestamp, baseX, baseY, 5, 5, 0);
    addSizeInfo(objs, 5, 5);
    handles.push_back(objs);

    // adding owner
    objs = atomSpace.addNode(SL_AVATAR_NODE, owner);
    atomSpace.addSpaceInfo(objs, timestamp, baseX + 10, baseY + 10, 10, 10, 0);
    addSizeInfo(objs, 10, 10);
    handles.push_back(objs);

    // adding avatar
    objs = atomSpace.addNode(SL_AVATAR_NODE, avatar);
    atomSpace.addSpaceInfo(objs, timestamp, baseX + 15, baseY + 15, 10, 10, 0);
    addSizeInfo(objs, 10, 10);
    handles.push_back(objs);

    // adding stick
    objs = atomSpace.addNode(SL_ACCESSORY_NODE, stick);
    atomSpace.addSpaceInfo(objs, timestamp, baseX + 2, baseY + 2, 2, 2, 0);
    addSizeInfo(objs, 2, 2);
    handles.push_back(objs);

    // adding ball
    objs = atomSpace.addNode(SL_OBJECT_NODE, ball);
    atomSpace.addSpaceInfo(objs, timestamp, baseX - 1 , baseY - 1, 10, 10, 0);
    addSizeInfo(objs, 10, 10);
    handles.push_back(objs);

    // adding doll
    objs = atomSpace.addNode(SL_OBJECT_NODE, doll);
    atomSpace.addSpaceInfo(objs, timestamp, baseX - 3 , baseY - 3, 2, 2, 0);
    addSizeInfo(objs, 2, 2);
    handles.push_back(objs);

    // adding food
    objs = atomSpace.addNode(SL_OBJECT_NODE, food);
    atomSpace.addSpaceInfo(objs, timestamp, baseX + 4, baseY + 4, 2, 2, 0);
    addSizeInfo(objs, 2, 2);
    handles.push_back(objs);

    // adding ration
    objs = atomSpace.addNode(SL_OBJECT_NODE, ration);
    atomSpace.addSpaceInfo(objs, timestamp, baseX + 13, baseY + 13, 1.5, 1.5, 0);
    addSizeInfo(objs, 1.5, 1.5);
    handles.push_back(objs);

    // adding drink
    objs = atomSpace.addNode(SL_OBJECT_NODE, drink);
    atomSpace.addSpaceInfo(objs, timestamp, baseX + 2, baseY + 2, 5, 5, 0);
    addSizeInfo(objs, 5, 5);
    handles.push_back(objs);

    // adding milk
    objs = atomSpace.addNode(SL_OBJECT_NODE, milk);
    atomSpace.addSpaceInfo(objs, timestamp, baseX + 14, baseY + 14, 3, 3, 0);
    addSizeInfo(objs, 3, 3);
    handles.push_back(objs);

}

void PopulateAtomSpace::addSizeInfo(Handle obj, double length, double width)
{
    HandleSeq seq0;
    seq0.push_back(obj);
    seq0.push_back(atomSpace.addNode(NUMBER_NODE, toString(length)));
    seq0.push_back(atomSpace.addNode(NUMBER_NODE, toString(width)));

    HandleSeq seq;
    seq.push_back(atomSpace.addNode(PREDICATE_NODE, "size"));
    seq.push_back(atomSpace.addLink(LIST_LINK, seq0));

    atomSpace.addLink(EVALUATION_LINK, seq);
}

void  PopulateAtomSpace::callUpdater()
{
    updater->update(handles);
}

PAI & PopulateAtomSpace::getPAI()
{
    return *pai;
}

