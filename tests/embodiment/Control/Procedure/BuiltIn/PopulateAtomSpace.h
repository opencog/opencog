#ifndef POPULATEATOMSPACE_H
#define POPULATEATOMSPACE_H

#include <opencog/atomspace/AtomSpace.h>
#include "PAI.h"
#include "SystemParameters.h"
#include "PetInterfaceMock.h"
#include "PredicatesUpdater.h"
#include "ActionPlanSenderMock.h"

using namespace OperationalPetController;
using namespace PerceptionActionInterface;

static const std::string petId  = "1";
static const std::string owner  = "Dudu";
static const std::string avatar = "Carlos";
static const std::string stick  = "stick";
static const std::string ball   = "ball";
static const std::string doll   = "doll";
static const std::string food   = "meat";
static const std::string ration = "ration";
static const std::string drink  = "water";
static const std::string milk   = "milk";

class PopulateAtomSpace {

    private:

		AtomSpace atomSpace;
        Control::SystemParameters parameters;
        PAI * pai;
		PetInterfaceMock * pet;
		PredicatesUpdater * updater;
	    FailureActionPlanSender *ap;

        HandleSeq handles;

        /**
         * add owns predicate between pet and avatar
         */
        void addOnwerInfo();

        /**
         * add size predicate for the given obj.
         */
		void addSizeInfo(Handle obj, double length, double width);

	public:

		PopulateAtomSpace();
		~PopulateAtomSpace();

        /**
         * add space info for the objs in the map, based on the x, y coords
         * passed.
         */
        void addSpaceInfoMock(int baseX, int baseY);

        /**
         * call predicates updater
         */
        void callUpdater();

        /**
         * returnt the PAI component
         */
        PAI& getPAI();
};

#endif
