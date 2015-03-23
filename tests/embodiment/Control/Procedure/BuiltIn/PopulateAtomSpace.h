/*
 * tests/embodiment/Control/Procedure/BuiltIn/PopulateAtomSpace.h
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
#ifndef POPULATEATOMSPACE_H
#define POPULATEATOMSPACE_H

#include <opencog/atomspace/AtomSpace.h>
#include "PAI.h"
#include "SystemParameters.h"
#include "AvatarInterfaceMock.h"
#include "PredicatesUpdater.h"
#include "ActionPlanSenderMock.h"

using namespace opencog::oac;
using namespace opencog;
using namespace opencog::pai;

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

class PopulateAtomSpace
{

private:

    AtomSpace atomSpace;
    control::SystemParameters parameters;
    PAI * pai;
    AvatarInterfaceMock * pet;
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
