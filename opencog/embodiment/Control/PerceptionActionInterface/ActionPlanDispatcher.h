/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionPlanDispatcher.h
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

#ifndef ACTIONPLANDISPATCHER_H_
#define ACTIONPLANDISPATCHER_H_
/**
 * ActionPlanDispatcher.h
 *
 * This class is useful for sending a list of actions (aka action plan)
 * and wait until the actions have finished or failed, despite of which individual
 * action failures or completion.
 */

#include "PAI.h"

namespace opencog { namespace pai {

class ActionPlanDispatcher
{

    PAI& pai;
    const std::list<AvatarAction>& actionPlan;
    ActionPlanID planId;
    bool badlyFailed;

public:
    ActionPlanDispatcher(PAI& _pai, const std::list<AvatarAction>&  _actionPlan);
    void dispatch();
    bool done();
    bool failed();
};

} } // namespace opencog::pai

#endif /*ACTIONPLANDISPATCHER_H_*/
