/*
 * opencog/embodiment/WorldWrapper/PAIWorldWrapper.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _PAIWORLDWRAPPER_H
#define _PAIWORLDWRAPPER_H

#include "WorldWrapper.h"
#include <opencog/util/exceptions.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

#include <exception>

namespace opencog { namespace world {

class PAIWorldWrapper : public WorldWrapperBase
{
public:

    PAIWorldWrapper(opencog::pai::PAI& _pai);
    ~PAIWorldWrapper();

    /**
     * @returns true is the current action plan is finished
     * false otherwise or if there is no action plan
     */
    bool isPlanFinished() const;

    /**
     * @pre the plan is finished
     * @returns true if the plan has failed false otherwise
     */
    bool isPlanFailed() const;

    /**
     * Send a sequence of sequential_and actions [from, to)
     * returns true iff and action plan gets executed
     */
    bool sendSequential_and(sib_it from, sib_it to) throw (opencog::ComboException,
            opencog::AssertionException,
            std::bad_exception);

    /**
     * evaluate a perception
     */
    combo::vertex evalPerception(pre_it per,
           combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    /**
     * evaluate an indefinite object
     */
    combo::vertex evalIndefiniteObject(combo::indefinite_object io,
           combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    /**
     * Create a navigation planning action sequence(including walk, jump
     * etc.) that will be sent to OAC.
     * @param actions Calculated path plan
     * @param useExistingID If this method get called two or more times and this
     * var is true, the new walks will be added to the same navigation plan
     * @param tuNudge nudge actions will be added to walk plan if it is not null
     */
    static bool createNavigationPlanAction( opencog::pai::PAI& pai,SpaceServer::SpaceMap& sm,
                                            const SpaceServer::SpaceMapPoint& startPoint,
                                            const SpaceServer::SpaceMapPoint& endPoint,
                                            opencog::pai::ActionPlanID _planID = "",
                                            float customSpeed = 0);

private:
    opencog::pai::PAI& pai;
    opencog::pai::ActionPlanID planID;

    // sometimes it is possible to know in advance whether the plan has failed
    // without sending it if _planHasFailed is true then we know for sure that
    // the plan has already failed otherwise we don't know
    bool _hasPlanFailed;

    /**
     * If pet is located at an invalid position, this method will return
     * a valid position
     * @param petLocation current pet location
     * @return a valid pet location
     */
    //spatial::Point getValidPosition( const spatial::Point& location );

    /**
     * This method will try to make path even smoother, reducing the number of
     * walks
     * @param actions walks processed by pathplanner
     * @param startPoint Current pet position
     * @param endPoint Goal position
     */
    /*void clearPlan( std::vector<spatial::Point>& actions,
                    const spatial::Point& startPoint,
                    const spatial::Point& endPoint );
    */
    /**
     * Given a start and an end point, return the waypoints necessary to travel between them
     *
     * @param startPoint Start point
     * @param endPoint End point
     * @param actions Return vector actions
     */
    //void getWaypoints( const spatial::Point& startPoint, const spatial::Point& endPoint, std::vector<spatial::Point>& actions );

//    /**
//     * Given a start and an end point, return the 3D waypoints necessary to
//     * travel between them.
//     *
//     * @param startPoint Start point
//     * @param endPoint End point
//     * @param actions Return the vector of 3d points.
//     */
//    void get3DWaypoints( const SpaceServer::SpaceMapPoint& startPoint, const SpaceServer::SpaceMapPoint& endPoint,
//                        std::vector<SpaceServer::SpaceMapPoint>& actions,SpaceServer::SpaceMap& sm);


    /**
     * Create a walk planning action that will be sent to OAC
     * @param actions Calculated path plan
     * @param useExistingID If this method get called two or more times
     * and this var is true, the new walks will be added to the same walk plan
     * @param tuNudge nudge actions will be added to walk plan if it is
     *                not null
     */
//    bool createWalkPlanAction( std::vector<spatial::Point>& actions,
//                               bool useExistingID = false,
//                               Handle toNudge = Handle::UNDEFINED,
//                               float customSpeed = 0);



    //! Builds plans for actions relying on goto (goto_obj, follow, etc)
    bool build_goto_plan(Handle goalHandle,
                         Handle goBehind = Handle::UNDEFINED,
                         float walkSpeed = 0);

    //! synthesize a PetAction obj from a subtree
    opencog::pai::PetAction buildPetAction(sib_it from);

    //! convert string to camelCase
    std::string toCamelCase(const std::string& str);

    //! type of a handle -> string
    std::string resolveType(combo::vertex);
    std::string resolveType(Handle);

    //! Convenience reference to string representing our avatar ("self" in combo)
    std::string selfName();

    //! Convenience reference to string representing the owner ("owner" in combo)
    std::string ownerName();

    //! Determine the angle an slobject is facing based on atomspace lookup
    double getAngleFacing(Handle) throw (opencog::ComboException,
                                         opencog::AssertionException,
                                         std::bad_exception);

    //! Convenience conversion function
    Handle toHandle(combo::definite_object);

    bool eval_percept_with_pai(pre_it it);

};

} } // namespace opencog::world

#endif
