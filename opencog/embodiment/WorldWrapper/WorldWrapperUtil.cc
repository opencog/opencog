/*
 * opencog/embodiment/WorldWrapper/WorldWrapperUtil.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller, Moshe Looks, Carlos Lopes
 *
 * Updated: by ZhenhuaCai, on 2011-02-08
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


#include <sstream>
#include <ext/functional>
#include <typeinfo>

#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>

#include <opencog/util/functional.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/SpaceTime.h>

#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>

#include "WorldWrapperUtil.h"

using namespace AvatarCombo;
using namespace opencog;
using namespace opencog::pai;
// This is already available in opencog::world, but this is placed here to
// make it obvious where predicate comes from
using opencog::world::predicate;

namespace opencog { namespace world {

typedef combo_tree::iterator pre_it;
typedef combo_tree::sibling_iterator sib_it;

WorldWrapperUtilCache WorldWrapperUtil::cache;

const float WorldWrapperUtil::meanTruthThreshold = 0.5;

definite_object WorldWrapperUtil::atom_name_to_definite_object(const string& atom_name, const string& self_id, const string& owner_id)
{
    if (atom_name == self_id)
        return definite_object(id::self);
    else if (atom_name == owner_id)
        return definite_object(id::owner);
    else {
        return definite_object(atom_name);
    }
}

string WorldWrapperUtil::definite_object_to_atom_name(const definite_object& d,
        const string& self_id,
        const string& owner_id)
{
    if (d == id::self)
        return self_id;
    else if (d == id::owner)
        return owner_id;
    else return string(d);
}

bool WorldWrapperUtil::definite_object_equal(const definite_object& d1,
        const definite_object& d2,
        const string& self_id,
        const string& owner_id)
{
    if (d1 == d2)
        return true;
    else {
        definite_object dt1 = atom_name_to_definite_object(d1,
                              self_id,
                              owner_id);
        definite_object dt2 = atom_name_to_definite_object(d2,
                              self_id,
                              owner_id);
        return dt1 == d2 || d1 == dt2 || dt1 == dt2;
    }
}

Handle WorldWrapperUtil::toHandle(AtomSpace& as,
                                  definite_object obj,
                                  const string& self_id,
                                  const string& owner_id)
{
    string id = definite_object_to_atom_name(obj, self_id, owner_id);
    Handle h = AtomSpaceUtil::getObjectHandle(as, id);
    if (h == Handle::UNDEFINED) { // not an object, let try if it's an agent
        h = AtomSpaceUtil::getAgentHandle(as, id);
    }
    if (h == Handle::UNDEFINED) {
        //Nil: I add this case because apparently some object have type NODE
        HandleSeq tmp;
        as.getHandlesByName(std::back_inserter(tmp), id, OBJECT_NODE);
        //unique id assumption
        OC_ASSERT(tmp.size() <= 1);
        return tmp.empty() ? Handle::UNDEFINED : tmp.front();
    }
    return h;
}

std::string WorldWrapperUtil::lookupInheritanceLink(
        const AtomSpace& as,
        Handle h)
{

    //look for X satisfying InheritanceLinkLink(X, h)

    HandleSeq seq;
    seq.push_back(Handle::UNDEFINED);
    seq.push_back(h);

    std::vector<Handle> res;
    as.getHandlesByOutgoing(back_inserter(res),
                    seq, NULL, NULL, 2, INHERITANCE_LINK, false);
    if (res.empty())
        return id::null_obj;

    // return the first element of the outgoing set of the inheritance link
    return as.getName(as.getOutgoing(res[randGen().randint(res.size())],0));
}

std::string WorldWrapperUtil::lookupExecLink(
        const AtomSpace& as,
        Handle h)
{
    //look for x satisfying ExecutionLink(h,ListLink(),ListLink(x))
    std::vector<Handle> res;
    std::vector<Handle> match(3);
    match[0] = h;
    Type t[] = { PREDICATE_NODE, LIST_LINK, LIST_LINK };
    as.getHandlesByOutgoing(back_inserter(res), match, t,
                    NULL, 3, EXECUTION_LINK, true);
    if (res.empty())
        return id::null_obj;
    return (as.getName(res[randGen().randint(res.size())]));
}

pre_it WorldWrapperUtil::maketree(string str, std::string h)
{
    stringstream ss;
    vertex v;
    ss << str;
    ss >> v;
    return maketree_vertex(v, h);
}

pre_it WorldWrapperUtil::maketree_percept(perception p, std::string h)
{
    vertex v(p);
    return maketree_vertex(v, h);
}

pre_it WorldWrapperUtil::maketree_vertex(const vertex& v, std::string h)
{
    static combo_tree tmp;
    tmp = combo_tree(v);
    tmp.append_child(tmp.begin(), h);
    return tmp.begin();
}

Handle WorldWrapperUtil::rec_lookup(AtomSpace& as, pre_it it,
                                    const string& self_id,
                                    const string& owner_id)
{
    stringstream ss;
    ss << (*it);

    //logger().error("rec_lookup <%s>",xx.str().c_str());
    //assert(is_definite_object(*it));

    definite_object obj = is_definite_object(*it) ?
                          get_definite_object(*it) : definite_object(ss.str());

    //search first for an OBJECT_NODE
    if (it.is_childless()) {
        Handle h = toHandle(as, obj, self_id, owner_id);
        if (h != Handle::UNDEFINED) {
            return h;
        }
    }

    //otherwise search for a predicate node
    /*
    HandleSeq tmp;
    as.getHandleByName(std::back_inserter(tmp), obj, PREDICATE_NODE);

//    if(tmp.size()==0) //do a dump before failing
//      as.print();

    OC_ASSERT(tmp.size() <= 1); //need to assume that PredicateNode names are unique
    */
    Handle lhs = as.getHandle( PREDICATE_NODE, obj );
    if (lhs == Handle::UNDEFINED ) {
        logger().debug(
                     "WWUtil - rec_lookup found no predicate node for '%s'.",
                     obj.c_str());
        return Handle::UNDEFINED;
    }
    
    //link lookup
    
    HandleSeq children;	
    for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
        children.push_back(rec_lookup(as, pre_it(sib), self_id, owner_id));
        if (children.back() == Handle::UNDEFINED) {
            return Handle::UNDEFINED;
        }
    }

    Handle rhs = as.getHandle(LIST_LINK, children);
    HandleSeq args{lhs, rhs};
    return (rhs != Handle::UNDEFINED ?
            as.getHandle(EVALUATION_LINK, args) : Handle::UNDEFINED);
}

Handle WorldWrapperUtil::selfHandle(AtomSpace& as,
                                    const string& self_id)
{
    //check if it's corresponding to a pet
    return AtomSpaceUtil::getAgentHandle( as, self_id );
}

Handle WorldWrapperUtil::ownerHandle(AtomSpace& as,
                                     const string& owner_id)
{
    return getAvatarHandle(as, owner_id);
}

Handle WorldWrapperUtil::getAvatarHandle(AtomSpace& as,
        const string& avatar_id)
{
    return as.getHandle(AVATAR_NODE, avatar_id);
}

bool WorldWrapperUtil::inSpaceMap(const SpaceServer::SpaceMap& sm,
                                  AtomSpace& as,
                                  const string& self_id,
                                  const string& owner_id,
                                  vertex v)
{
    OC_ASSERT(is_definite_object(v));

    OC_ASSERT(toHandle(as, get_definite_object(v), self_id, owner_id)
              != Handle::UNDEFINED,
              "WorldWrapperUtil - Null handle for definite objetc %s.",
              get_definite_object(v).c_str());

    return sm.containsObject(toHandle(as, get_definite_object(v),
                                        self_id, owner_id));
}

vertex WorldWrapperUtil::evalIndefiniteObject(
        Handle smh,
        unsigned long time,
        AtomSpace& atomSpace,
        const string& self_id,
        const string& owner_id,
        combo::indefinite_object io,
        bool isInThePast,
        combo::variable_unifier& vu)
throw (opencog::ComboException,
       opencog::AssertionException,
       std::bad_exception)
{

    return evalIndefiniteObject(smh, time, atomSpace,
                                self_id, owner_id, get_enum(io),
                                isInThePast, vu);

}

vertex WorldWrapperUtil::evalIndefiniteObject(
        Handle smh,
        unsigned long time,
        AtomSpace& atomSpace,
        const string& self_id,
        const string& owner_id,
        combo::avatar_indefinite_object_enum ioe,
        bool isInThePast,
        combo::variable_unifier& vu)
throw (opencog::ComboException,
       opencog::AssertionException,
       std::bad_exception)
{

    std::string res;

    switch (ioe) {
    default: {
        printf("switch(ioe) defaulted..\n");
        {
            std::stringstream stream (std::stringstream::out);
            stream << "Unrecognized indefinite object '"
                   << vertex(ioe) << "'" << std::endl;
            throw opencog::ComboException(TRACE_INFO,
                                          "WorldWrapperUtil - %s.",
                                          stream.str().c_str());
        }
    }
    }

    /*
    SpaceServer::SpaceMapPoint selfLoc;

    OC_ASSERT(
                     smh != Handle::UNDEFINED,
                     "A SpaceMap must exist");
    const SpaceServer::SpaceMap& sm = atomSpace.getSpaceServer().getMap(smh);

    try {
        selfLoc = getLocation(sm, atomSpace, selfHandle(atomSpace,self_id));

    } catch (...) {
        // TODO: check if it is necessary to catch the exceptions separately.
        return vertex("null_obj");
    }
    perception p;//used for nearest_X and random_X indefinite objects
    indefinite_object io = get_instance(ioe);
    switch (ioe) {
    case id::nearest_object: {

        //retrieve the possible holded object
        std::string objectId;
        if (isInThePast) {
            objectId =
                AtomSpaceUtil::getHoldingObjectIdAtTime(atomSpace, self_id, time);
        } else {
            objectId =
                AtomSpaceUtil::getHoldingObjectId(atomSpace, self_id);
        }
        //if
        if ( objectId != "" ) {
            logger().debug(
                         "WorldWrapperUtil - isHolding (%s)",
                         objectId.c_str());
            res = sm.findNearestFiltered
                  (selfLoc,
                   boost::bind(std::logical_and<bool>(),
                               boost::bind(std::not_equal_to<std::string>(),
                                           _1,
                                           objectId),
                               boost::bind(std::not_equal_to<std::string>(),
                                           _1,
                                           self_id) ) );
        } else {
            logger().debug(
                          "WorldWrapperUtil - not holding any object" );
            res = sm.findNearestFiltered
                  (selfLoc,
                   std::bind2nd( std::not_equal_to<std::string>(), self_id )
                  );
        } // if

    }
    break;
    case id::nearest_edible: {
        p = nearest_random_X_to_is_X(ioe);

        logger().debug(
                      "WorldWrapperUtil - nearest edible.");
        res = sm.findNearestFiltered
              (selfLoc,
               boost::bind(&combo::vertex_to_bool,
                           boost::bind(&WorldWrapperUtil::evalPerception,
                                       smh, time, boost::ref(atomSpace),
                                       boost::cref(self_id),
                                       boost::cref(owner_id),
                                       boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                       isInThePast, vu)));
    }
    break;
    case id::nearest_movable:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findNearestFiltered
              (selfLoc,
               boost::bind(&combo::vertex_to_bool,
                           boost::bind(&WorldWrapperUtil::evalPerception,
                                       smh, time, boost::ref(atomSpace),
                                       boost::cref(self_id),
                                       boost::cref(owner_id),
                                       boost::bind(&WorldWrapperUtil::maketree_percept,
                                                   p, _1),
                                       isInThePast, vu)));
        break;
    case id::nearest_pickupable: {
        std::string objectId;
        if (isInThePast) {
            objectId =
                AtomSpaceUtil::getHoldingObjectIdAtTime( atomSpace,
                        self_id, time );
        } else {
            objectId =
                AtomSpaceUtil::getHoldingObjectId( atomSpace,
                                                   self_id );
        }
        p = nearest_random_X_to_is_X(ioe);
        if ( objectId != "" ) {
            logger().debug(
                         "WorldWrapperUtil - isHolding (%s)",
                         objectId.c_str());
            res = sm.findNearestFiltered(selfLoc,
                                         boost::bind<bool>(std::logical_and<bool>(),
                                                           boost::bind<bool>(std::logical_and<bool>(),
                                                                             boost::bind( std::not_equal_to<std::string>(), _1, objectId ),
                                                                             boost::bind( std::not_equal_to<std::string>(), _1, self_id )),
                                                           boost::bind<bool>(combo::vertex_to_bool,
                                                                             boost::bind(&WorldWrapperUtil::evalPerception,
                                                                                         smh, time, boost::ref(atomSpace),
                                                                                         boost::cref(self_id),
                                                                                         boost::cref(owner_id),
                                                                                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                                                                         isInThePast, vu))));

            logger().debug(
                         "WorldWrapperUtil - Nearest filtered (%s)",
                         res.c_str( ) );
        } else {
            logger().debug(
                         "WorldWrapperUtil - not holding any object");
            res = sm.findNearestFiltered
                  (selfLoc,
                   boost::bind(&combo::vertex_to_bool,
                               boost::bind(&WorldWrapperUtil::evalPerception,
                                           smh, time, boost::ref(atomSpace),
                                           boost::cref(self_id),
                                           boost::cref(owner_id),
                                           boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                           isInThePast, vu)));
        } // if
    }
    break;
    case id::nearest_drinkable:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findNearestFiltered
              (selfLoc,
               boost::bind(&combo::vertex_to_bool,
                           boost::bind(&WorldWrapperUtil::evalPerception,
                                       smh, time, boost::ref(atomSpace),
                                       boost::cref(self_id),
                                       boost::cref(owner_id),
                                       boost::bind(&WorldWrapperUtil::maketree_percept,
                                                   p, _1),
                                       isInThePast, vu)));
        break;
    case id::nearest_avatar: //this is more complex than the others to make sure we don't return the avatar to imitation when the pet takes its skin
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findNearestFiltered
              (selfLoc,
               bind(std::logical_and<bool>(),
                    boost::bind(std::not_equal_to<std::string>(), self_id, _1),
                    boost::bind(&combo::vertex_to_bool,
                                boost::bind(&WorldWrapperUtil::evalPerception,
                                            smh, time,
                                            boost::ref(atomSpace),
                                            boost::cref(self_id),
                                            boost::cref(owner_id),
                                            bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                            isInThePast, vu))));
        break;
    case id::nearest_small:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findNearestFiltered
              (selfLoc,
               boost::bind(&combo::vertex_to_bool,
                           boost::bind(&WorldWrapperUtil::evalPerception,
                                       smh, time,
                                       boost::ref(atomSpace),
                                       boost::cref(self_id),
                                       boost::cref(owner_id),
                                       boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                       isInThePast, vu) ) );
        break;
    case id::nearest_moving: //this is more complex than the others to make sure we don't return the subject in case it is moving
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findNearestFiltered
              (selfLoc,
               boost::bind(std::logical_and<bool>(),
                           boost::bind(std::not_equal_to<std::string>(), self_id, _1),
                           boost::bind(&combo::vertex_to_bool,
                                       boost::bind(&WorldWrapperUtil::evalPerception,
                                                   smh, time,
                                                   boost::ref(atomSpace),
                                                   boost::cref(self_id),
                                                   boost::cref(owner_id),
                                                   bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                                   isInThePast, vu) ) ) );
        break;
    case id::nearest_noisy: //this is more complex than the others to make sure we don't return the subject in case it is noisy
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findNearestFiltered
              (selfLoc,
               boost::bind(std::logical_and<bool>(),
                           boost::bind(std::not_equal_to<std::string>(),
                                       self_id,
                                       _1),
                           boost::bind(&combo::vertex_to_bool,
                                       boost::bind(&WorldWrapperUtil::evalPerception,
                                                   smh, time,
                                                   boost::ref(atomSpace),
                                                   boost::cref(self_id),
                                                   boost::cref(owner_id),
                                                   bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                                   isInThePast, vu))));
        break;
    case id::random_object:
        res = sm.findRandomFiltered(make_const_function(true));
        break;
    case id::random_edible:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findRandomFiltered
              (bind(&combo::vertex_to_bool,
                    bind(&WorldWrapperUtil::evalPerception,
                         smh, time, boost::ref(atomSpace),
                         boost::cref(self_id),
                         boost::cref(owner_id),
                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                         isInThePast, vu)));
        break;
    case id::random_movable:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findRandomFiltered
              (bind(&combo::vertex_to_bool,
                    bind(&WorldWrapperUtil::evalPerception,
                         smh, time, boost::ref(atomSpace),
                         boost::cref(self_id),
                         boost::cref(owner_id),
                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                         isInThePast, vu)));
        break;
    case id::random_pickupable: {
        std::string objectId = AtomSpaceUtil::getHoldingObjectId( atomSpace, self_id );

        p = nearest_random_X_to_is_X(ioe);
        if ( objectId != "" ) {
            logger().debug("WorldWrapperUtil - isHolding (%s)", objectId.c_str( ) );
            res = sm.findRandomFiltered(boost::bind<bool>(std::logical_and<bool>(),
                                        boost::bind<bool>(std::logical_and<bool>(),
                                                          boost::bind( std::not_equal_to<std::string>(),
                                                                       _1,
                                                                       objectId ),
                                                          boost::bind( std::not_equal_to<std::string>(),
                                                                       _1,
                                                                       self_id )),
                                        boost::bind<bool>(combo::vertex_to_bool,
                                                          bind(&WorldWrapperUtil::evalPerception,
                                                               smh, time, boost::ref(atomSpace),
                                                               boost::cref(self_id),
                                                               boost::cref(owner_id),
                                                               boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                                                               isInThePast,
vu))));

            logger().debug(
                         "WorldWrapperUtil - Random pickupable filtered (%s)",
                         res.c_str( ) );
        } else {
            logger().debug(
                         "WorldWrapperUtil - Pet isn't holding an object");
            res = sm.findRandomFiltered
                  (bind(&combo::vertex_to_bool,
                        bind(&WorldWrapperUtil::evalPerception,
                             smh, time, boost::ref(atomSpace),
                             boost::cref(self_id),
                             boost::cref(owner_id),
                             boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                             isInThePast, vu)));
        } // if
        logger().debug(
                      "WorldWrapperUtil - Selected random pickupable %s",
                      res.c_str( ) );
    }
    break;
    case id::random_drinkable:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findRandomFiltered
              (bind(&combo::vertex_to_bool,
                    bind(&WorldWrapperUtil::evalPerception,
                         smh, time, boost::ref(atomSpace),
                         boost::cref(self_id),
                         boost::cref(owner_id),
                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                         isInThePast, vu)));
        break;
    case id::random_avatar:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findRandomFiltered
              (bind(&combo::vertex_to_bool,
                    bind(&WorldWrapperUtil::evalPerception,
                         smh, time, boost::ref(atomSpace),
                         boost::cref(self_id),
                         boost::cref(owner_id),
                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                         isInThePast, vu)));
        break;
    case id::random_small:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findRandomFiltered
              (bind(&combo::vertex_to_bool,
                    bind(&WorldWrapperUtil::evalPerception,
                         smh, time, boost::ref(atomSpace),
                         boost::cref(self_id),
                         boost::cref(owner_id),
                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                         isInThePast, vu)));
        break;
    case id::random_moving:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findRandomFiltered
              (bind(&combo::vertex_to_bool,
                    bind(&WorldWrapperUtil::evalPerception,
                         smh, time, boost::ref(atomSpace),
                         boost::cref(self_id),
                         boost::cref(owner_id),
                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                         isInThePast, vu)));
        break;
    case id::random_noisy:
        p = nearest_random_X_to_is_X(ioe);
        res = sm.findRandomFiltered
              (bind(&combo::vertex_to_bool,
                    bind(&WorldWrapperUtil::evalPerception,
                         smh, time, boost::ref(atomSpace),
                         boost::cref(self_id),
                         boost::cref(owner_id),
                         boost::bind(&WorldWrapperUtil::maketree_percept, p, _1),
                         isInThePast, vu)));
        break;
    case id::last_food_place:
        res = lookupExecLink(atomSpace, toHandle(atomSpace, "last_food_place",
                             self_id, owner_id));
        break;
    case id::exemplar_avatar: {
        Handle evalLink = AtomSpaceUtil::getMostRecentEvaluationLink(atomSpace, "is_exemplar_avatar");

        if (evalLink != Handle::UNDEFINED) {
            if (atomSpace.getMean(evalLink) > meanTruthThreshold) {
                Handle ll = atomSpace.getOutgoing(evalLink, 1);
                if (ll != Handle::UNDEFINED) {
                    Handle teacher = atomSpace.getOutgoing(ll, 0);
                    Handle student = atomSpace.getOutgoing(ll, 1);

                    if (teacher != Handle::UNDEFINED
                            && self_id == atomSpace.getName(student)) {
                        return vertex(atomSpace.getName(teacher));
                    }
                }
            }
        }
        return vertex(id::null_obj);
    }
    break;

    default:
        std::stringstream stream (std::stringstream::out);
        stream << "Unrecognized indefinite object '"
        << vertex(io) << "'" << std::endl;
        throw opencog::ComboException(TRACE_INFO,
                                      "WorldWrapperUtil - %s.",
                                      stream.str().c_str());
    }

    std::stringstream ss;
    ss << io;
*/
    if (res == "")
        res = id::null_obj;
    //if(res==self_id)
    //  res = id::self;
    //else if(res==owner_id)
    //  res = id::owner;

//    logger().debug(
//                 "WorldWrapperUtil - Analyzing '%s'. Result: '%s'.",
//                 ss.str().c_str(), res.c_str());

    return vertex(res);
}

combo::vertex WorldWrapperUtil::evalPerception(
        Handle smh,
        unsigned long time,
        AtomSpace& atomSpace,
        const string& self_id,
        const string& owner_id,
        const pre_it it,
        bool isInThePast,
        combo::variable_unifier& vu)
{

    OC_ASSERT(
                     is_perception(*it),
                     "It is assumed that '*it' is a perception");

    perception p = get_perception(*it);
    if (p == get_instance(id::is_null)) { //just check directly
        OC_ASSERT(it.number_of_children() == 1);

        combo::vertex v = *it.begin();
        if (is_indefinite_object(v)) { //eval indefinite object of argument
            v = WorldWrapperUtil::evalIndefiniteObject(
                    smh,
                    time,
                    atomSpace,
                    self_id, owner_id,
                    get_indefinite_object(v),
                    isInThePast);
        }

        //std::cout << "is_null(" << *it.begin() << ") = " << (get_definite_object(v) == id::null_obj ? "true" : "false") << std::endl;
        return combo::bool_to_vertex(get_definite_object(v) == id::null_obj);

    } else if (p == get_instance(id::exists)) {
        OC_ASSERT(
                         it.number_of_children() == 1);
        OC_ASSERT(
                         smh != Handle::UNDEFINED,
                         "A SpaceMap must exist");
        const SpaceServer::SpaceMap& sm = spaceServer().getMap(smh);
        return combo::bool_to_vertex(inSpaceMap(sm, atomSpace, self_id, owner_id,
                                                *it.begin()));
    }

    // temporary tree used to copy perception to
    // not modify it
    combo_tree tmp(it);

    avatar_perception_enum pe = get_enum(p);
    switch (pe) { //could be made more efficient if needed

    case id::exists_edible:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_edible)
                                      != vertex(id::null_obj)));
    case id::exists_movable:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_movable)
                                      != vertex(id::null_obj)));
    case id::exists_pickupable:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_pickupable)
                                      != vertex(id::null_obj)));
    case id::exists_drinkable:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_drinkable)
                                      != vertex(id::null_obj)));
    case id::exists_avatar:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_avatar)
                                      != vertex(id::null_obj)));
    case id::exists_small:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_small)
                                      != vertex(id::null_obj)));
    case id::exists_moving:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_moving)
                                      != vertex(id::null_obj)));
    case id::exists_noisy:
        return combo::bool_to_vertex((evalIndefiniteObject(smh, time,
                                      atomSpace,
                                      self_id, owner_id,
                                      id::random_noisy)
                                      != vertex(id::null_obj)));
    case id::exists: {
        assert(it.number_of_children() == 1);
        vertex vo = *it.begin();
        std::vector<definite_object> definite_objects =
            WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                 atomSpace,
                                                 self_id, owner_id,
                                                 vo, isInThePast, vu);

        if ( definite_objects.size( ) != 1 ) {
            // there is no definite object, so return false immediately
            return combo::bool_to_vertex(false);
        } // if

        // cache predicate data variables
        std::string name = p->get_name();
        std::vector<std::string> argument;

        combo::definite_object& def_obj = definite_objects[0];
        argument.push_back(std::string(def_obj));
        predicate pred(name, argument);
        float data = WorldWrapperUtil::cache.find(time, pred);

        Handle handle = atomSpace.getHandle(OBJECT_NODE,
                                            def_obj);

        bool result;
        if (data != CACHE_MISS) {
            result = (data == 1.0f);
        } else {

                // if the definite object exists inside the latest map it exists
                bool isExist = spaceServer().getLatestMap( ).containsObject(handle);
                if (isExist)
                result = true;
                else
                // wheter not found inside map, it doesn't exists
                result = false;
            }

            // cache miss, compute value and cache it
            if (result) {
                WorldWrapperUtil::cache.add(time, pred, 1.0f);
            } else {
                WorldWrapperUtil::cache.add(time, pred, 0.0f);
            } // else

            return combo::bool_to_vertex(result);
        } // else

    // is_avatar - handle directly via type inference
    case id::is_avatar:
        assert(it.number_of_children() == 1);
        {
            vertex vo = *it.begin();
            std::vector<definite_object> definite_objects =
                WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                     atomSpace,
                                                     self_id, owner_id, vo,
                                                     isInThePast, vu);

            // cache predicate data variables
            std::string name = p->get_name();
            std::vector<std::string> argument;

            bool general_result = false;
            foreach(combo::definite_object def_obj, definite_objects) {

                argument.clear();
                argument.push_back(std::string(def_obj));
                predicate pred(name, argument);
                float data = WorldWrapperUtil::cache.find(time, pred);

                bool result;
                if (data != CACHE_MISS) {
                    result = (data > meanTruthThreshold);

                } else {
                    result = classserver().isA(atomSpace.getType(toHandle(atomSpace,
                                                         def_obj,
                                                         self_id,
                                                         owner_id)),
                                                        AVATAR_NODE);

                    // cache miss, compute value and cache it
                    if (result) {
                        WorldWrapperUtil::cache.add(time, pred, 1.0f);
                    } else {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                    }
                }

                if (is_wild_card(vo)) {
                    vu.setVariableState(def_obj, result);
                }
                if (result) {
                    general_result = true;
                }
            }

            if (is_wild_card(vo)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }

    case id::is_object:
        assert(it.number_of_children() == 1);
        {
            vertex vo = *it.begin();
            std::vector<definite_object> definite_objects =
                WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                     atomSpace,
                                                     self_id, owner_id, vo,
                                                     isInThePast, vu);

            // cache predicate data variables
            std::string name = p->get_name();
            std::vector<std::string> argument;

            bool general_result = false;
            foreach(combo::definite_object def_obj, definite_objects) {

                argument.clear();
                argument.push_back(std::string(def_obj));
                predicate pred(name, argument);
                float data = WorldWrapperUtil::cache.find(time, pred);

                bool result;
                if (data != CACHE_MISS) {
                    result = (data > meanTruthThreshold);

                } else {
                    Handle obj_handle = WorldWrapperUtil::toHandle(atomSpace,
                                        def_obj,
                                        self_id,
                                        owner_id);

                    // for the time being all nodes that are no avatar or structure
                    // are considered objects
                    result =
                        !classserver().isA(atomSpace.getType(obj_handle),
                                                AVATAR_NODE)
                        &&
                        !classserver().isA(atomSpace.getType(obj_handle),
                                                STRUCTURE_NODE);

                    // cache miss, compute value and cache it
                    if (result) {
                        WorldWrapperUtil::cache.add(time, pred, 1.0f);
                    } else {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                    }
                }

                if (is_wild_card(vo)) {
                    vu.setVariableState(def_obj, result);
                }
                if (result) {
                    general_result = true;
                }
            }

            if (is_wild_card(vo)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }

        //is_moving predicate
    /*case id::is_moving: {
        if (isInThePast) {
            OC_ASSERT(it.number_of_children() == 1);
            vertex vo = *it.begin();
            std::vector<definite_object> definite_objects =
                WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                     atomSpace,
                                                     self_id, owner_id, vo,
                                                     isInThePast, vu);

            OC_ASSERT(
                             is_definite_object(vo),
                             "Should be of type definite object");

            const SpaceServer& spaceServer = atomSpace.getSpaceServer();
            Handle pre_smh = spaceServer.getPreviousMapHandle(smh);
            if (pre_smh == Handle::UNDEFINED) {
                return id::logical_false;
            }
            const SpaceServer::SpaceMap& pre_sm = spaceServer.getMap(pre_smh);
            const SpaceServer::SpaceMap& sm = spaceServer.getMap(smh);


            // cache predicate data variables
            std::string name = p->get_name();
            std::vector<std::string> argument;

            bool general_result = false;
            foreach(combo::definite_object def_obj, definite_objects) {

                argument.clear();
                argument.push_back(std::string(def_obj));
                predicate pred(name, argument);
                float data = WorldWrapperUtil::cache.find(time, pred);

                bool result;
                if (data != CACHE_MISS) {
                    result = (data > meanTruthThreshold);

                } else {

                    Handle obj_h = toHandle(atomSpace, def_obj, self_id, owner_id);
                    result = AtomSpaceUtil::isMovingBtwSpaceMap(atomSpace,
                             pre_sm,
                             sm,
                             obj_h);

                    // cache miss, compute value and cache it
                    if (result) {
                        WorldWrapperUtil::cache.add(time, pred, 1.0f);
                    } else {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                    }
                }

                if (is_wild_card(vo)) {
                    vu.setVariableState(def_obj, result);
                }
                if (result) {
                    general_result = true;
                }
            }
            if (is_wild_card(vo)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }
    }
    break; //if not in the past then get off the case
    */
    case id::is_holding_something: {
        OC_ASSERT(it.number_of_children() == 1,
                         "is_holding_something perception takes"
                         " only one argument and got %d",
                         it.number_of_children());
        if (isInThePast) {
            OC_ASSERT(false, "not implemented yet");
        } else {
            vertex vo = *it.begin();
            std::vector<definite_object> definite_objects =
                WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                     atomSpace,
                                                     self_id, owner_id, vo,
                                                     isInThePast, vu);

            // cache predicate data variables
            std::string name = p->get_name();
            std::vector<std::string> argument;

            bool general_result = false;
            foreach(combo::definite_object def_obj, definite_objects) {

                argument.clear();
                argument.push_back(std::string(def_obj));
                predicate pred(name, argument);
                float data = WorldWrapperUtil::cache.find(time, pred);

                bool result;
                if (data != CACHE_MISS) {
                    result = (data > meanTruthThreshold);

                } else {

                    result = AtomSpaceUtil::isHoldingSomething(atomSpace,
                             def_obj);

                    // cache miss, compute value and cache it
                    if (result) {
                        WorldWrapperUtil::cache.add(time, pred, 1.0f);
                    } else {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                    }
                }

                if (is_wild_card(vo)) {
                    vu.setVariableState(def_obj, result);
                }
                if (result) {
                    general_result = true;
                }
            }

            if (is_wild_card(vo)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }
    }
    break;

    //spatial predicates
    case id::near:
    case id::inside:
    case id::above:
    case id::below: {
        // no cache here since this case only care with past occurencies.
        // present occurancies are taken care as default
        if (isInThePast) {
            OC_ASSERT(it.number_of_children() == 2,
                             "WWUtil - near - wrong number of arguments (should be 2) : %d",
                             it.number_of_children());
            {
                sib_it sib_arg = it.begin();

                vertex vo1 = *sib_arg;
                std::vector<combo::definite_object> vo1_definite_objects =
                    WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                         atomSpace,
                                                         self_id,
                                                         owner_id,
                                                         vo1,
                                                         isInThePast,
                                                         vu);

                vertex vo2 = *(++sib_arg);
                std::vector<combo::definite_object> vo2_definite_objects =
                    WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                         atomSpace,
                                                         self_id,
                                                         owner_id,
                                                         vo2,
                                                         isInThePast,
                                                         vu);

                bool general_result = false;
                foreach(combo::definite_object vo1_def_obj,
                        vo1_definite_objects) {
                    Handle h1 = toHandle(atomSpace, vo1_def_obj, self_id, owner_id);
                    OC_ASSERT(
                                     h1 != Handle::UNDEFINED,
                                     "WWUtil - near - Handle h1 should not be Handle::UNDEFINED.");

                    foreach(combo::definite_object vo2_def_obj,
                            vo2_definite_objects) {

                        Handle h2 = toHandle(atomSpace, vo2_def_obj,
                                             self_id, owner_id);

                        OC_ASSERT(h1 != Handle::UNDEFINED, "WWUtil - near - Handle h1 should not be Handle::UNDEFINED.");
                        OC_ASSERT(smh != Handle::UNDEFINED, "WWUtil - near - A SpaceMap must exist");

                        stringstream ss;
                        ss << get_instance(pe);

                        const SpaceServer::SpaceMap& sm = spaceServer().getMap(smh);

                        bool result =
                            AtomSpaceUtil::getPredicateValueAtSpaceMap(atomSpace,
                                    ss.str(),
                                    sm,
                                    h1,
                                    h2);

                        // note that, although we test booth vo1 and vo2, currently only one wild card
                        // is accepted. Thus, a near (_*_, _*_) will always be true because the arguments
                        // represent the same object.
                        if (is_wild_card(vo1)) {
                            vu.setVariableState(vo1_def_obj, result);
                        }
                        if (is_wild_card(vo2)) {
                            vu.setVariableState(vo2_def_obj, result);
                        }

                        if (result) {
                            general_result = true;
                        }
                    }
                }
                if (is_wild_card(vo1)) {
                    vu.setUpdated(true);
                    vu.setOneVariableActive(general_result);
                }
                if (is_wild_card(vo2)) {
                    vu.setUpdated(true);
                    vu.setOneVariableActive(general_result);
                }
                return combo::bool_to_vertex(general_result);
            }
        }
    }
    break; //if not in the past then get off the case

    case id::has_said: {
        OC_ASSERT(it.number_of_children() == 2, "WWUtil - hasSaid - must have 2 children");

        sib_it sib_arg = it.begin();
        vertex va1 = *sib_arg;
        std::vector<definite_object> definite_objects =
            WorldWrapperUtil::getDefiniteObjects(smh, time, atomSpace,
                                                 self_id, owner_id, va1,
                                                 isInThePast, vu);

        //not necessary since we do not use the destination to reconstitute
        //the message but rather the message
        vertex va2 = *(++sib_arg);
        OC_ASSERT(is_message(va2), "WWUtil - hasSaid - 2nd argument must be of type message");
        const std::string& message = get_message(va2).getContent();

        // cache predicate data variables
        std::string name = p->get_name();
        std::vector<std::string> argument;

        bool general_result = false;
        foreach(definite_object def_obj, definite_objects) {

            argument.clear();
            argument.push_back(std::string(def_obj));
            argument.push_back(message);
            predicate pred(name, argument);
            float data = WorldWrapperUtil::cache.find(time, pred);

            bool result;
            if (data != CACHE_MISS) {
                result = (data > meanTruthThreshold);

            } else {
                Handle from_h = toHandle(atomSpace, def_obj, self_id, owner_id);
                result = AtomSpaceUtil::getHasSaidValueAtTime(atomSpace,
                         time,
                         getHasSaidDelay(),
                         from_h,
                         Handle::UNDEFINED,
                         message);

                // cache miss, compute value and cache it
                if (result) {
                    WorldWrapperUtil::cache.add(time, pred, 1.0f);
                } else {
                    WorldWrapperUtil::cache.add(time, pred, 0.0f);
                }
            }

            if (is_wild_card(va1)) {
                vu.setVariableState(def_obj, result);
            }
            if (result) {
                general_result = true;
            }
        }
        if (is_wild_card(va1)) {
            vu.setUpdated(true);
            vu.setOneVariableActive(general_result);
        }
        return combo::bool_to_vertex(general_result);
    }
    break;

    // -------------- pet feelings (signals) perceptions. these return the value of the signal
    // at the moment. is_X functions for pet feelings are implemented as combo scripts.
    case id::get_hunger: {
        OC_ASSERT(it.number_of_children() == 1,
                         "WWUtil - getHunger perception accept only one argument. Got '%d'.", it.number_of_children());
        vertex vo = *it.begin();
        if (is_indefinite_object(vo)) {
            vo = WorldWrapperUtil::evalIndefiniteObject(smh, time,
                    atomSpace,
                    self_id, owner_id,
                    get_indefinite_object(vo),
                    isInThePast);
        }

        std::string target =
            definite_object_to_atom_name(get_definite_object(vo),
                                         self_id, owner_id);
        return WorldWrapperUtil::getPhysiologicalFeeling
               (atomSpace, HUNGER_PREDICATE_NAME, target, time);

        /*
        // cache predicate data variables
        std::string name = p->get_name();
        std::vector<std::string> argument;
        argument.push_back(target);

        predicate pred(name, argument);
        float feeling = WorldWrapperUtil::cache.find(time, pred);

        if(feeling == CACHE_MISS){
        feeling = AtomSpaceUtil::getCurrentPetFeelingLevel(atomSpace, target,
        std::string(HUNGER_PREDICATE_NAME));
        WorldWrapperUtil::cache.add(time, pred, feeling);
        }

        logger().debug("WWUtil - '%d' hunger '%f'.", target.c_str(), feeling);
        return vertex(feeling);
        */
    }
    break;

    case id::get_thirst:
        OC_ASSERT(it.number_of_children() == 1,
                         "WWUtil - get_thirst perception accept only one argument. Got '%d'.", it.number_of_children());
        {
            vertex vo = *it.begin();
            if (is_indefinite_object(vo)) {
                vo = WorldWrapperUtil::evalIndefiniteObject(smh, time,
                        atomSpace,
                        self_id, owner_id,
                        get_indefinite_object(vo),
                        isInThePast);
            }

            std::string target =
                definite_object_to_atom_name(get_definite_object(vo),
                                             self_id, owner_id);
            return WorldWrapperUtil::getPhysiologicalFeeling
                   (atomSpace,  THIRST_PREDICATE_NAME, target, time);

            /*
                        // cache predicate data variables
                        std::string name = p->get_name();
                        std::vector<std::string> argument;
                        argument.push_back(target);

                        predicate pred(name, argument);
                        float feeling = WorldWrapperUtil::cache.find(time, pred);

                        if(feeling == CACHE_MISS){
                         feeling = AtomSpaceUtil::getCurrentPetFeelingLevel(atomSpace, target,
                                  std::string(THIRST_PREDICATE_NAME));
                            WorldWrapperUtil::cache.add(time, pred, feeling);
                        }

                  logger().debug("WWUtil - '%s' thirst '%f'.", target.c_str(), feeling);
                  return vertex(feeling);*/
        }
        break;

    case id::get_energy:
        OC_ASSERT(it.number_of_children() == 1,
                         "WWUtil - get_energy perception accept only one argument. Got '%d'.", it.number_of_children());
        {
            vertex vo = *it.begin();
            if (is_indefinite_object(vo)) {
                vo = WorldWrapperUtil::evalIndefiniteObject(smh, time,
                        atomSpace,
                        self_id, owner_id,
                        get_indefinite_object(vo),
                        isInThePast);
            }

            std::string target =
                definite_object_to_atom_name(get_definite_object(vo),
                                             self_id, owner_id);
            return WorldWrapperUtil::getPhysiologicalFeeling
                   (atomSpace,  ENERGY_PREDICATE_NAME, target, time);

            /*
                        // cache predicate data variables
                        std::string name = p->get_name();
                        std::vector<std::string> argument;
                        argument.push_back(target);

                        predicate pred(name, argument);
                        float feeling = WorldWrapperUtil::cache.find(time, pred);

                        if(feeling == CACHE_MISS){
                            feeling = AtomSpaceUtil::getCurrentPetFeelingLevel(atomSpace, target,
                                  std::string(ENERGY_PREDICATE_NAME));
                            WorldWrapperUtil::cache.add(time, pred, feeling);
                        }

                  logger().debug("WWUtil - '%s' energy '%f'.", target.c_str(), feeling);
                  return vertex(feeling);*/
        }
        break;

    case id::get_fitness:
        OC_ASSERT(it.number_of_children() == 1,
                         "WWUtil - get_fitness perception accept only one argument. Got '%d'.", it.number_of_children());
        {
            vertex vo = *it.begin();
            if (is_indefinite_object(vo)) {
                vo = WorldWrapperUtil::evalIndefiniteObject(smh, time,
                        atomSpace,
                        self_id, owner_id,
                        get_indefinite_object(vo),
                        isInThePast);
            }

            std::string target =
                definite_object_to_atom_name(get_definite_object(vo),
                                             self_id, owner_id);
            return WorldWrapperUtil::getPhysiologicalFeeling
                   (atomSpace, FITNESS_PREDICATE_NAME, target, time);

            /*
                        // cache predicate data variables
                        std::string name = p->get_name();
                        std::vector<std::string> argument;
                        argument.push_back(target);

                        predicate pred(name, argument);
                        float feeling = WorldWrapperUtil::cache.find(time, pred);

                        if(feeling == CACHE_MISS){
                         feeling = AtomSpaceUtil::getCurrentPetFeelingLevel(atomSpace, target,
                                std::string(FITNESS_PREDICATE_NAME));
                            WorldWrapperUtil::cache.add(time, pred, feeling);
                        }

                     logger().debug("WWUtil - '%s' fitness '%f'.", target.c_str(), feeling);
                     return vertex(feeling);*/
        }
        break;

    case id::get_current_action_repetition: {
        // cache predicate data variables
        std::string name = p->get_name();
        std::vector<std::string> argument;

        predicate pred(name, argument);
        float repetition = WorldWrapperUtil::cache.find(time, pred);

        if (repetition == CACHE_MISS) {
            Handle handle = atomSpace.getHandle(CONCEPT_NODE,
                                                "currentActionRepetition");
            OC_ASSERT(handle != Handle::UNDEFINED,
                             "WWUtil - There should be a ConceptNode for currentActionRepetition");

            HandleSeq result;
            atomSpace.getHandleSet(back_inserter(result),
                                   handle, FREQUENCY_LINK, false);

            if (result.size() != 1) {
                logger().warn("WWUtil - There should be only one FrequencyLink, got '%d'",
                             result.size());
                repetition = 0.0f;

            } else {

                handle = atomSpace.getOutgoing(result[0], 1);
                if (atomSpace.getType(handle) != NUMBER_NODE) { // error
                    logger().warn(
                                 "WWUtil - Outgoing atom[1] should be a NumberNode.");
                    repetition = 0.0f;

                } else {

                    repetition = atof(atomSpace.getName(handle).c_str());
                }
            }

            WorldWrapperUtil::cache.add(time, pred, repetition);
        }

        logger().debug(
                     "WWUtil - Current action repetition '%f'.",
                     repetition);
        return vertex(repetition);
    }
    break;

    case id::is_agent_state: {
        OC_ASSERT(
                         it.number_of_children() == 1 && is_contin(*it.begin()),
                         "WWUtil - is_agent_state perception accept one argument and it must be a contin. Got '%d' arguments.",
                         it.number_of_children());

        std::string stateNumber;
        {
            std::stringstream parser;
            parser << get_contin(*it.begin());
            stateNumber = parser.str( );
        }

        // cache predicate data variables
        std::string name = p->get_name();
        std::vector<std::string> argument;
        argument.push_back( stateNumber );

        predicate pred(name, argument);
        float state = WorldWrapperUtil::cache.find(time, pred);

        if (state == CACHE_MISS) {
            Handle agentHandle = AtomSpaceUtil::getAgentHandle( atomSpace, self_id );
            OC_ASSERT(
                             agentHandle != Handle::UNDEFINED,
                             "WWUtil - is_agent_state invalid agentHandle");
            Handle stateNodeHandle = atomSpace.getHandle( NUMBER_NODE, stateNumber );
            if (stateNodeHandle != Handle::UNDEFINED) {
                state =
                    AtomSpaceUtil::isPredicateTrue(atomSpace,
                                                   "agentModeState",
                                                   agentHandle,
                                                   stateNodeHandle ) ? 1 : 0;
            } else {
                state = 0;
            }
            WorldWrapperUtil::cache.add(time, pred, state);
        } // if

        logger().debug(
                     "WWUtil - scavenger hunt truth value: '%f' for state %s .",
                     state,
                     stateNumber.c_str( ) );
        return combo::bool_to_vertex( state == 1.0f );

    } // case
    break;

    case id::is_there_relation:
        OC_ASSERT(it.number_of_children() == 3,
                         "WWUtil - is_there_relation perception needs"
                         " three arguments. Got '%d'.",
                         it.number_of_children());
        {
            pre_it tree_it = tmp.begin();
            sib_it sib_relation = tree_it.begin();

            // relation parameter should be childless and a definite_object
            if (!sib_relation.is_childless() && !is_definite_object(*sib_relation)) {
                throw opencog::InvalidParamException(TRACE_INFO,
                                                     "WWUtil - Relation vertex should be childless and a definite_object. Got '%d' children an is_definite_object '%s'",
                                                     sib_relation.number_of_children(), is_definite_object(*sib_relation) ? "true" : "false");
            }

            *tree_it = *sib_relation;
            tmp.erase(sib_relation);
        }
        break;

    case id::is_proportional_next:
        OC_ASSERT(it.number_of_children() == 4,
                         "WWUtil - is_proportional_next perception needs four arguments. Got '%d'.", it.number_of_children());
        {
            WorldWrapperUtil::changePredicateName(tmp, "next");

            pre_it tree_it = tmp.begin();
            sib_it sibling_it = tree_it.begin();

            // skip the objects
            sibling_it++;
            sibling_it++;

            // get operator and value from the tree and remove them from tree. This is
            // necessary to avoid parsing the tree wrongly.
            vertex op = *sibling_it;
            sibling_it = tmp.erase(sibling_it);
            vertex val = *sibling_it;
            tmp.erase(sibling_it);

            if (!is_contin(op) && !is_contin(val)) {
                throw opencog::InvalidParamException(TRACE_INFO,
                                                     "WWUtil - is_proportional_next params error. Got operation is contin '%s', value is contin '%s'",
                                                     is_contin(op) ? "true" : "false", is_contin(val) ? "true" : "false");
            }

            double value = get_contin(val);
            int operation = (int)get_contin(op);

            std::vector<sib_it> wild_card_it;

            //eval indefinite arg
            for (sib_it arg = tree_it.begin(); arg != tree_it.end(); ++arg) {
                if (is_indefinite_object(*arg)) {
                    *arg = WorldWrapperUtil::evalIndefiniteObject(smh,
                            time,
                            atomSpace,
                            self_id,
                            owner_id,
                            get_indefinite_object(*arg),
                            isInThePast);

                } else if (is_wild_card(*arg)) {
                    wild_card_it.push_back(arg);
                }
            }

            // default eval - no wild card
            if (wild_card_it.empty()) {

                // next predicate name and the two object
                std::string name;// = p->name_get_name();
                if (is_perception(*tree_it)) {
                    perception per = get_perception(*tree_it);
                    name = per->get_name();

                } else if ( is_definite_object(*tree_it)) {
                    name = get_definite_object(*tree_it);

                } else {
                    OC_ASSERT(
                                     false,
                                     "WWUtil - is_proportional_next - argument should be perception or definite_object");
                }

                std::vector<std::string> arguments;
                for (sib_it arg = tree_it.begin(); arg != tree_it.end(); arg++) {
                    arguments.push_back(get_definite_object(*arg));
                }

                predicate pred(name, arguments);
                float data = WorldWrapperUtil::cache.find(time, pred);

                if (data != CACHE_MISS) {
                    return combo::bool_to_vertex
                           (WorldWrapperUtil::evaluateLogicalOperation(operation,
                                   data,
                                   value));

                } else {
                    // cache miss, compute the new value and cache it
                    Handle h = rec_lookup(atomSpace, tree_it, self_id, owner_id);

                    if (h == Handle::UNDEFINED) {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                        return id::logical_false;

                    } else {
                        float tv = atomSpace.getMean(h);
                        WorldWrapperUtil::cache.add(time, pred, tv);

                        return combo::bool_to_vertex
                               (WorldWrapperUtil::evaluateLogicalOperation(operation,
                                       tv,
                                       value));
                    }
                }

                // eval wild card
            } else {

                combo::UnifierIt it;
                bool general_result = false;

                // next predicate name and the two objects
                std::string name;// = p->get_name();
                if (is_perception(*tree_it)) {
                    perception per = get_perception(*tree_it);
                    name = per->get_name();

                } else if ( is_definite_object(*tree_it)) {
                    name = get_definite_object(*tree_it);

                } else {
                    OC_ASSERT(false, "WWUtil - is_proportional_next - argument should be perception or definite_object");
                }

                for (it = vu.begin(); it != vu.end(); it++) {
                    if (it->second) {
                        combo::definite_object def_obj = (*it).first;

                        foreach(sib_it arg, wild_card_it) {
                            *arg = def_obj;
                        }

                        // next predicate name and the two objects
                        std::vector<std::string> arguments;
                        for (sib_it arg = tree_it.begin(); arg != tree_it.end(); arg++) {
                            arguments.push_back(get_definite_object(*arg));
                        }
                        predicate pred(name, arguments);
                        float data = WorldWrapperUtil::cache.find(time, pred);

                        bool result = false;
                        if (data != CACHE_MISS) {
                            result = WorldWrapperUtil::evaluateLogicalOperation(operation, data, value);

                        } else {

                            // cache miss, compute the new value and cache it
                            Handle h = rec_lookup(atomSpace, tree_it,
                                                  self_id, owner_id);

                            if (h != Handle::UNDEFINED) {
                                float tv = atomSpace.getMean(h);
                                WorldWrapperUtil::cache.add(time, pred, tv);

                                result = WorldWrapperUtil::evaluateLogicalOperation(operation, tv, value);

                            } else {
                                WorldWrapperUtil::cache.add(time, pred, 0.0f);
                            }
                        }

                        vu.setVariableState(def_obj, result);
                        if (result) {
                            general_result = true;
                        }
                    }
                }
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
                return combo::bool_to_vertex(general_result);
            }
        }
        break;

    case id::is_last_group_command:
        OC_ASSERT(it.number_of_children() >= 2, "WWUtil - is_last_group_command perception needs at least two arguments. Got '%d'.", it.number_of_children());
        {
            sib_it sib_arg = it.begin();

            OC_ASSERT(is_definite_object(*sib_arg),
                      "WWUtil - is_last_group_command 1nd parameter should be a definite_object" );

            combo::vertex vo1 = *sib_arg;
            std::vector<combo::definite_object> agent_definite_objects =
                WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                     atomSpace,
                                                     self_id,
                                                     owner_id,
                                                     vo1, isInThePast, vu);

            // std::string targetName = get_definite_object(*sib_arg);
            ++sib_arg;

            OC_ASSERT(is_definite_object(*sib_arg), "WWUtil - is_last_group_command 2nd parameter should be a definite_object" );
            std::string command = get_definite_object(*sib_arg);



            std::stringstream stringParameters;
            std::vector<std::pair<int, boost::any> > parameters;
            int operation = 0;
            bool operationDefined = false;

            for ( ++sib_arg; sib_arg != it.end( ); ++sib_arg ) {
                combo::vertex argument = *sib_arg;
                if ( is_definite_object( argument ) ) {

                    std::string parameterValue = get_definite_object(*sib_arg);
                    if ( parameterValue == "any" ) {
                        if ( operationDefined ) {
                            throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - is_last_group_command the 'any' parameter shouldn't be preceded by an operation" );
                        } // if
                        stringParameters << " -1 NULL";
                        parameters.push_back(std::pair<int, boost::any>(-1, 0));
                        continue;
                    } // if
                    if ( !operationDefined ) {
                        throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - is_last_group_command an operation must be defined before the parameter value: %s", parameterValue.c_str( ) );
                    } // if
                    stringParameters << " " << operation << " " << parameterValue;
                    parameters.push_back( std::pair<int, boost::any>( operation, parameterValue ) );
                } else if ( is_contin( argument ) ) {
                    if ( !operationDefined ) {
                        operation = static_cast<int>( get_contin( argument ) );
                        operationDefined = true;
                    } else {
                        double value = get_contin( argument );
                        stringParameters << " " << operation << " " << value;
                        parameters.push_back( std::pair<int, boost::any>( operation, value ) );
                        operationDefined = false;
                    } // else
                } else {
                    throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - is_last_group_command a command parameter should be definite_object or contin" );
                } // else
            } // for

            // cache predicate data variables
            std::string name = p->get_name( );
            std::vector<std::string> argument;
            bool general_result = false;

            foreach(combo::definite_object def_obj, agent_definite_objects) {
                if ( AtomSpaceUtil::getAgentHandle( atomSpace, def_obj ) == Handle::UNDEFINED ) {
                    // ignore non-agent objects
                    continue;
                } // if

                argument.clear();
                argument.push_back( def_obj );
                argument.push_back( command );
                argument.push_back( stringParameters.str( ) );

                predicate pred(name, argument);
                float data = WorldWrapperUtil::cache.find(time, pred);

                if (data != CACHE_MISS) {
                    bool result = (data > meanTruthThreshold);

                    if (is_wild_card(vo1)) {
                        vu.setVariableState(def_obj, result);
                    }
                    if (result) {
                        general_result = true;
                    }

                } else {
                    // look for the latest executed action (30 secs frame)
                    // TODO is 30 secs enough?
                    //Note that by using getMostRecentAgentActionLinkWithinTime
                    //we handle in once the now and isInThePast cases
                    unsigned long delay_past = 30 * PAIUtils::getTimeFactor();
                    unsigned long t_past = (delay_past < time ? time - delay_past : 0);

                    Handle agentActionLink = AtomSpaceUtil::getMostRecentAgentActionLink( atomSpace, def_obj, "group_command", Temporal( t_past, time ) );

                    if (agentActionLink == Handle::UNDEFINED ) {
                        // found no previous action inside the timeframe
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                        if (is_wild_card(vo1)) {
                            vu.setVariableState(def_obj, false);
                        }
                        continue;
                    } // if

                    if ( atomSpace.getArity( agentActionLink ) <= 2 ) {
                        throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - group_command should have have more than one parameters" );
                    } // if

                    Handle actionParametersLink = atomSpace.getOutgoing(agentActionLink, 2);
                    if ( actionParametersLink == Handle::UNDEFINED ) {
                        throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - is_last_group_command an action EvalLink should has a listLink" );
                    } // if

                    Handle groupCommandParametersLink = atomSpace.getOutgoing( actionParametersLink, 2 );
                    if ( actionParametersLink == Handle::UNDEFINED ) {
                        throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - is_last_group_command an action Parameters ListLink should has three params" );
                    } // if

                    std::vector<std::string> actionParams;
                    std::string actionParamName = atomSpace.getName( groupCommandParametersLink );
                    boost::split( actionParams, actionParamName, boost::is_any_of( ";" ) );
                    unsigned int i;
                    foreach(std::string param, actionParams) {
                        boost::trim(param);
                        if ( param.length( ) == 0 ) {
                            throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - is_last_group_command invalid empty parameter" );
                        } // if
                    } // foreach

                    if ( parameters.size( ) != actionParams.size( ) ) {
                        throw opencog::InvalidParamException(TRACE_INFO, "WWUtil - is_last_group_command you must inform the same number of parameters used on group_command calling" );
                    } // if


                    for ( i = 0; i < parameters.size( ); ++i ) {
                        if ( parameters[i].first == -1 ) {
                            continue;
                        } // if
                        double value1 = 0;
                        double value2 = 0;

                        if ( std::isdigit( actionParams[i][0] ) ) {
                            std::istringstream parser( actionParams[i] );
                            parser >> value2;
                        } else {
                            value2 = static_cast<double>( boost::hash<std::string>()( actionParams[i] ) );
                        } // else

                        if ( typeid( double ) == parameters[i].second.type( ) ) {
                            value1 = boost::any_cast<double>( parameters[i] );
                        } else if ( typeid( std::string ) == parameters[i].second.type( ) ) {
                            value1 = static_cast<double>( boost::hash<std::string>()( boost::any_cast<std::string>( parameters[i] ) ) );
                        } // else

                        if ( !evaluateLogicalOperation( parameters[i].first, value1, value2 ) ) {
                            WorldWrapperUtil::cache.add(time, pred, 0.0f);
                            if (is_wild_card(vo1)) {
                                vu.setVariableState(def_obj, false);
                            }
                            continue;
                        } // if
                    } // for

                    if (is_wild_card(vo1)) {
                        vu.setVariableState(def_obj, true);
                    }
                    WorldWrapperUtil::cache.add(time, pred, 1.0f);
                    general_result = true;
                } // else
            } // foreach

            if (is_wild_card(vo1)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            } // if

            return combo::bool_to_vertex(general_result);

        }
        break;

    case id::is_last_agent_action:
        OC_ASSERT(it.number_of_children() >= 2,
                         "WWUtil - is_last_agent_action perception needs at least two arguments. Got '%d'.", it.number_of_children());
        {
            sib_it sib_arg = it.begin();

            vertex vo1 = *sib_arg;
            std::vector<combo::definite_object> agent_definite_objects =
                WorldWrapperUtil::getDefiniteObjects(smh, time, atomSpace,
                                                     self_id, owner_id, vo1,
                                                     isInThePast, vu);

            vertex vo2 = *(++sib_arg);
            OC_ASSERT(is_definite_object(vo2), "WWUtil - is_last_agent_action 2nd parameter should be a definite_object");

            // action name comes with an ACTION_NAME_POSTFIX so it won't be
            // confused with combo actions - so first we need to remove this
            // postfix
            std::string action_name = get_action_name(get_definite_object(vo2));

            std::vector<std::string> parameters;
            for (++sib_arg; sib_arg != it.end(); sib_arg++) {
                vertex v_temp = *sib_arg;

                if (is_indefinite_object(v_temp)) {
                    v_temp = WorldWrapperUtil::evalIndefiniteObject(smh, time, atomSpace, self_id,
                             owner_id, get_indefinite_object(v_temp),
                             isInThePast);
                }
                OC_ASSERT(is_definite_object(v_temp),
                                 "WWUtil - is_last_agent_action action parameters should already be a definite_objects.");
                parameters.push_back(std::string(get_definite_object(v_temp)));
            }


            // cache predicate data variables
            std::string name = p->get_name();
            std::vector<std::string> arguments;

            bool general_result = false;
            foreach(combo::definite_object def_obj, agent_definite_objects) {



                arguments.clear();
                arguments.push_back(def_obj);
                arguments.push_back(action_name);
                for (unsigned int i = 0; i < parameters.size(); i++) {
                    arguments.push_back(parameters[i]);
                }

                predicate pred(name, arguments);
                float data = WorldWrapperUtil::cache.find(time, pred);

                if (data != CACHE_MISS) {
                    bool result = (data > meanTruthThreshold);

                    if (is_wild_card(vo1)) {
                        vu.setVariableState(def_obj, result);
                    }
                    if (result) {
                        general_result = true;
                    }

                } else {

                    // look for the latest executed action (30 secs frame)
                    // TODO is 30 secs enough?
                    //Note that by using getMostRecentAgentActionLinkWithinTime
                    //we handle in once the now and isInThePast cases
                    unsigned long delay_past = 30 * PAIUtils::getTimeFactor();
                    unsigned long t_past = (delay_past < time ? time - delay_past : 0);
                    Handle agentActionLink = AtomSpaceUtil::getMostRecentAgentActionLinkWithinTime(atomSpace,
                                             std::string(def_obj),
                                             t_past, time);
                    // found no previous action inside the timeframe
                    if (agentActionLink == Handle::UNDEFINED) {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                        if (is_wild_card(vo1)) {
                            vu.setVariableState(def_obj, false);
                        }
                        continue;
                    }

                    //std::cout << "LAST AGENT ACTION : " << atomSpace.getName(atomSpace.getOutgoing(agentActionLink, 1))
                    //      << " IS_LAST_AGENT_ACTION TARGET : " << action_name << std::endl;

                    // action found isn't the one we are looking for
                    if (atomSpace.getName(atomSpace.getOutgoing(agentActionLink, 1)) != action_name) {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                        if (is_wild_card(vo2)) {
                            vu.setVariableState(def_obj, false);
                        }
                        continue;
                    }

                    // the action we want has parameters, check them with the ones stored in
                    // AtomSpace last agent action
                    if (!parameters.empty()) {
                        std::string actionParamsStr = AtomSpaceUtil::convertAgentActionParametersToString(atomSpace, agentActionLink);

                        std::vector<std::string> actionParams;
                        boost::split( actionParams, actionParamsStr, boost::is_any_of( ";" ) );
                        foreach(std::string param, actionParams) {
                            boost::trim(param);
                        }

                        bool result = true;
                        for (unsigned int i = 0; result && i < std::min(parameters.size(), actionParams.size()); ++i )
                            result = definite_object_equal(parameters[i], actionParams[i], self_id, owner_id);

                        if (is_wild_card(vo1)) {
                            vu.setVariableState(def_obj, result);
                        }

                        if (result) {
                            WorldWrapperUtil::cache.add(time, pred, 1.0f);
                            general_result = true;

                        } else {
                            WorldWrapperUtil::cache.add(time, pred, 0.0f);
                        }

                        // action we want has no parameters and match the lastest agent action
                        // from AtomSpace
                    } else {
                        if (is_wild_card(vo1)) {
                            vu.setVariableState(def_obj, true);
                        }
                        WorldWrapperUtil::cache.add(time, pred, 1.0f);
                        general_result = true;
                    }
                }
            }

            if (is_wild_card(vo1)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }
        break;

    case id::is_last_avatar_schema:
        OC_ASSERT(it.number_of_children() >= 1,
                         "WWUtil - is_last_avatar_schema perception needs at"
                         " least one argument. Got '%d'.",
                         it.number_of_children());
        {
            sib_it sib_arg = it.begin();

            vertex vo1 = *sib_arg;
            OC_ASSERT(is_definite_object(vo1),
                             "WWUtil - is_last_avatar_action 1st parameter"
                             " should be a definite_object");

            std::string action_name = get_action_name(get_definite_object(vo1));

            vo1 = *(++sib_arg);
            OC_ASSERT(is_action_result(vo1),
                             "WWUtil - is_last_avatar_schema 2nd parameter"
                             " should be an action_result");

            bool schemaSuccessful = false;
            if (get_action(vo1) == combo::id::action_success) {
                schemaSuccessful = true;
            }

            logger().debug(
                         "WWUtil - is_last_avatar_schema desired '%s'.",
                         action_name.c_str());

            bool has_wild_card = false;
            std::vector<std::vector<combo::definite_object> > parameters;

            for (++sib_arg; sib_arg != it.end(); sib_arg++) {
                vertex v_temp = *sib_arg;

                if (is_wild_card(v_temp) && !has_wild_card) {
                    has_wild_card = true;
                }

                std::vector<combo::definite_object> v_temp_defs =
                    WorldWrapperUtil::getDefiniteObjects(smh, time,
                                                         atomSpace, self_id,
                                                         owner_id, v_temp,
                                                         isInThePast, vu);
                parameters.push_back(v_temp_defs);
            }

            // look for the latest executed action (30 secs frame)
            // TODO is 30 secs enough?
            unsigned long  t = time - (30 * PAIUtils::getTimeFactor());
            Handle execLink = AtomSpaceUtil::getMostRecentPetSchemaExecLink(atomSpace, t, schemaSuccessful);

            // found no executed action in recent past,
            // mark wild card candidates false (if any) and return false
            if (execLink == Handle::UNDEFINED) {

                logger().debug(
                             "WWUtil - Found no action executed by pet"
                             " in last 30s.");
                if (has_wild_card) {
                    combo::UnifierIt u_it;
                    for (u_it = vu.begin(); u_it != vu.end(); u_it++) {
                        u_it->second = false;
                    }
                    vu.setUpdated(true);
                    vu.setOneVariableActive(false);
                }
                return combo::bool_to_vertex(false);
            }

            Handle groundedPred = atomSpace.getOutgoing(execLink, 0);
            if (groundedPred != Handle::UNDEFINED) {

                // last executed action not the one we are looking for
                if (atomSpace.getName(groundedPred) != action_name) {

                    logger().debug(
                                 "WWUtil - Last action found '%s',"
                                 " needed '%s'.",
                                 atomSpace.getName(groundedPred).c_str(),
                                 action_name.c_str());

                    if (has_wild_card) {
                        for (combo::UnifierIt uit = vu.begin();
                                uit != vu.end(); uit++) {
                            uit->second = false;
                        }
                        vu.setUpdated(true);
                        vu.setOneVariableActive(false);
                    }
                    return combo::bool_to_vertex(false);
                }
            }

            bool general_result = false;
            if (!parameters.empty()) {
                std::string actionParamsStr = AtomSpaceUtil::convertPetExecLinkParametersToString(atomSpace, execLink);

                std::vector<std::string> actionParams;
                boost::split( actionParams, actionParamsStr, boost::is_any_of( "," ) );
                foreach(std::string param, actionParams) {
                    boost::trim(param);
                }

                bool result = true;
                for (unsigned int i = 0;
                        i < std::min(parameters.size(), actionParams.size());
                        ++i ) {
                    std::vector<combo::definite_object> p = parameters[i];
                    // normal definite object
                    if (p.size() == 1) {
                        if (!definite_object_equal(p[0],
                                                   actionParams[i],
                                                   self_id, owner_id)) {
                            result = false;
                        }

                        // wild card
                    } else {
                        foreach(combo::definite_object def_obj, p) {
                            if (definite_object_equal(def_obj,
                                                      actionParams[i],
                                                      self_id, owner_id)) {
                                vu.setVariableState(def_obj, true);
                                vu.setOneVariableActive(true);
                            } else {
                                vu.setVariableState(def_obj, false);
                            }
                        }

                        // all candidates failed
                        if (!vu.isOneVariableActiveTMP()) {
                            result = false;
                        }
                    }
                } // for

                if (result) {
                    general_result = true;
                }

                // action we want has no parameters and match the lastest agent action
                // from AtomSpace
            } else {
                general_result = true;
            }

            if (has_wild_card) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }
        break;

    // Modulators        
    case id::get_activation_modulator:
        logger().debug("WorldWrapperUtil::%s - id::get_activation_modulator", __FUNCTION__);
        return WorldWrapperUtil::getModulator(atomSpace, ACTIVATION_MODULATOR_NAME, time);
        break;

    case id::get_resolution_modulator:
       logger().debug("WorldWrapperUtil::%s - id::get_resolution_modulator", __FUNCTION__);
       return WorldWrapperUtil::getModulator( atomSpace, 
                                              RESOLUTION_MODULATOR_NAME,
                                              time
                                             );
        break;

    case id::get_securing_threshold_modulator:
      logger().debug("WorldWrapperUtil::%s - id::get_securing_threshold_modulator", __FUNCTION__);
      return WorldWrapperUtil::getModulator( atomSpace, 
                                             SECURING_THRESHOLD_MODULATOR_NAME,
                                             time
                                           );
        break;

    case id::get_selection_threshold_modulator:
     logger().debug( "WorldWrapperUtil::%s - id::get_selection_threshold_modulator",
                     __FUNCTION__
                   );
     return WorldWrapperUtil::getModulator( atomSpace, 
                                            SELECTION_THRESHOLD_MODULATOR_NAME,
                                            time
                                          );

        break;

    // Demands    
    case id::get_energy_demand:
        logger().debug("WorldWrapperUtil::%s - id::get_energy_demand", __FUNCTION__);
        return WorldWrapperUtil::getDemand( atomSpace, 
                                            ENERGY_DEMAND_NAME,
                                            time
                                          );
        break;

    case id::get_water_demand:
        logger().debug("WorldWrapperUtil::%s - id::get_water_demand", __FUNCTION__);
        return WorldWrapperUtil::getDemand( atomSpace, 
                                            WATER_DEMAND_NAME,
                                            time
                                          );
        break;

    case id::get_integrity_demand:
        logger().debug("WorldWrapperUtil::%s - id::get_integrity_demand", __FUNCTION__);
        return WorldWrapperUtil::getDemand( atomSpace, 
                                            INTEGRITY_DEMAND_NAME,
                                            time
                                          );
        break;

    case id::get_affiliation_demand:
        logger().debug("WorldWrapperUtil::%s - id::get_affiliation_demand", __FUNCTION__);
        return WorldWrapperUtil::getDemand( atomSpace, 
                                            AFFILIATION_DEMAND_NAME,
                                            time
                                          );
        break;
        
    case id::get_certainty_demand:
        logger().debug("WorldWrapperUtil::%s - id::get_certainty_demand", __FUNCTION__);
        return WorldWrapperUtil::getDemand( atomSpace, 
                                            CERTAINTY_DEMAND_NAME,
                                            time
                                          );
        break;

     case id::get_competence_demand:
        logger().debug("WorldWrapperUtil::%s - id::get_competence_demand", __FUNCTION__);
        return WorldWrapperUtil::getDemand( atomSpace, 
                                            COMPETENCE_DEMAND_NAME,
                                            time
                                          );
        break;

    case id::get_current_demand_goal_truth_value:
        logger().debug("WorldWrapperUtil::%s - id::get_current_demand_goal_truth_value", __FUNCTION__);
        return WorldWrapperUtil::getDemandGoalTruthValue(atomSpace, 
                                                          "CurrentDemandGoal", 
                                                          self_id, 
                                                          time
                                                        );
        break;

    case id::get_previous_demand_goal_truth_value:
        logger().debug("WorldWrapperUtil::%s - id::get_previous_demand_goal_truth_value", __FUNCTION__);
        return WorldWrapperUtil::getDemandGoalTruthValue( atomSpace, 
                                                          "PreviousDemandGoal", 
                                                          self_id, 
                                                          time
                                                        );
        break;

    case id::get_integrity_demand_goal_truth_value:
        logger().debug("WorldWrapperUtil::%s - id::get_integrity_demand_goal_truth_value", __FUNCTION__); 
        return WorldWrapperUtil::getDemandGoalTruthValue( atomSpace, 
                                                          INTEGRITY_DEMAND_NAME,
                                                          self_id, 
                                                          time 
                                                        );
        break; 

    case id::get_affiliation_demand_goal_truth_value:
        logger().debug("WorldWrapperUtil::%s - id::get_affiliation_demand_goal_truth_value", __FUNCTION__); 
        return WorldWrapperUtil::getDemandGoalTruthValue( atomSpace, 
                                                          AFFILIATION_DEMAND_NAME, 
                                                          self_id, 
                                                          time
                                                        );
        break;

    case id::get_certainty_demand_goal_truth_value:
        logger().debug("WorldWrapperUtil::%s - id::get_certainty_demand_goal_truth_value", __FUNCTION__);
        return WorldWrapperUtil::getDemandGoalTruthValue( atomSpace, 
                                                          CERTAINTY_DEMAND_NAME,
                                                          self_id, 
                                                          time
                                                        );
        break;  

    case id::get_competence_demand_goal_truth_value:
        logger().debug("WorldWrapperUtil::%s - id::get_competence_demand_goal_truth_value", __FUNCTION__); 
        return WorldWrapperUtil::getDemandGoalTruthValue( atomSpace, 
                                                          COMPETENCE_DEMAND_NAME,
                                                          self_id, 
                                                          time
                                                        );
        break; 
   
        // -------------- pet traits perceptions
        //case id::is_aggressive:
    case id::get_aggressiveness:
        WorldWrapperUtil::changePredicateName(tmp, AGGRESSIVENESS_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_curious:
    case id::get_curiosity:
        WorldWrapperUtil::changePredicateName(tmp, CURIOSITY_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_playful:
    case id::get_playfulness:
        WorldWrapperUtil::changePredicateName(tmp, PLAYFULNESS_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_friendly:
    case id::get_friendliness:
        WorldWrapperUtil::changePredicateName(tmp, FRIENDLINESS_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_fearful:
    case id::get_fearfulness:
        WorldWrapperUtil::changePredicateName(tmp, FEARFULNESS_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_appreciative:
    case id::get_appreciativeness:
        WorldWrapperUtil::changePredicateName(tmp, APPRECIATIVENESS_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_excitable:
    case id::get_excitability:
        WorldWrapperUtil::changePredicateName(tmp, EXCITABILITY_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        // -------------- pet emotional feeling perceptions
        //case id::is_happy:
    case id::get_happiness:
        WorldWrapperUtil::changePredicateName(tmp, HAPPINESS_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_fear:
    case id::get_fear:
        WorldWrapperUtil::changePredicateName(tmp, FEAR_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_proud:
    case id::get_pride:
        WorldWrapperUtil::changePredicateName(tmp, PRIDE_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_lovely:
    case id::get_love:
        WorldWrapperUtil::changePredicateName(tmp, LOVE_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_hateful:
    case id::get_hate:
        WorldWrapperUtil::changePredicateName(tmp, HATE_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_angry:
    case id::get_anger:
        WorldWrapperUtil::changePredicateName(tmp, ANGER_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_grateful:
    case id::get_gratitude:
        WorldWrapperUtil::changePredicateName(tmp, GRATITUDE_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

        //case id::is_excited:
    case id::get_excitement:
        WorldWrapperUtil::changePredicateName(tmp, EXCITEMENT_PREDICATE_NAME);
        return WorldWrapperUtil::getEmotionalFeelingOrTrait(smh, time, atomSpace, self_id, owner_id,
                tmp.begin(), isInThePast, vu);
        break;

    case id::is_owner:
        WorldWrapperUtil::changePredicateName(tmp, OWNERSHIP_PREDICATE_NAME);
        break;

    default:
        break;

    }

    //copy the perception to not modify it
    //combo_tree tmp2(it);

    //get the string of tmp, just for the opencog::cassert
    stringstream ss;
    ss << tmp;
    OC_ASSERT(!isInThePast, "There is no methods yet to evaluate the perception %s in the past",
                     ss.str().c_str());


    logger().debug(
                 "WWUtil - default - %s", ss.str().c_str());

    pre_it tmp_it = tmp.begin();
    std::vector<sib_it> wild_card_it;

    //eval indefinite arg
    for (sib_it arg = tmp_it.begin(); arg != tmp_it.end(); ++arg) {
        if (is_indefinite_object(*arg)) {
            *arg = WorldWrapperUtil::evalIndefiniteObject(smh, time,
                    atomSpace,
                    self_id, owner_id,
                    get_indefinite_object(*arg),
                    isInThePast);

        } else if (is_wild_card(*arg)) {
            wild_card_it.push_back(arg);
        }
    }
    logger().debug(
                 "WWUtil - default - %s - eval all indefinite"
                 " objects or wild_card",
                 ss.str().c_str());


    // default eval - no wild card
    if (wild_card_it.empty()) {
        // next predicate name

        std::string name;
        if (is_perception(*tmp_it)) {
            perception per = get_perception(*tmp_it);
            name = per->get_name();

        } else if (is_definite_object(*tmp_it)) {
            name = get_definite_object(*tmp_it);
        } else {
            OC_ASSERT(false,
                             "WWUtil - default - argument tmp_it should"
                             " be a perception or definite_object.");
        }
        std::vector<std::string> arguments;

        for (sib_it arg = tmp_it.begin(); arg != tmp_it.end(); ++arg) {
            OC_ASSERT(is_definite_object(*arg),
                             "WWUtil - default eval - argument should"
                             " already be definite_object");
            arguments.push_back(get_definite_object(*arg));
        }

        logger().debug(
                     "WWUtil - default - %s - pred created",
                     ss.str().c_str());

        predicate pred(name, arguments);
        float data = WorldWrapperUtil::cache.find(time, pred);

        logger().debug(
                     "WWUtil - default - %s - cache consult done",
                     ss.str().c_str());

        if (data != CACHE_MISS) {

            logger().debug(
                         "WWUtil - default - %s - cache hit",
                         ss.str().c_str());

            return  combo::bool_to_vertex(data > meanTruthThreshold);

        } else {

            logger().debug(
                         "WWUtil - default - %s - cache miss",
                         ss.str().c_str());

            Handle h = rec_lookup(atomSpace, tmp_it, self_id, owner_id);

            if (h != Handle::UNDEFINED) {
                float tv = atomSpace.getMean(h);
                WorldWrapperUtil::cache.add(time, pred, tv);

            } else {
                WorldWrapperUtil::cache.add(time, pred, 0.0f);
            }

            //closed-world-assumption
            return combo::bool_to_vertex((h != Handle::UNDEFINED && atomSpace.getMean(h) > meanTruthThreshold));
        }

        // eval wild card
    } else {

        combo::UnifierIt it;
        bool general_result = false;

        // cache predicate name
        std::string name;
        if (is_perception(*tmp_it)) {
            perception per = get_perception(*tmp_it);
            name = per->get_name();
        } else if (is_definite_object(*tmp_it)) {
            name = get_definite_object(*tmp_it);
        } else {
            OC_ASSERT(false,
                             "WWUtil - default - argument tmp_it should"
                             " be a perception or definite_object.");
        }

        for (it = vu.begin(); it != vu.end(); it++) {
            if (it->second) {
                combo::definite_object def_obj = it->first;

                foreach(sib_it arg, wild_card_it) {
                    *arg = def_obj;
                }

                // cache predicate arguments
                std::vector<std::string> arguments;
                for (sib_it arg = tmp_it.begin(); arg != tmp_it.end(); ++arg) {
                    OC_ASSERT(is_definite_object(*arg),
                                     "WWUtil - default eval - argument should"
                                     " already be definite_object");
                    arguments.push_back(get_definite_object(*arg));
                }

                predicate pred(name, arguments);
                float data = WorldWrapperUtil::cache.find(time, pred);

                bool result = false;
                if (data != CACHE_MISS) {
                    result = (data >  meanTruthThreshold);
                } else {

                    Handle h = rec_lookup(atomSpace, tmp_it, self_id, owner_id);

                    if (h != Handle::UNDEFINED) {
                        float tv =  atomSpace.getMean(h);
                        WorldWrapperUtil::cache.add(time, pred, tv);
                    } else {
                        WorldWrapperUtil::cache.add(time, pred, 0.0f);
                    }

                    result = (h != Handle::UNDEFINED
                              && (atomSpace.getMean(h) > meanTruthThreshold));
                }

                vu.setVariableState(def_obj, result);
                if (result) {
                    general_result = true;
                }
                logger().debug(
                             "WWUtil - default - %s - with _*_ = %s"
                             " - result '%s'",
                             ss.str().c_str(), def_obj.c_str(),
                             result ? "true" : "false");
            }
        }
        vu.setUpdated(true);
        vu.setOneVariableActive(general_result);
        return combo::bool_to_vertex(general_result);
    }
}

/*
SpaceServer::SpaceMapPoint
WorldWrapperUtil::getLocation(const SpaceServer::SpaceMap& sm,
                              const AtomSpace& atomSpace,
                              const std::string& handleName)
throw (opencog::InvalidParamException, opencog::AssertionException, std::bad_exception)
{

    if (!sm.containsObject(handleName)) {
        static unsigned int mapCounter = 0;
        std::stringstream fileName;
        fileName << "ww_mapPersistence_";
        fileName << mapCounter;
        fileName << "_";
        // Add timestamp for preventing (or minimizing chance of) same file name by another oac.
        fileName << time(NULL);
        fileName << ".bin";
        SpaceServer::SpaceMap& map = (SpaceServer::SpaceMap&) sm;

        FILE* saveFile = fopen( fileName.str( ).c_str( ), "w+b" );
        map.save( saveFile );
        fclose( saveFile );
        ++mapCounter;

        throw opencog::InvalidParamException(TRACE_INFO,
                                             "WorldWrapperUtil - Space map does not contain '%s'."
                                             " The corresponding space map was saved in the %s file.",
                                             handleName.c_str(),
                                             fileName.str().c_str());
    }

    //const SpaceServer::ObjectMetadata& md = sm.getMetaData(handleName);
    const spatial::EntityPtr& entity = sm.getEntity( handleName );
    return SpaceServer::SpaceMapPoint(entity->getPosition( ).x, entity->getPosition( ).y );
}
*/

SpaceServer::SpaceMapPoint
WorldWrapperUtil::getLocation(const SpaceServer::SpaceMap& sm,
                              const AtomSpace& atomSpace,
                              Handle h)
{
    const spatial::Entity3D* entity = sm.getEntity( h );
    if (entity == 0)
        return SpaceServer::SpaceMapPoint::ZERO;

    return SpaceServer::SpaceMapPoint(entity->getPosition());
}
/*
double
WorldWrapperUtil::getAltitude(const SpaceServer::SpaceMap& sm,
                              const AtomSpace& atomSpace,
                              const std::string& handleName)
throw (opencog::InvalidParamException, opencog::AssertionException, std::bad_exception)
{
    if (!sm.containsObject(handleName)) {
        throw opencog::InvalidParamException(TRACE_INFO,
                                             "WorldWrapperUtil - Space map does not contain '%s'.",
                                             handleName.c_str());
    }

    //const SpaceServer::ObjectMetadata& md = sm.getMetaData(handleName);
    const spatial::EntityPtr& entity = sm.getEntity( handleName );
    return entity->getPosition().z;
}
*/
double
WorldWrapperUtil::getOrientation(const SpaceServer::SpaceMap& sm,
                                 const AtomSpace& atomSpace,
                                 Handle h) throw (opencog::InvalidParamException, opencog::AssertionException, std::bad_exception)
{
    std::string handleName = atomSpace.getName(h);

    if (!sm.containsObject(h)) {
        throw opencog::InvalidParamException(TRACE_INFO,
                                             "WorldWrapperUtil - Space map does not contain '%s'.", handleName.c_str());
    }

    //const SpaceServer::ObjectMetadata& md = sm.getMetaData(handleName);
    return sm.getEntity( h )->getYaw();
    //return md.yaw;
}

perception WorldWrapperUtil::nearest_random_X_to_is_X(indefinite_object io)
{
    return nearest_random_X_to_is_X(get_enum(io));
}

perception WorldWrapperUtil::nearest_random_X_to_is_X(avatar_indefinite_object_enum ioe)
{

    // print debug
    std::cout << "ioe = " << get_instance(ioe) << std::endl;

    switch (ioe) {
        //edible
    case id::random_edible:
    case id::nearest_edible:
        return get_instance(id::is_edible);
        //movable
    case id::random_movable:
    case id::nearest_movable:
        return get_instance(id::is_movable);
        //pickupable
    case id::random_pickupable:
    case id::nearest_pickupable:
        return get_instance(id::is_pickupable);
        //drinkable
    case id::random_drinkable:
    case id::nearest_drinkable:
        return get_instance(id::is_drinkable);
        //avatar
    case id::random_avatar:
    case id::nearest_avatar:
        return get_instance(id::is_avatar);
        //small
    case id::random_small:
    case id::nearest_small:
        return get_instance(id::is_small);
        //moving
    case id::random_moving:
    case id::nearest_moving:
        return get_instance(id::is_moving);
        //noisy
    case id::random_noisy:
    case id::nearest_noisy:
        return get_instance(id::is_noisy);
        //default
    default:
        OC_ASSERT(false, "nothing is associated with io");
        return get_instance(id::is_null);
    }
}

bool WorldWrapperUtil::is_builtin_compound_action(const vertex& v)
{
    if (const builtin_action* ba = boost::get<builtin_action>(&v)) {
        const avatar_builtin_action* pba =
            dynamic_cast<const avatar_builtin_action*>(*ba);
        return pba->is_compound();
    }
    return false;
}
bool WorldWrapperUtil::is_builtin_atomic_action(const vertex& v)
{
    if (const builtin_action* ba = boost::get<builtin_action>(&v)) {
        const avatar_builtin_action* pba =
            dynamic_cast<const avatar_builtin_action*>(*ba);
        return !pba->is_compound();
    }
    return false;
}

std::vector<combo::definite_object>
WorldWrapperUtil::getDefiniteObjects(Handle smh, unsigned long time,
                                     AtomSpace& atomSpace,
                                     const string& self_id,
                                     const string& owner_id, combo::vertex& v,
                                     bool isInThePast,
                                     combo::variable_unifier& vu)
{

    std::vector<definite_object> definite_objects;

    if (is_indefinite_object(v)) { //eval indefinite object of argument
        v = WorldWrapperUtil::evalIndefiniteObject(smh, time, atomSpace,
                self_id, owner_id,
                get_indefinite_object(v),
                isInThePast);
        if (is_definite_object(v)) {
            definite_objects.push_back(get_definite_object(v));
        }

    } else if (is_wild_card(v)) {
        OC_ASSERT(!vu.empty(),
                         "WWUtil - evalPerception - unifier should"
                         " not be empty.");
        definite_objects = vu.getActiveSet();
    } else if (is_definite_object(v)) {
        std::string target =
            definite_object_to_atom_name(get_definite_object(v),
                                         self_id, owner_id);
        definite_objects.push_back(definite_object(target));
    }

    return definite_objects;
}

void WorldWrapperUtil::changePredicateName(combo::combo_tree& tr,
        const std::string& name)
{
    pre_it t_it = tr.begin();
    *t_it = vertex(name);
}

bool WorldWrapperUtil::evaluateLogicalOperation(int operation,
        double valueA, double valueB)
{
    switch ( operation ) {
    case 0: // lesser than
        return ( valueA < valueB );
    case 1: // lesser equals
        return ( valueA <= valueB );
    case 2: // equals
        return isApproxEq(valueA, valueB, 0.001);
    case 3: // greater than
        return ( valueA > valueB );
    case 4: // greater equals then
        return ( valueA >= valueB );
    default:
        return false;
    } // switch
}

float WorldWrapperUtil::getPhysiologicalFeeling(AtomSpace& atomSpace,
        const std::string& feeling,
        const std::string& target,
        unsigned long time)
{

    // cache predicate data variables
    std::string name(feeling);
    std::vector<std::string> argument;
    argument.push_back(target);

    predicate pred(name, argument);
    float value = WorldWrapperUtil::cache.find(time, pred);

    if (value == CACHE_MISS) {
        value = AtomSpaceUtil::getCurrentPetFeelingLevel(atomSpace, target, feeling);
        WorldWrapperUtil::cache.add(time, pred, value);
    }

    logger().debug(
                 "WWUtil - '%s' with '%s' '%f'.",
                 target.c_str(), feeling.c_str(), value);
    return value;
}

float WorldWrapperUtil::getModulator(AtomSpace & atomSpace,
                                     const std::string & modulatorName,
                                     unsigned long time)
{
    // Cache predicate data variables
    //
    // Actually, Modulator is not a Predicate in AtomSpace 
    // (see "rules_core.scm" for the detail),
    // while we only want to use WorldWrapperUtil's cache mechanism.
    std::string name(modulatorName);
    std::vector<std::string> argument;

    predicate pred(name, argument);
    float value = WorldWrapperUtil::cache.find(time, pred);

    if (value == CACHE_MISS) {
        value = AtomSpaceUtil::getCurrentModulatorLevel(atomSpace, modulatorName);
        WorldWrapperUtil::cache.add(time, pred, value);
    }

    logger().debug( "WorldWrapperUtil::%s - '%s' for the avatar is '%f'", 
                    __FUNCTION__, 
                    modulatorName.c_str(), 
                    value
                  );

    return value;
}

float WorldWrapperUtil::getDemand(AtomSpace & atomSpace,
                                  const std::string & demandName,
                                  unsigned long time)
{
    // Cache predicate data variables
    //
    // Actually, Demand is not a Predicate in AtomSpace 
    // (see "rules_core.scm" for the detail),
    // while we only want to use WorldWrapperUtil's cache mechanism.
    std::string name(demandName);
    std::vector<std::string> argument;

    predicate pred(name, argument);
    float value = WorldWrapperUtil::cache.find(time, pred);

    if (value == CACHE_MISS) {
        value = AtomSpaceUtil::getCurrentDemandLevel(atomSpace, demandName);
        WorldWrapperUtil::cache.add(time, pred, value);
    }

    logger().debug( "WorldWrapperUtil::%s - current '%s' level is '%f'.", 
                    __FUNCTION__, 
                    demandName.c_str(), 
                    value
                  );

    return value;
}

float WorldWrapperUtil::getDemandGoalTruthValue(AtomSpace & atomSpace,
                                                const std::string & demand,
                                                const std::string & self_id,
                                                unsigned long time)
{
    // Cache predicate data variables
    //
    // Actually, Demand is not a Predicate in AtomSpace 
    // (see "rules_core.scm" for the detail),
    // while we only want to use WorldWrapperUtil's cache mechanism.
    std::string name(demand);
    std::vector<std::string> argument;

    predicate pred(name, argument);
    float value = WorldWrapperUtil::cache.find(time, pred);

    if (value == CACHE_MISS) {
        Handle hDemandGoal = AtomSpaceUtil::getDemandGoalEvaluationLink(atomSpace, demand);

        if ( hDemandGoal == opencog::Handle::UNDEFINED ) 
            value = randGen().randfloat();
        else
            value = atomSpace.getMean(hDemandGoal); 
    }

    logger().debug( "WorldWrapperUtil::%s - '%s' for the avatar with id '%s' is '%f'.", 
                    __FUNCTION__, 
                    demand.c_str(), 
                    self_id.c_str(),
                    value
                  );

    return value;
}

float WorldWrapperUtil::getEmotionalFeelingOrTrait(
        Handle smh,
        unsigned long time,
        AtomSpace& atomSpace,
        const string& self_id,
        const string& owner_id,
        const combo::combo_tree::iterator it,
        bool isInThePast,
        combo::variable_unifier& vu)
{


    //eval indefinite arg
    for (sib_it arg = it.begin(); arg != it.end(); ++arg) {
        if (is_indefinite_object(*arg)) {
            *arg = WorldWrapperUtil::evalIndefiniteObject(smh, time,
                    atomSpace,
                    self_id, owner_id,
                    get_indefinite_object(*arg),
                    isInThePast);
        }
        logger().debug(
                     "WWUtil - getEmotionalFeelingOrTrait - eval all"
                     " indefinite objects.");
    }

    combo_tree tmp(it);
    stringstream ss;
    ss << tmp;

    // constructing predicate struct to search cache
    std::string name;
    if (is_definite_object(*it)) {
        name = get_definite_object(*it);
    } else {
        OC_ASSERT(false,
                         "WWUtil - getEmotionalFeelingOrTrait - argument"
                         " it should be definite_object.");
    }
    std::vector<std::string> arguments;

    for (sib_it arg = it.begin(); arg != it.end(); ++arg) {
        OC_ASSERT(is_definite_object(*arg),
                         "WWUtil -  getEmotionalFeelingOrTrait - argument"
                         " should be definite_object");
        arguments.push_back(get_definite_object(*arg));
    }

    predicate pred(name, arguments);
    float data = WorldWrapperUtil::cache.find(time, pred);

    if (data != CACHE_MISS) {
        logger().debug(
                     "WWUtil -  getEmotionalFeelingOrTrait - %s - cache hit"
                     " - value '%.2f'.",
                     ss.str().c_str(), data);
        return data;

    } else {

        float data = -1.0f;
        Handle h = rec_lookup(atomSpace, it, self_id, owner_id);

        if (h != Handle::UNDEFINED) {
            data = atomSpace.getMean(h);
        }
        WorldWrapperUtil::cache.add(time, pred, data);

        logger().debug("WWUtil -  getEmotionalFeelingOrTrait - %s - cache miss - value '%.2f'.",
                     ss.str().c_str(), data);
        return data;
    }
}

} } // namespace opencog::world

