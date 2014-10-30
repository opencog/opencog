/*
 * opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
 *
 * Updated: by Jinhua Chua, on 2011-11-22
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

#ifndef ATOMSPACEUTIL_H_
#define ATOMSPACEUTIL_H_

/**
 * AtomSpaceUtil.h
 *
 * Class with util methods for related to AtomSpace manipulation
 */

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/spatial/math/Vector3.h>

#include <opencog/spacetime/HandleTemporalPair.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/Temporal.h>
#include <opencog/spacetime/TemporalTable.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>

using namespace opencog;

class AtomSpaceUtil
{


private:

    /**
     * Add a property predicate into the AtomSpace for the given arguments.
     * The predicate has the following basic template:
     *
     * <code>
     * EvaluationLink
     *    pred:PredicateNode
     *    ListLink
     *       arg1
     *       arg2
     *       ...
     *       argN
     * </code>
     *
     * @param atomSpace The AtomSpace where the predicate will be added.
     * @param predicateName The name of the predicate. This name should be exactly
     * the same names of combo predicates.
     * @param ll_out The list link's outgoing set with the arguments arg1,arg2,...,argN
     * @param tv Truth value for the evaluation link. If truth value is smaller
     *           than 0.5, the predicate do not apply to the arguments. Predicate
     *           is true otherwise.
     * @param permanent  An optional flag to indicate if this property must be kept forever
     *                   (or until it is explicitly removed). It's false by default.
     * @param t  An optional temporal (should not be used arbitrarily).
     * @return On success, the handle of the predicate's AtTimeLink (if timestamped);
     *                     or of the predicate's EvalLink (if not timestamped);
     *                     or Handle::UNDEFINED, if predicate is false but it
     *                     is not represented in AtomSpace.
     *         On failure, return Handle::UNDEFINED
     */
    static Handle addGenericPropertyPred(AtomSpace& atomSpace,
                                         std::string predicateName,
                                         const HandleSeq& ll_out,
                                         TruthValuePtr tv,
                                         bool permanent = false,
                                         const Temporal& t = UNDEFINED_TEMPORAL);

    /**
     * Static variables for storing the handles for the LatestLink(AtTimeLink(...))
     * that represents the latest data for each type of information
     */
    typedef std::map<Handle, Handle> HandleToHandleMap;
    static HandleToHandleMap latestAgentActionDone;
    static HandleToHandleMap latestPhysiologicalFeeling;
    static HandleToHandleMap latestAvatarSayActionDone;
    static HandleToHandleMap latestAvatarActionDone;
    static HandleToHandleMap latestAvatarActionPredicate;

    static HandleToHandleMap latestModulators; 
    static HandleToHandleMap latestDemands; 
    static HandleToHandleMap latestFeelings; 
    static HandleToHandleMap latestStimulus; 

    static std::map<Handle, HandleToHandleMap > latestSpatialPredicate;
    static HandleToHandleMap latestSchemaPredicate;
    static boost::unordered_map<std::string, HandleSeq> frameElementsCache;
    static Handle latestIsExemplarAvatar;

    /**
     * Create a LatestLink holding an AtTimeLink and update infoMap
     *
     * @param infoMap     pairs of (PredicateNode, LatestLink)
     * @param as
     * @param atTimeLink
     * @param key         key (a handle to PredicateNode) of infoMap
     *
     * @note
     *     LatestLink
     *         AtTimeLink
     *             TimeNode
     *             EvaluationLink
     */
    static void updateGenericLatestInfoMap(HandleToHandleMap & infoMap,
                                           AtomSpace & as,
                                           Handle atTimeLink,
                                           Handle key);

    static void updateGenericLatestSingleInfo(Handle & latestSingleInfoHandle,
                                              AtomSpace & as,
                                              Handle atTimeLink);

public:

    /**
     * Return the ReferenceLink given its first outgoing
     *
     * @note We assume there should be only one ReferenceLink
     *       containing the given atom
     *
     * ReferenceLink
     *     firstOutgoing
     *     secondOutgoing
     */
    static Handle getReferenceLink(AtomSpace & atomSpace, Handle hFirstOutgoing); 

    /**
     * Return the second outgoing of a ReferenceLink given the first outgoing
     */
    static Handle getReference(AtomSpace & atomSpace, Handle hFirstOutgoing); 

    /**
     * Return the InheritanceLink list given its first outgoing
     *
     * InheritanceLink
     *     firstOutgoing
     *     secondOutgoing
     */
    static std::vector<Handle> getInheritanceLinks(AtomSpace & atomSpace,
                                                   Handle hFirstOutgoing);

   /**
     * Returns the current level of the given feeling name.  If the
     * feeling does not exists on AtomSpace or some problem occurs
     * when invoked, this method will return -1.
     *
     * @param atomSpace The atom space
     * @param petId Pet's name(id)
     * @param feelingName name of the feeling wanted
     * @return The current level of a given feeling
     */
    static float getCurrentPetFeelingLevel( AtomSpace& atomSpace,
                                            const std::string& petId,
                                            const std::string& feelingName );

    /**
     * Returns the current level of the given Modulator.
     *
     * If the Modulator does not exist on AtomSpace or some problem
     * occurs when invoked, this method will return a random value in
     * [0, 1]
     *
     * @param atomSpace      The AtomSpace
     * @param modulatorName  Name of the Modulator wanted
     * @param rangGen        Random number generator
     *
     * @return float    The current level of a given Modulator
     *
     * @note The format of a Modulator stored in AtomSpace is:
     *
     * AtTimeLink
     *     TimeNode "timestamp"
     *     SimilarityLink
     *         NumberNode: "modulator_value"
     *         ExecutionOutputLink
     *             GroundSchemaNode: modulatorUpdater
     */ 
    static float getCurrentModulatorLevel(AtomSpace & atomSpace, 
                                          const std::string & modulatorName);
    /**
     * Returns the current level of the given Demand.
     *
     * If the Demand does not exist on AtomSpace or some problem
     * occurs when invoked, this method will return a random value in
     * [0, 1]
     *
     * @param atomSpace The AtomSpace
     * @param demand    Name of the Demand wanted
     *
     * @return float    The current level of a given Demand
     */ 
    static float getCurrentDemandLevel(AtomSpace & atomSpace, 
                                       const std::string & demandName);

    /**
     * Get the handle to Demand Goal (EvaluationLink)
     *
     * @param atomSpace The AtomSpace
     * @param demand    Name of the Demand 
     *
     * @return Handle   The Handle to Demand Goal (EvaluationLink), or
     *                  Handle::UNDEFINED if fails
     */
    static Handle getDemandGoalEvaluationLink(AtomSpace & atomSpace, 
                                              const std::string & demand 
                                             );

    /**
     * Returns a pointer to the most recent evaluation link. With the
     * given name of the predicateNode link, this method performs a
     * search on AtomSpace to find the link marked with the most
     * recent timestamp. If the evaluation link does not exists, an
     * Handle::UNDEFINED will be returned.
     *
     * @param atomSpace The atom space
     * @param predicateNodeName The name of the predicate node of the Evaluation link
     * @return Link evaluation link
     */
    static Handle getMostRecentEvaluationLink(const AtomSpace& atomSpace,
            const std::string& predicateNodeName );

    // get all the nodes from InheritanceLinks, given the SecondOutgoing
    static std::vector<Handle> getNodesByInheritanceLink(AtomSpace &atomSpace, Handle &hSecondOutgoing);

    // get all the nodes from EvaluationLinks, given the PredicateNode,and the second,third...outgoings of the listLink if any
    static std::vector<Handle> getNodesByEvaluationLink(AtomSpace &atomSpace, string predicate, HandleSeq& hNonFirstOutgoings);

    static std::vector<Handle> getEvaluationLinks(AtomSpace &atomSpace, string predicate, HandleSeq &hfirstOutgoings);

    /**
     * Returns witin timestamps vector all EvaluationLinks for a given predicate
     * node. Optionally it's possible to specify temporal and search criterion on
     * the time search.
     *
     * Default temporal and search criterion returns all EvaluationLinks
     *
     * @param atomSpace The atomspace
     * @param std::vector<HandleTemporalPair> all evaluation links found
     * @param predicateNodeName the name of the predicateNode used to find evaluation links
     * @param temporal the temporal used in time searchs
     * @param criterion the search criterion (see TemporalTable.h for possible
     *        values)
     * @param needSort Inform if the resulting vector need to be sorted since if
     * there are more than one handle for the predicate node, there results will
     * be ordered by ther handles not the temporal.
     */
    static void getAllEvaluationLinks(const AtomSpace& atomSpace,
                                      std::vector<HandleTemporalPair>& timestamps,
                                      const std::string& predicateNodeName,
                                      const Temporal& temporal = UNDEFINED_TEMPORAL,
                                      TemporalTable::TemporalRelationship criterion = TemporalTable::EXACT,
                                      bool needSort = false);

    /**
     * Add a generic predicate into the AtomSpace or update it's value if it was already defined
     * The predicate has the following basic template:
     *
     * <code>
     * EvaluationLink
     *    pred:PredicateNode
     *    ListLink
     *       object1 (optional)
     *       object2 (optional)
     *       object3 (optional)
     * </code>
     *
     * @param atomSpace The AtomSpace.
     * @param predicateName The name of the predicate. This name should be exactly
     * the same names of combo predicates.
     * @param tv Truth value for the evaluation link. if the mean of TV is less
     * than 0.5 it will be added anyway
     * @param object1 The handle of the object1
     * @param object1 The handle of the object2
     * @param object1 The handle of the object3
     * @return On success, the handle of the predicate's EvalLink (if not
     * timestamped) or Handle::UNDEFINED, if predicate is false but it is not
     * represented in AtomSpace. On failure, return Handle::UNDEFINED.
     */
    static Handle setPredicateValue( AtomSpace& atomSpace,
                                     std::string predicateName,
                                     TruthValuePtr tv,
                                     Handle object1 = Handle::UNDEFINED,
                                     Handle object2 = Handle::UNDEFINED,
                                     Handle object3 = Handle::UNDEFINED );


    const static double highLongTermImportance;

    /**
    * Adds a node into the AtomSpace.
    * @param the AtomSpace which the node will be added to
    * @param the type of the node
    * @param the name of the node
    * @param flag to indicate if the atom should be permanent (not removed by decay importance task) or not.
    * @param flag to indicate if, in case of the permanent flag is false and the atom already exists, the atom should be re-added, so that a merge is done and, consequently it renews its sti value.
    * @return the Handle of the added node.
    */
    static Handle addNode(AtomSpace& atomSpace,
                          Type nodeType,
                          const std::string& nodeName,
                          bool permanent = false,
                          bool renew_sti = true);

    /**
     * Adds a link into the AtomSpace, if it does not exist yet.
     * @param the AtomSpace which the link will be added to
     * @param the type of the link
     * @param the outgoing set of the link
     * @param flag to indicate if the atom should be permanent (not removed by decay importance task) or not.
     * @param flag to indicate if, in case of the permanent flag is false and the atom already exists, the atom should be re-added, so that a merge is done and, consequently it renews its sti value.
     * @return the Handle of the added link.
     */
    static Handle addLink(AtomSpace& atomSpace,
                          Type linkType,
                          const HandleSeq& outgoing,
                          bool permanent = false,
                          bool renew_sti = true,
                          TruthValuePtr tv = TruthValue::TRUE_TV());

    /**
    * Adds a reward predicate into the AtomSpace
    * @param   The AtomSpace which the reward predicate will be added to
    * @param   The id of the pet which the reward is for
    * @param   The timestamp when this reward happened
    * @return  The Handle of the EvaluationLink for the reward predicate
    */
    static Handle addRewardPredicate(AtomSpace& atomSpace,
                                     const char* petID,
                                     unsigned long timestamp);

    /**
     * Adds a punishment predicate into the AtomSpace
     * @param   The AtomSpace which the punishment predicate will be added to
     * @param   The id of the pet which the punishment is for
     * @param   The timestamp when this punishment happened
     * @return  The Handle of the EvaluationLink for the punishment predicate
     */
    static Handle addPunishmentPredicate(AtomSpace& atomSpace,
                                         const char* petID,
                                         unsigned long timestamp);

    /**
     * Check if the action with the given id was done since the given timestamp.
     * @param atomSpace the AtomSpace where the condition will be checked
     * @param actionpredicateName the predicate name which the existence for
     * the given action will be checked for
     * @param actionExecLink the Handle of the ExecLink that represents the action
     * @param sinceTimestamp the timestamp since which the ActionDone mark of
     * the action will be checked for
     * @return true if the action was marked as done after the given timestamp,
     * false otherwise
     *
     * @note You can use PAI::getCurrentTimestamp() just before calling addAction() to get
     * the timestamp argument for this method.
     */
    static bool isActionPredicatePresent(AtomSpace& atomSpace,
                                         const char* actionPredicateName,
                                         Handle actionExecLink,
                                         unsigned long sinceTimestamp);


    /**
     * Gets the position coordinates and object handle from an EvalLink for
     * a "position" predicate
     * @param the AtomSpace where the data will be extracted from
     * @param The handle of the EvaluationLink
     * @param x coordinate to be get
     * @param y coordinate to be get
     * @param z coordinate to be get
     * @param handle of the node that represents the object the predicate refers to
     * @return if got the coordinates and object's node handle
     * successfully. False otherwise
     */
    static bool getXYZOFromPositionEvalLink(const AtomSpace& atomSpace,
                                            Handle evalLink,
                                            double &x,
                                            double &y,
                                            double &z,
                                            Handle &o);

//    /**
//     * Gets the spaceMapHandle which is either stamped at time t or
//     * just preceeding t.
//     * If there no such spaceMapHandle then return Handle::UNDEFINED
//     *
//     * @param atomSpace The AtomSpace with the spaceServer where the spaceMap is
//     * @param t  The timestamp to look at
//     * @return The spaceMapHandle at time t or if none the first one before t
//     */
//    static Handle getSpaceMapHandleAtTimestamp(const AtomSpace &atomSpace,
//            unsigned long t);

    static Handle getCurrentSpaceMapHandle(const AtomSpace &atomSpace);

    /**
     * Gets the value of the predicate at a spaceMap in the past. Based
     * on SpaceMap.
     *
     * @param predicate The predicate to be checked: near, above, inside, below.
     * @param timestamp The timestamp used to search for the SpaceMap.
     * @param objectA   The handle of the first object to be used in the
     *                  predicate.
     * @param objectB   The handle of the second object to be used in the
     *                  predicate.
     */
    static bool getPredicateValueAtSpaceMap(AtomSpace& atomSpace,
                                            const std::string predicate,
                                            const SpaceServer::SpaceMap& sm,
                                            Handle obj1, Handle obj2);

//    /**
//     * Gets the value of the predicate at a given timestamp in the past. Based
//     * on SpaceMap.
//     *
//     * @param predicate The predicate to be checked: near, above, inside, below.
//     * @param timestamp The timestamp used to search for the SpaceMap.
//     * @param objectA   The handle of the first object to be used in the predicate.
//     * @param objectB   The handle of the second object to be used in the
//     * predicate.
//     */
//    static bool getPredicateValueAtTimestamp(const AtomSpace &atomSpace,
//            const std::string& predicate,
//            unsigned long timestamp,
//            Handle objectA, Handle objectB);

    /**
     * Check whether an avatar has said a given message between
     * timestamp and timestamp-delay
     *
     * @param atomSpace    atomSpace to search
     * @param timestamp    world time at which the predicate is evaluated
     * @param delay        time to look behind
     * @param from_h       Handle of the source of the message
     * @param to_h         Handle of the destination of the message
     * @param message      message
     * @param include_to   if false then the message must fit the name of
     *                     the sentence atom, the Handle to_h is then unused
     *                     if true then the name of the sentence atom is
     *                     reconstituted according to the following format
     *                     "to:destinationID: message"
     */
    static bool getHasSaidValueAtTime(AtomSpace &atomSpace,
                                      unsigned long timestamp,
                                      unsigned long delay,
                                      Handle from_h,
                                      Handle to_h,
                                      const std::string& message,
                                      bool include_to = false);

    /**
     * Return the lastest handle ( with the latest timestamp ) among a handle set
     */
    static Handle getLatestHandle(const AtomSpace &atomSpace,HandleSeq& handles);

    /**
     * Return the handle of the latest evaluationlink of the given predicate, with its objects
     *
     * @param predicateName The name of the predicate whose EvalLink is going to
     *        be checked
     * @param a The handle of the first object
     * @param b The handle of the second object (optional). If this handle is
     *        not informed then a unary predicate is assumed
     * @param getPositiveResult: try to get the EvaluationLink with truth value >= 0.5 if any, if not, return the one < 0.5. vice versa
     */
    static Handle getLatestEvaluationLink(AtomSpace &atomSpace,
                                 std::string predicateName,
                                 Handle a,
                                 Handle b = Handle::UNDEFINED,
                                 Handle c = Handle::UNDEFINED, bool getPositiveResult = true) throw(opencog::NotFoundException);

    /**
     * Return the handle of the value node the given predicate, with its objects
     *
     * @param predicateName The name of the predicate whose EvalLink is going to
     *        be checked
     * @param a The handle of the first object
     * @param b The handle of the second object (optional). If this handle is
     *        not informed then a unary predicate is assumed
     */
    static Handle getPredicateValueNode(const AtomSpace &atomSpace,
                                 std::string predicateName,
                                 Handle a,
                                 Handle b = Handle::UNDEFINED,
                                 Handle c = Handle::UNDEFINED);

    /**
     * Return the mean value of the given predicate, with its objects
     *
     * @param predicateName The name of the predicate whose EvalLink is going to
     *        be checked
     * @param a The handle of the first object
     * @param b The handle of the second object (optional). If this handle is
     *        not informed then a unary predicate is assumed
     */
    static float getPredicateValue(AtomSpace& atomSpace,
                                   std::string predicateName,
                                   Handle a,
                                   Handle b = Handle::UNDEFINED)
        throw(opencog::NotFoundException);

    /**
     * Return true if the given predicate, with its objects, has the truth value
     * of its EvaluationLink greater than 0.5
     *
     * @param predicateName The name of the predicate whose EvalLink is going to
     *        be checked
     * @param a The handle of the first object
     * @param b The handle of the second object (optional). If this handle is
     *        not informed then a unary predicate is assumed
     */
    static bool isPredicateTrue(AtomSpace& atomSpace,
                                std::string predicateName,
                                Handle a,
                                Handle b = Handle::UNDEFINED);

    /**
     * Return true if the given avatar is the owner of the given pet
     * @param avatar The handle of the avatar
     * @param avatar The handle of the pet
     */
    static bool isPetOwner( AtomSpace& atomSpace,
                            Handle avatar, Handle pet );


    /**
     * Return true iff obj has changed its location between sm1 and sm2
     *
     * @param atomSpace The atomSpace to look at
     * @param sm1 the spaceMap before
     * @param sm2 the spaceMap after
     * @param the object to look at
     */
    static bool isMovingBtwSpaceMap(const AtomSpace& atomSpace,
                                    const SpaceServer::SpaceMap& sm1,
                                    const SpaceServer::SpaceMap& sm2,
                                    Handle obj);

    /**
     * Return true iff obj has changed its location between sm 
     * and the previous spaceMap
     *
     * @param atomSpace The atomSpace to look at
     * @param sm the spaceMap after
     * @param the object to look at
     */
    static bool isMovingBtwSpaceMap(const AtomSpace& atomSpace,
                                    const SpaceServer::SpaceMap& sm,
                                    Handle obj);

    /**
     * Get the length, width and height information for a given object, if any.
     * No need to inform the predicate name since this is static for this case.
     *
     * @param the AtomSpace where the data will be extracted from.
     * @param The handle of the object.
     * @param length The length to be get
     * @param width  The width to be get
     * @param height The heigth to be get
     *
     * @return True if a size predicate was found for the object and false
     * otherwise. If false is returned then the results of the passed by
     * reference objects are not valid. (May be corrupted)
     */
    static bool getSizeInfo(AtomSpace& atomSpace,
                            Handle object, double& length,
                            double& width, double &height);

    /**
     * Add a property predicate into the AtomSpace for the given node.
     * The predicate has the following basic template:
     *
     * <code>
     * EvaluationLink
     *    pred:PredicateNode
     *    ListLink
     *       obj:ObjectNode (or its descendents)
     * </code>
     *
     * @param atomSpace The AtomSpace.
     * @param predicateName The name of the predicate. This name should be exactly
     * the same names of combo predicates.
     * @param object The handle of the object
     * @param tv Truth value for the evaluation link. If truth value is smaller
     *           than 0.5, the predicate do not apply to the object. Predicate
     *           is true otherwise.
     * @param permanent  An optional flag to indicate if this property must be kept forever
     *                   (or until it is explicitly removed). It's false by default.
     * @param t  An optional temporal (should not be used arbitrarily).
     * @return On success, the handle of the predicate's AtTimeLink (if timestamped);
     *                     or of the predicate's EvalLink (if not timestamped);
     *                     or Handle::UNDEFINED, if predicate is false but it
     *                     is not represented in AtomSpace.
     *         On failure, return Handle::UNDEFINED
     */
    static Handle addPropertyPredicate(AtomSpace& atomSpace,
                                       std::string predicateName,
                                       Handle object,
                                       TruthValuePtr tv,
                                       bool permanent = false,
                                       const Temporal &t = UNDEFINED_TEMPORAL);

    /**
     * Add a property predicate (basically position predicates such as near,
     * inside, below, above) into the AtomSpace for the given nodes.
     * The predicate has the following basic template:
     *
     * <code>
     * EvaluationLink
     *     pred:PredicateNode
     *     ListLink
     *        A:ObjectNode (or its descendents)
     *        B:ObjectNode (or its descendents)
     * </code>
     *
     * The first object is the reference one, i.e. A is near B (in this case
     * the relation is commutative), A is inside B, A is above B, and A is
     * below B. The last two predicates are the oposite of each other, that
     * is, if A is above B, then B is below A. And similar, if A is below B
     * then, B is above A.
     *
     * @param atomSpace The AtomSpace.
     * @param predicateName The name of the predicate. This name should be exactly
     * the same names of combo predicates
     * @param objectA The handle of the first object
     * @param objectB The handle of the second object
     * @param tv The truth value of the evaluation link for the predicate
     * @param t  An optional temporal (should not be used arbitrarily).
     * @return On success, the handle of the predicate's AtTimeLink (if timestamped);
     *                     or of the predicate's EvalLink (if not timestamped);
     *                     or Handle::UNDEFINED, if predicate is false but it
     *                     is not represented in AtomSpace.
     *         On failure, return Handle::UNDEFINED
     */
    static Handle addPropertyPredicate(AtomSpace& atomSpace,
                                       std::string predicateName,
                                       Handle a,
                                       Handle b,
                                       TruthValuePtr tv,
                                       const Temporal& t = UNDEFINED_TEMPORAL);


    /**
     * Do the maintenance of the held object by an holder
     *
     * Add a isHolding predicate (basically position predicates such as near,
     * inside, below, above) into the AtomSpace for the given nodes.
     * The predicate has the following basic template:
     *
     * <code>
     * AtTimeLink
     *   TimeNode "$timestamp"
     *   EvaluationLink
     *     PredicateNode:"isHolding"
     *     ListLink
     *       AvatarNode|PetNode:"$avatar-id" or "$pet-id"
     *       ObjectNode|AccessoryNode:"$object-id"
     * </code>
     *
     * Besides, add/update the isHoldingSomething predicate (which is not
     * timestamped), as follows:
     *
     * <code>
     * EvaluationLink
     *   PredicateNode:"isHoldingSomething"
     *   ListLink
     *     AvatarNode|PetNode:"$avatar-id" or "$pet-id"
     * </code>
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @param objectId The id of the held object. If empty "", it will be a drop action
     * @param currentTimestamp Timestamp corresponding to 'now'
     */
    static void setupHoldingObject( AtomSpace& atomSpace,
                                    const std::string& holderId,
                                    const std::string& objectId,
                                    unsigned long currentTimestamp );

    /**
     * Retrieve the latest held, by an holderId, object's handle
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @return The Handle of the latest grabbed object. It can be Handle::UNDEFINED
     *         if there is no grabbed object
     */
    static Handle getLatestHoldingObjectHandle(AtomSpace& atomSpace,
            const std::string& holderId );

    static bool isObjectBeingHolded( AtomSpace& atomSpace,
                                     const std::string& objectId );
    static Handle getObjectHolderHandle( AtomSpace& atomSpace,
                                         const std::string& objectId );
    static std::string getObjectHolderId( AtomSpace& atomSpace,
                                          const std::string& objectId );

    /**
     * Retrieve the most recent atTimeLink that points to a 'isHolding'
     * EvaluationLink, used by a given holder
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @return The Handle of the latest atTimeLink that points to isHolding
     * Link. It can be Handle::UNDEFINED if there is no isHolding EvaluationLink
     */
    static Handle getMostRecentIsHoldingAtTimeLink(AtomSpace& atomSpace,
            const std::string& holderId);

    /**
     * Retrieve the most recent 'isHolding' Eval Link, used by a given holder
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @return The Handle of the latest isHolding Link. It can be Handle::UNDEFINED
     *         if there is no isHolding eval link
     */
    static Handle getMostRecentIsHoldingLink(AtomSpace& atomSpace,
            const std::string& holderId );


    /**
     * Retrieve the held, by an holderId, object's handle at time 'time'
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @param time time at which we consider the predicate is holding
     *
     * @return The Handle of the latest grabbed object. It can be Handle::UNDEFINED
     *         if there is no grabbed object
     */
    static Handle getHoldingObjectHandleAtTime(AtomSpace& atomSpace,
            const std::string& holderId,
            unsigned long time);


    /**
     * Retrieve the 'isHolding' Eval Link, used by a given holder at time 'time'
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @param time time at which isHolding predicate must be retrieved
     *
     * @return The Handle of the latest isHolding Link. It can be Handle::UNDEFINED
     *         if there is no isHolding eval link
     */
    static Handle getIsHoldingLinkAtTime(AtomSpace& atomSpace,
                                         const std::string& holderId,
                                         unsigned long time);


    /**
     * Helper function that retrieves the id of the object held by the pet or
     * avatar at time 'time'
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @param time at time at which the isHolding predicate is
     *
     * @return The id of the held object. It can be empty "" if there is no held object
     */
    static std::string getHoldingObjectIdAtTime(AtomSpace& atomSpace,
            const std::string& holderId,
            unsigned long time);

    /**
     * Helper function that retrieves the id of the object held by the pet or avatar
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding or dropping something
     * @return The id of the held object. It can be empty "" if there is no held object
     */
    static std::string getHoldingObjectId(AtomSpace& atomSpace,
                                          const std::string& holderId );


    /**
     * Helper function that checks whether an object is held by a pet or avatar
     *
     * @param atomSpace AtomSpace pointer
     * @param holderId Id of an avatar or a pet that is holding something
     * @return true if and only if holderId is holding something
     */
    static bool isHoldingSomething(AtomSpace& atomSpace,
                                   const std::string& holderId );



    /**
     * Given the handle of an object, it will return the real name of the
     * object, stored in a WORD_NODE.
     * Whether there is no defined name for the handle, an empty string will
     * be returned.
     *
     * @param atomSpace The agent AtomSpace
     * @param object The object handle
     * @return the name of the object if it exists, or an empty string, otherwise.
     */
    static std::string getObjectName( const AtomSpace& atomSpace, 
                                      Handle object );
    
    /**
     * Given the name of an object, whose letters  may be all in upercase, gets its id.
     * @return the id of the object, if it exists, or an empty string, otherwise.
     */
    static std::string getObjIdFromName( AtomSpace& atomSpace,
                                         const std::string& objectName );

    /**
     * Get the most recent agent-link, possibly according to temporal relationship
     *
     * @param atomSpace AtomSpace pointer
     * @param agentID The agent's id
     * @param temporal the temporal used in time searchs
     * @param criterion the search criterion (see TemporalTable.h for possible
     *        values)
     *
     * @return a handle that points to the latest agent action link or
     * Handle::UNDEFINED if there is no action link for the given agentid
     */
    static Handle getMostRecentAgentActionLink( const AtomSpace& atomSpace,
            const std::string& agentId,
            const Temporal& temporal = UNDEFINED_TEMPORAL,
            TemporalTable::TemporalRelationship criterion = TemporalTable::EXACT);

    /**
     * Returns a pointer to the most recent evaluation link. Using an agent id
     * this method performs a search on AtomSpace to find the evaluation link
     * marked with the most recent timestamp. If the evaluation link does not exists,
     * a Handle::UNDEFINED will be returned
     *
     * @param atomSpace The atom space
     * @param agentId The agent's id
     * @param actionName Action name
     * @param temporal Temporal range used on TimeServer search
     * @param criterion Time comparation criterion used on TimeServer search
     * @return Link evaluation link
     */
    static Handle getMostRecentAgentActionLink( AtomSpace& atomSpace,
            const std::string& agentId,
            const std::string& actionName,
            const Temporal& temporal = UNDEFINED_TEMPORAL,
            TemporalTable::TemporalRelationship criterion = TemporalTable::EXACT
                                              );

    /**
     * Get the most recent agent-link, executed in the interval [t1, t2]
     *
     * @param atomSpace AtomSpace pointer
     * @param agentID The agent's id
     * @param t1 Only agent-links executed after or during t1 will be inspected
     * @param t2 Only agent-links executed after or during t1 will be inspected
     *
     * @return a handle that points to the latest agent action link or Handle::UNDEFINED if
     *                            there is no action link for the given agentid
     */
    static Handle getMostRecentAgentActionLinkWithinTime( const AtomSpace& atomSpace,
            const std::string& agentId,
            unsigned long t1,
            unsigned long t2 );

    /**
     * Get the most recent agent-link, executed after a given timestamp
     *
     * @param atomSpace AtomSpace pointer
     * @param agentID The agent's id
     * @param timestamp Timestamp used as a floor time. Only agent-links executed after
     *                            this timestamp will be inspected
     * @return a handle that points to the latest agent action link or Handle::UNDEFINED if
     *                            there is no action link for the given agentid
     */
    static Handle getMostRecentAgentActionLinkAfterTime( const AtomSpace& atomSpace,
            const std::string& agentId,
            unsigned long timestamp );

    /**
     * Given an agentActionLink, this method will return a formated string
     * containing all the action parameters
     *
     * @param atomSpace AtomSpace pointer
     * @param agentActionLink An handle that points to and agent-action link
     * @return a formated string with the action parameters
     */
    static std::string convertAgentActionParametersToString( const AtomSpace& atomSpace,
            Handle agentActionLink );

    /**
     * Return the ImplicationLink handle referenced by a given rule.
     *
     * @param atomSpace The AtomSpace
     * @param rule The rule name
     * @return The ImplicationLink handle or Handle::UNDEFINED if an error occur.
     */
    static Handle getRuleImplicationLink(AtomSpace& atomSpace,
                                         const std::string& rule);
    
    /**
     * Return the Handle of SimilarityLink that holds ModulatorUpdater
     *
     * The format of a Modulator stored in AtomSpace is:
     *
     * SimilarityLink
     *     NumberNode: "modulator_value"
     *     ExecutionOutputLink
     *         GroundSchemaNode: modulatorUpdater
     *         ListLink 
     *             PET_HANDLE
     */
    static Handle getModulatorSimilarityLink(AtomSpace & atomSpace,
                                             const std::string & modulator, 
                                             const std::string & petId);

    /**
     * Return the Handle of SimilarityLink that holds DemandUpdater
     *
     * The format of a DemandGoal stored in AtomSpace is:
     *
     * SimultaneousEquivalenceLink
     * EvaluationLink
     *     PredicateNode: "demand_name_goal" 
     *                    (SimpleTruthValue indicates how well the demand is satisfied)
     *                    (ShortTermInportance indicates the urgency of the demand)
     * EvaluationLink
     *     GroundedPredicateNode: "FuzzyWithin"
     *     ListLink
     *         NumberNode: "min_acceptable_value"
     *         NumberNode: "max_acceptable_value"
     *         SimilarityLink (stv 1.0 1.0)
     *             NumberNode: "demand_value"
     *             ExecutionOutputLink
     *                 GroundedSchemaNode: "demand_schema_name"
     *                 ListLink
     *                     PET_HANDLE
     *
     */
    static Handle getDemandSimultaneousEquivalenceLink (const AtomSpace & atomSpace, 
                                                        const std::string & demand, 
                                                        const std::string & petId);

     /**
     * Return the EvaluationLinks (both DemandGoal and FUzzyWithin) of specific demand
     *
     * The format of a DemandGoal stored in AtomSpace is:
     *
     * SimultaneousEquivalenceLink
     * EvaluationLink
     *     PredicateNode: "demand_name_goal" 
     *                    (SimpleTruthValue indicates how well the demand is satisfied)
     *                    (ShortTermInportance indicates the urgency of the demand)
     * EvaluationLink
     *     GroundedPredicateNode: "FuzzyWithin"
     *     ListLink
     *         NumberNode: "min_acceptable_value"
     *         NumberNode: "max_acceptable_value"
     *         ExecutionOutputLink
     *             GroundedSchemaNode: "demand_schema_name"
     *
     */
    static bool getDemandEvaluationLinks (AtomSpace & atomSpace, 
                                          const std::string & demandName, 
                                          Handle & hDemandGoal, 
                                          Handle & hFuzzyWithin);

           
    /**
     * Return the schema strength for the given rule. This strength is stored as
     * the truth value of the ImplicationLink between the rule precondition and
     * the rule effect, i.e., the schema.
     *
     * @param atomSpace The AtomSpace
     * @param rule The rule name
     * @param agentModeName The name of the current agent operation mode
     * @return The ImplicationLink strength or -1 if an error occurs.
     */
    static float getRuleImplicationLinkStrength(AtomSpace& atomSpace,
            const std::string& rule,
            const std::string& agentModeName );

    /**
     * Return a vector containing the latest velocity coords of a given moving object
     *
     * @param atomSpace The AtomSpace
     * @param objectID The id of a moving object
     * @return A 3D Vector(x,y,z) indicating the current object velocity
     */
    static spatial::math::Vector3 getMostRecentObjectVelocity( const AtomSpace& atomSpace,
            const std::string& objectId,
            unsigned long afterTimestamp = 0 );

    /**
     * Return the most recent ExecutionLink for SchemaDone predicates, if any.
     *
     * @param atomSpace The AtomSpace
     * @param timestamp Timestamp used as a delimiter in SchemaDone predicate search
     * @param schemaSuccessful Indicates if SchemaDone or SchemaFailure should
     *                         be used during search. True = SchemaDone, False =
     *                         SchemaFailure
     * @return The ExecLink for the most recent schema done or Handle::UNDEFINED
     */
    static Handle getMostRecentPetSchemaExecLink(const AtomSpace& atomSpace,
            unsigned long timestamp,
            bool schemaSuccessful);

    /**
     * Given an ExecLink for a SchemaDone predicated, this method will return a formated string
     * containing all the action parameters
     *
     * @param atomSpace AtomSpace pointer
     * @param execLink An handle that points to and agent-action link
     * @return a formated string with the action parameters
     */
    static std::string convertPetExecLinkParametersToString(const AtomSpace& atomSpace,
            Handle execLink);


    /**
     * Return the object (structure or accessory) handle using its id.
     *
     * @note includes the concept nodes food_bowl, water_bowl and pet_home.
     *
     * @param atomSpace The AtomSpace
     * @param objectID The object's ID
     * @return The object's handle. UNDEFINE_HANDLE if the agent's doesn't exists
     */
    static Handle getObjectHandle ( AtomSpace& atomSpace, const std::string& objectID );


    /**
     * Return the agent handle using its agentID
     *
     * @param atomSpace The AtomSpace
     * @param agentID The agent's ID
     * @return The agent's handle. Handle::UNDEFINED if the agent doesn't exist.
     */
    static Handle getAgentHandle( AtomSpace& atomSpace, const std::string& agentID );

    /**
     * Return the entity handle using its agentID, it can be an object or an avatar
     *
     * @param atomSpace The AtomSpace
     * @param entityId The entity's ID
     * @return The object's handle. Handle::UNDEFINED if the agent doesn't exist.
     */
    static Handle getEntityHandle( AtomSpace& atomSpace,
                                   const std::string& entityId );

    /**
     * Return the Temporal pointed by a given AtTimeLink
     * It is assumed atTimeLink is of type AT_TIME_LINK
     * and well formed, otherwise a cassert is raised
     *
     * @param atTimeLink the Handle of the given AtTimeLink
     * @return a new Temporal object pointed by atTimeLink
     */
    static Temporal getTemporal(AtomSpace& as, Handle atTimeLink);

    /**
     * Return the Handle pointed by a given AtTimeLink
     * It is assumed atTimeLink is of type AT_TIME_LINK
     * and well formed, otherwise a cassert is raised
     *
     * @param atTimeLink the Handle of the given AtTimeLink
     * @return the Handle pointed by atTimeLink (second outgoing Atom)
     */
    static Handle getTimedHandle(AtomSpace& as, Handle atTimeLink);

    static void updateLatestAgentActionDone(AtomSpace& as,
                                            Handle atTimeLink,
                                            Handle agentNode);

    static void updateLatestPhysiologicalFeeling(AtomSpace& as,
                                                 Handle atTimeLink,
                                                 Handle predicateNode);

    static void updateLatestAvatarSayActionDone(AtomSpace& as,
                                                Handle atTimeLink,
                                                Handle avatarNode);

    static void updateLatestAvatarActionDone(AtomSpace& as,
                                             Handle atTimeLink,
                                             Handle avatarNode);

    static void updateLatestAvatarActionPredicate(AtomSpace& as,
                                               Handle atTimeLink,
                                               Handle predicateNode);

    static void updateLatestSpatialPredicate(AtomSpace& as,
                                             Handle atTimeLink,
                                             Handle predicateNode,
                                             Handle objectNode);

    static void updateLatestSchemaPredicate(AtomSpace& as,
                                            Handle atTimeLink,
                                            Handle predicateNode);

    static void updateLatestIsExemplarAvatar(AtomSpace& as,
                                             Handle atTimeLink);

    static void updateLatestModulator(AtomSpace & as, 
                                      Handle atTimeLink, 
                                      Handle modulatorPredicateNode); 

    static void updateLatestDemand(AtomSpace & as, 
                                   Handle atTimeLink, 
                                   Handle demandPredicateNode); 

    static void updateLatestFeeling(AtomSpace & as, 
                                    Handle atTimeLink, 
                                    Handle feelingPredicateNode); 
    
    static void updateLatestStimulus(AtomSpace & as, 
                                     Handle atTimeLink, 
                                     Handle stimulusPredicateNode);

    /**
     * Retrieve the handles of all Frame Elements (DefinedFrameElementNodes) 
     * given frame name. 
     *
     * @param AtomSpace the agent atomSpace
     * @param frameName the name of the FrameNet Frame
     * @param frameElementsHandles A HandleSeq which will be filled with
     *                             the Frame Elements handles
     *
     * @return The Frame handle. Handle::UNDEFINED if the frame doesn't exists
     *
     * @note this function will also return the frame elements of parent frames.
     */
    static Handle getFrameElements( AtomSpace& atomSpace, 
                                    const std::string& frameName, 
                                    HandleSeq& frameElementsHandles );

    /**
     * Creates a representation of a specific predicate instance
     * in Framenet(http://framenet.icsi.berkeley.edu/) Frames format. 
     * This method uses a previous definition of the Frame structure 
     * to set the given frame elements values.
     *
     * i.e. suppose you want to create a Frame corresponding to the
     * is_near predicate. So you will call this method giving the following
     * parameters: <agent atom space>, "#Locative_relation", "is_near",
     * HandleSeq(SemeNodeHandle(object1), SemeNodeHandle(object2), ConceptNodeHandle(is_near))
     *
     * Then, a Frame will be created as follows:
     *
     * <code>
     * InheritanceLink
     *    DefinedFrameNode "#Locative_relation"
     *    PredicateNode "is_near"
     *  
     * InheritanceLink
     *    DefinedFrameElementNode "#Locative_relation:Figure"
     *    PredicateNode "is_near_Figure"
     *  
     * InheritanceLink
     *    DefinedFrameElementNode "#Locative_relation:Ground"
     *    PredicateNode "is_near_Ground"
     *  
     * InheritanceLink
     *    DefinedFrameElementNode "#Locative_relation:Relation_type"
     *    PredicateNode "is_near_Relation_type"
     *  
     * FrameElementLink
     *    PredicateNode "is_near"
     *    PredicateNode "is_near_Figure"
     *  
     * FrameElementLink
     *    PredicateNode "is_near"
     *    PredicateNode "is_near_Ground"
     *  
     * FrameElementLink
     *    PredicateNode "is_near"
     *    PredicateNode "is_near_Relation_type"
     *  
     * EvaluationLink
     *    PredicateNode "is_near_Figure"
     *    SemeNode "object1"
     *  
     * EvaluationLink
     *    PredicateNode "is_near_Ground"
     *    SemeNode "object2"
     *  
     * EvaluationLink
     *    PredicateNode "is_near_Relation_type"
     *    ConceptNode "is_near"
     * </code>
     *
     * @param atomSpace The AtomSpace reference where the Frame will be create into
     * @param frameName The name of the Frame that will be instantiated
     * @param predicateName The name of the frame instance
     * @param frameElementsValuesHandles The map of the elements and its values that will be 
     *                                   used to set the frame instance elements     
     * @param truthValue The truthValue that will be set as the frame instance TV
     * @param permanent If true set the LTI of the frame atoms to 1, 0 otherwise
     *
     * @return The Frame instance handle
     */
    static Handle setPredicateFrameFromHandles(
            AtomSpace& atomSpace, 
            const std::string& frameName, 
            const std::string& frameInstanceName, 
            const std::map<std::string, Handle>& frameElementsValuesHandles, 
            TruthValuePtr truthValue,
            bool permanent = true );

    /**
     * Given a frame instance (PredicateNode handle) this method returns all the
     * frame element names and corresponding values related.
     *
     * @param atomSpace The AtomSpace reference
     * @param frameInstancePredicateNode The handle of the PredicateNode which
     *                                   represents the frame instance
     *
     * @return A Map containing the names and values of all the frame instance elements
     *
     * @note Below is an example. If the atomspace contains the info below, 
     *
     *       red@xxx_Color is an instance of Frame #Color
     *       (InheritanceLink (stv 1 1) 
     *           (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
     *           (DefinedFrameNode "#Color")
     *       )
     *
     *       red@xxx_Color_Entity belongs to Frame instance red@xxx_Color   
     *       (FrameElementLink (stv 1 1)
     *           (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
     *           (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
     *       )  
     *
     *       word ball@xxx is the value of (or stands for) the FrameElement instance red@xxx_Color_Entity
     *       (EvaluationLink (stv 1 1)
     *           (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
     *           (WordInstanceNode "ball@8631fad6-f29d-4b15-905c-8594fa1d27d3") 
     *       ) 
     *      
     *       then calling 
     *       getFrameElementInstanceNameValues( (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color") )
     *
     *       would return a std::map<std::string, Handle> holding the pair
     *       ( "Entity", (WordInstanceNode "ball@8631fad6-f29d-4b15-905c-8594fa1d27d3") )
     */
    static std::map<std::string, Handle> getFrameElementInstanceNameValues (
            AtomSpace& atomSpace, Handle frameInstancePredicateNode );

    /**
     * Given a value of a Frame instance element and the name of the frame.
     * this method returns a list of all PredicateNodes that represents
     * instances of the frame of name 'frameName', that has
     * 'aElementValue' as a value of at least one of its elements
     *
     * @param atomSpace The AtomSpace reference
     * @param frameName The name of the frame
     * @param aElementValue the element value
     *
     * @return A handleSeq containing all the matched predicateNodes
     */
    static HandleSeq retrieveFrameInstancesUsingAnElementValue (
            AtomSpace& atomSpace, const std::string& frameName, Handle aElementValue );

    /**
     * Excludes from the given AtomSpace an instance of a Frame represented
     * by the given frame instance PredicateNode handle
     *
     * @param atomSpace The AtomSpace reference
     * @param frameInstance Handle of the PredicateNode which represents the frame instance
     */
    static void deleteFrameInstance( AtomSpace& atomSpace, Handle frameInstance );

};


#endif /*ATOMSPACEUTIL_H_*/
