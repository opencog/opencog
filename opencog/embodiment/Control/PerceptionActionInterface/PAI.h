/*
 * opencog/embodiment/Control/PerceptionActionInterface/PAI.h
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
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

/** PAI.h
 *
 * Perception and Actions Interface.
 */
#ifndef PAI_H
#define PAI_H

#include <map>
#include <vector>
#include <exception>

#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

#include <opencog/atomspace/AtomSpace.h>

#include "AvatarAction.h"
#include "ActionPlan.h"
#include "ActionPlanSender.h"
#include <opencog/embodiment/Control/AvatarInterface.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/embodiment/Control/Language/LanguageComprehension.h>

#include "EmbodimentErrorHandler.h"

#ifdef HAVE_LIBPTHREAD
#include <pthread.h>
#endif

#ifdef HAVE_PROTOBUF
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.MapInfo.pb.h>
#endif

// If this is defined, uses decimal of seconds as time unit. Otherwise, uses centesimal of seconds.
// This is the time unit of the values returned by getTimestampFromXsdDateTimeStr() and getLatestSimWorldTimestamp().
//#define DATETIME_DECIMAL_RESOLUTION

using namespace opencog::control;
using namespace opencog::oac; 

using XERCES_CPP_NAMESPACE::DOMElement;
using XERCES_CPP_NAMESPACE::DOMDocument;

namespace opencog { namespace pai {

// action plan status
typedef enum {
    DONE,
    ERROR,
    NONE
} ActionStatus;

typedef Handle ActionID;
typedef map<ActionPlanID, ActionPlan> ActionPlanMap;
typedef map<unsigned int, ActionID> ActionIdMap;
typedef map<ActionPlanID, ActionIdMap> PlanToActionIdsMap;
typedef map<std::string, std::string> PropertyMap;

/** Perception and Actions Interface.
 *
 * This class provides the interface for receiving perceptions and send actions
 * from/to Embodiment Proxy
 */
class PAI
{

private:

    /**
     * Transform a string with upper cases by underscore+lower cases
     * e.g. bareTeeth -> bare_teeth
     *
     * This is used because the format of the pet combo vocabulary uses
     * underscore as visual separator while the format of the proxy uses upper
     * case
     */
    std::string camelCaseToUnderscore(std::string s);

    std::string camelCaseToUnderscore(const char* s);

    /**
     * The reference to the AtomSpace this PAI is working with.
     */
    AtomSpace& atomSpace;

    /**
     * The reference to an ActionPlanSender object this PAI uses to send action
     * plans.
     */
    ActionPlanSender& actionSender;

    /**
     * A reference to a AvatarInterface (actualy a pet)
     */
    AvatarInterface& avatarInterface;

    /**
     * The ID for the next action plan to be created
     */
    unsigned long nextActionPlanId;

    /**
     * A map whose keys are the IDs of all in progress (not sent to PVP yet)
     * action plans and the value is an ActionPlan object.
     */
    ActionPlanMap inProgressActionPlans;

    /**
     * A map whose keys are the IDs of all pending (already sent to PVP, but
     * that did not received a full feedback from it) action plans and the value
     * is an ActionPlan object.
     */
    ActionPlanMap pendingActionPlans;

    /**
     * A set of ids of the action plans that has failed (i.e., any of its
     * actions has received a fail status from PVP)
     */
    set<ActionPlanID> failedActionPlans;

    /**
     * A map whose keys are the IDs of all action plans (still in progress or
     * pending) and the value are maps from action sequence numbers to ActionIDs
     * inside each plan.
     */
    PlanToActionIdsMap planToActionIdsMaps;

    LanguageComprehension * languageTool;

#ifdef HAVE_LIBPTHREAD
    /**
     * A mutex variable to control the increment of action plan IDs.
     */
    pthread_mutex_t plock;
#endif

    /**
     * The latest timestamp received from the SimWorld (via Proxy) so far.
     **/
    unsigned long latestSimWorldTimestamp;

    XERCES_CPP_NAMESPACE::XercesDOMParser *parser;
    EmbodimentErrorHandler errorHandler;
    string schemaLocation;

    /**
     * Indicates if the pvp messages should be logged or not.
     */
    bool logPVPMessage;

    bool enableCollectActions;

    bool isFirstPerceptTerrian;

    int perceptTerrianBeginTime;

    int blockNum;

    Handle trueConceptNode;
    Handle falseConceptNode;

public:

    /**
     * Gets the latest received timestamp from the SimWorld Proxy. This
     * timestamp is an unsigned long value, which represents the number of
     * decimals of second since a specific date (EPOCH, which is defined
     * internally).
     */
    unsigned long getLatestSimWorldTimestamp();

    /**
     * Converts from xsd:DataTime format, as follows:
     *  [-]CCYY-MM-DDThh:mm:ss[Z|(+|-)hh:mm]
     *  Example: "2007-07-10T16:44:33.848-07:00"
     * ...to boost date_time format (like "2007-Jul-10 16:44:33.848000").
     *
     * Then, gets the ptime from it and convert it to a "unsigned long" that
     * represents the elapsed decimals of second since a specific date (EPOCH,
     * which is defined internally) used as reference.
     */
    static unsigned long getTimestampFromXsdDateTimeStr(
            const char* xsdDateTimeStr)
        throw (opencog::RuntimeException, std::bad_exception);

    /**
     * Retrieve timestamp from an element's TIMESTAMP_ATTIBUTE.
     * Better abstracted than getTimestampFromXsdDateTimeStr because
     * you don't have to deal with all the XML scaffolding crap.
     */
    unsigned long getTimestampFromElement(DOMElement* element) const;

    // get an avatar's weight
    double getAvatarWeight(Handle avatarNode);

    /**
     * Constructor
     *
     * @param _atomSpace Reference to AtomSpace which this PAI will add Atoms
     * related to received perceptions or sent actions in.
     * @param actionSender The reference to the object that will be used to
     * send action plans to the Virtual World.
     * @param nextPlanID The next plan ID to be used as return of the next
     * call to startActionPlan() method.  This argument may be useful to
     * continue a previous sequence of action plan IDs (when restarting OAC, for
     * instance). If not provided, assumes 0 as default.
     */
    PAI(AtomSpace& _atomSpace, ActionPlanSender& actionSender,
            AvatarInterface& avatarInterface, unsigned long nextPlanID = 0);

    /**
     * Destructor
     */
    ~PAI();

    /**
     * Gets the reference to the AtomSpace this PAI works with
     */
    AtomSpace& getAtomSpace();

    /**
     * Gets the reference to the AvatarInterface this PAI works with
     */
    AvatarInterface& getAvatarInterface();

    LanguageComprehension & getLanguageTool(void);

    /**
     * Creates an Action Plan XML message to be sent to the PVP. Once
     * this function in called it creates a new DOMDocument and
     * add the approprieted XML elements.
     *
     * @return The ActionPlanID value for the action plan identification.
     */
    ActionPlanID createActionPlan();

    /**
     * Sends an Action Plan XML message, that is, converted the XML
     * DOM tree into a string, encapsulates it in a message and send
     * the message to PVP.
     *
     * @param planId The identification of the action plan being sent.
     * @throw RuntimeException in the operation fails for the following
     * reasons:
     * - there is no action plan with the given ID (the plan may have
     * never been created or was already sent away before).
     * - the action plan could not be sent by the ActionPlanSender.
     */
    void sendActionPlan(ActionPlanID planId) throw (opencog::RuntimeException,
            std::bad_exception);

    /**
	 * In order to support multiple virtual world platforms and make 
	 * the action plan handling development less painful in the platform end, 
	 * we design to extract the actions from a plan and send them one by one 
	 * orderly.
	 */
	void sendExtractedActionFromPlan(ActionPlanID planId, 
                                     unsigned int actionSeqNum = 1) 
		throw (opencog::RuntimeException, std::bad_exception); 

    HandleSeq getActionSeqFromPlan(ActionPlanID planId);

    /**
     * Sends an Feelings XML message to PVP. Note that not all feelings are
     * sent, only the ones that where updated during last actions selection
     * iteration.
     *
     * @param petId The id of the pet
     * @param feelingsValueMap The map containing the feelings and its
     * updated values.
     */
    void sendEmotionalFeelings(const std::string& petId, const
            std::map<std::string, float>& feelingsValueMap);

    /**
     * Send a Demands XML message to PVP. 
     */
    void sendDemandSatisfactions(const std::string& petId,
        const std::map<std::string, float> & demandsValueMap); 

    /**
     * Insert an action into the an action-plan. Adds the corresponding
     * representation of the action in the AtomSpace which PAI is working with.
     *
     * @param planId The identification of the action plan where the action must
     * be inserted.
     * @param action A reference to the AvatarAction object containg
     * the action to be inserted into de actio plan.
     *
     * @return An action identifier used to track the execution of the action in
     * the SL World.
     * @throw RuntimeException if the action contains invalid parameters for its
     * type.
     */
    ActionID addAction(ActionPlanID planId, const AvatarAction& action) throw
        (opencog::RuntimeException, opencog::InvalidParamException,
         std::bad_exception);

    /**
     * Return true if and only if the ActionPlan corresponding to
     * a given ActionPlanID is empty or if there is no corresponding ActionPlan
     *
     * @param planId The identifier of the action plan to check
     *
     * @return true iff planId corresponds to an empty plan or no plan at all
     */
    bool isActionPlanEmpty(const ActionPlanID& planId);

    // send a single action command to the virtual world
    // not in a plan
    void sendSingleActionCommand(std::string& actionName, std::vector<ActionParamStruct> & paraList);

    /**
     * Process a XML message comming from PVP by adding the corresponding
     * representation of the perceptual data into the AtomSpace which this PAI
     * is working with.
     *
     * @param pvpMessage the PVP message in the XML format to be processed
     * @param toUpdateHandles a vector where the handles of all objects
     * added/update by the processing of a PVPMessage. This information is used
     * to update is_X predicates in the OAC (by PredicatesUpdater class).
     *
     * @return A boolean value for indicating if the PVP message was processed
     * successfully or not.
     */
    bool processPVPMessage(const string& pvpMessage, HandleSeq &toUpdateHandles);

    /**
     * Mark all pending actions as failed. This method should be called when
     * PROXY is down and hence the pending action plans won't have feedback.
     *
     * This method should be called only by the OAC
     */
    void setPendingActionPlansFailed();

    // TODO: TEMPORARY PUBLIC METHODS: They should become built-in predicates later.

    /**
     * Check if the action with the given id was done since the given timestamp.
     * @param actionId the id of the action
     * @param sinceTimestamp the timestamp since which the ActionDone mark of
     * the action will be checked for
     * @return true if the action is done, false otherwise
     *
     * @note You can use the getLatestSimWorldTimestamp() method just before
     * calling the addAction() method to get the timestamp argument for this
     * method.
     */
    bool isActionDone(ActionID actionId, unsigned long sinceTimestamp) const;

    bool isActionDone(ActionPlanID planId, unsigned int seqNumber) const;

    /**
     * Check if the action with the given id failed since the given timestamp.
     *
     * @param actionId the id of the action
     * @param sinceTimestamp the timestamp since which the ActionFailed mark of
     * the action will be checked for
     * @return true if the action failed, false otherwise
     *
     * @note You can use the getLatestSimWorldTimestamp() method just before
     * calling the addAction() method to get the timestamp argument for this
     * method.
     */
    bool isActionFailed(ActionID actionId, unsigned long sinceTimestamp) const;

    bool isActionFailed(ActionPlanID planId, unsigned int seqNumber) const;

    /**
     * Check if the action with the given id was already tried since the given
     * timestamp.
     * @param actionId the id of the action
     * @param sinceTimestamp the timestamp since which the ActionTried mark of
     * the action will be checked for
     * @return true if the action was tried, false otherwise
     *
     * @note you can use the getLatestSimWorldTimestamp() method just before
     * calling the addAction() method to get the timestamp argument for this
     * method.
     */
    bool isActionTried(ActionID actionId, unsigned long sinceTimestamp) const;

    /**
     * Check if the action plan with the given id is finished, i.e., all its
     * actions are done or failed.
     * @param planId the id of the action plan
     */
    bool isPlanFinished(ActionPlanID planId) const;

    /**
     * Check if the action plan with the given id has failed, i.e., if any of
     * its actions has failed, despite of the plan (or the remaining actions)
     * has already finished or not.
     * @param planId the id of the action plan
     */
    bool hasPlanFailed(ActionPlanID planId) const;

    static std::string getObjectTypeFromHandle(const AtomSpace& atomSpace, Handle objectH);

    bool hasFinishFistTimeWorldPercetption() const
    {
        return (! isFirstPerceptTerrian) ;
    }

private:

    /**
     * Sets the latest received timestamp from the SimWorld Proxy. This
     * timestamp is an unsigned long value, which represents the number of
     * decimals of second since a specific date (EPOCH, which is defined
     * internally).
     *
     * @return true if the new timestamp is greater than or equals to the latest
     * received timestamp. Otherwise, return false.
     */
    bool setLatestSimWorldTimestamp(unsigned long timestamp);

    /**
     * @param doc The parsed XML file to be processed
     * @param toUpdateHandles a vector where the handles of all objects
     * added/update by the processing of a PVPMessage. This information is used
     * to update is_X predicates in the OAC (by PredicatesUpdater class).
     */
    void processPVPDocument(DOMDocument * doc, HandleSeq
            &toUpdateHandles);

    /**
     * @param element The agent-sensor-info element to be processed
     */
    void processAgentSensorInfo(DOMElement * element);

    /**
     * @param element The avatar-signal element to be processed
     */
    void processAvatarSignal(DOMElement * element) throw
        (opencog::RuntimeException, opencog::InvalidParamException,
         std::bad_exception);

    //! Supported method for processAvatarSignal
    Handle processAgentType(const string& agentTypeStr, const string& internalAgentId);
    //! Support method from processAvatarSignal
    Handle processAgentActionParameter(DOMElement* paramElement,const std::string& actionNameStr, Handle& actionInstancenode, std::string&changedStateName,  Handle& newStateValNode, std::vector<Handle> &actionConcernedHandles);
    //! Support method from processAvatarSignal
    void processAgentActionWithParameters(Handle& agentNode, const string& internalAgentId, unsigned long tsValue, const string& nameStr, const string& instanceNameStr, DOMElement* signal);
    void processAgentActionPlanResult(char* agentID, unsigned long tsValue, const string& name, char* planIdStr, DOMElement* signal);

    /**
     * This function will process the availability status of an action in the
     * atomspace. In the virtual world, some actions are available only when the
     * avatar is close enough to an object. e.g. When the avatar is close to a
     * soccer, it will get the ability of "Kick". 
     * Thus to be a convention, the atom of an available action can be represented
     * as:
     *      EvaluationLink (stv 1 1) (av 1 1 1)
     *          PredicateNode "can_do"
     *          ListLink
     *              "#Kick"
     *              "#actor"
     *              "#target"
     * Otherwise when the action is no longer available, just set the truth value of
     * evaluation link to 0.
     */
    Handle processActionAvailability(DOMElement* signal);

    /**
     * @param element The agent-signal element to be processed
     */
    void processAgentSignal(DOMElement * element) throw
        (opencog::RuntimeException, opencog::InvalidParamException,
         std::bad_exception);

    /**
     * @param element The instruction element to be processed. 
     *
     * @note The instruction can be a command or any English sentences parsed by
     *        RelexServer. This function only update the AtomSpace based on the
     *        message and the actual work like generating corresponding answers
     *        or actions is done within PsiActionSelectionAgent and dialog_system.scm
     */
    void processInstruction(DOMElement * element);

    /**
     * @param signal The state-info element to be processed.
     *
     * @note When a robot is loaded, there are many existing states of the objects in the virtual world.
     *       This function is to save all this existing states into the Atomspace.
     *       It will only runs for one time, just at the very beginning of a robot.
     */
    void processExistingStateInfo(DOMElement* element);

    void processBlockStructureSignal(DOMElement* element);

    void processFinishedFirstTimePerceptTerrianSignal(DOMElement* element, HandleSeq &toUpdateHandles);


    /**
     * @param element The param element to be processed.
     *
     * @note Get value hanle from a param doc
     */
    Handle getParmValueHanleFromXMLDoc(DOMElement* paramElement);

    /** 
     * Extract double attributes from XML message.
     * @param tagName the name of the attribute.
     * @return the value of the attribute parsed as a double.
     */
    double getPositionAttribute(DOMElement * element, const char* tagName);

    /**
     * Extract int attributes from XML message.
     * @param tagName the name of the attribute.
     * @return the value of the attribute parsed as a int.
     */
    int getIntAttribute(DOMElement * element, const char* tagName);

    /**
     * Extract string attributes from XML message.
     * @param tagName the name of the attribute.
     * @return the value of the attribute parsed as a string.
     */
    string getStringAttribute(DOMElement * element, const char* tagName);

    /**
     * @param element The map-info element to be processed
     * @param toUpdateHandles a vector where the handles of all objects
     * added/update by the processing of a PVPMessage. This information is used
     * to update is_X predicates in the OAC (by PredicatesUpdater class).
     */
    //void processMapInfo(DOMElement* element, HandleSeq& toUpdateHandles);

    /**
     * Override processMapInfo to support features of protobuf. If protobuf is
     * found, the function will process the map info by using protobuf.
     * Otherwise, old function that uses XML parser will be invoked.
     *
     * @param element The map-info element to be processed
     * @param toUpdateHandles a vector where the handles of all objects
     * @param useProtoBuf the flag to enable protobuf
     */
    void processMapInfo(DOMElement* element, HandleSeq& toUpdateHandles, bool useProtoBuf);

    /**
     * Retrieve velocity vector data from a velocity XML element.
     *
     * @param velocityElement The velocity element to be processed
     */
    Vector getVelocityData(DOMElement* velocityElement);

    /**
     * Adds the representation of the given action in the AtomSpace.
     * @param planId for the given action
     * @param action action to be represented
     * @return the Handle of the action representation.
     */
    Handle addActionToAtomSpace(ActionPlanID planId, const AvatarAction& action)
        throw (opencog::RuntimeException, opencog::InvalidParamException,
                std::bad_exception);

    /**
     * Adds the representation of the predicate about a status of a given action
     *
     * @param predicateName the name of the predicate (actionTried, actionDone
     * or actionFailed)
     * @param action a referent to the AvatarAction object
     * @param timestamp an unsigned long value that represents the timestamp of
     * this predicate.
     * @param actionId the id that identifies the action (with the exact
     * arguments)
     */
    Handle addActionPredicate(const char* predicateName, const AvatarAction&
            action, unsigned long timestamp, ActionID actionId);

//    /**
//     * Get the global space map information from network stream and set it into
//     * space server.
//     *
//     * @param element the XML Dom element that contains the information
//     */
//    void addSpaceMapBoundary(DOMElement * element);

    // when first time enter a new map, all the spaceServer to add a new spaceMap
    void addSpaceMap(DOMElement * element,unsigned long timestamp);

    /**
     * Adds the representation of the predicates about space info of a given
     * object (position, rotation and size)
     *
     * @param objectNode    The handle of the node that represents the object
     * @param timestamp an unsigned long value that represents the timestamp of
     * this predicate.
     * @param positionElement   The DOMElement with the position arguments
     * @param rotationElement   The DOMElement with the rotation arguments
     * @param length the length of the object's bounding box.
     * @param width the witdh of the object's bounding box.
     * @param height the height of the object's bounding box.
     * @param isEdible if the object is edible.
     * @param isDrinkable if the object is drinkable.
     * @param isTerrainObject if the object is a terrain object.
     * @return true if any property of the object has changed (or it's a new object). False, otherwise.
     */
   /* bool addSpacePredicates(Handle objectNode, unsigned long timestamp,
                            DOMElement* positionElement, DOMElement* rotationElement,
                            double length, double width, double height, bool isTerrainObject,bool isFirstTimePercept);
    */
    /**
     * Add a property predicate in atomSpace
     *
     * @param predicateName The name of the predicate to be added
     * @param objectNode  The handle of the node that represents the object
     * @param propertyValue The value indicating if the object has or not the
     * given property
     * @param permanent An optional flag to indicate If the property must be
     * kept forever (or until it is explicitly removed). It's false by default
     */
    void addPropertyPredicate(std::string predicateName, Handle objectNode,
                              bool propertyValue, bool permanent = false);

    /**
     * Add an inheritance link between a concept node, whose name is given
     * as a function parameter, and an ObjectNode whose handle is also a
     * parameter.
     *
     * @param conceptNodeName The name of the concept node. This is the
     *        base node in the inheritance relation.
     * @param subNodeHandle The handle of the ObjectNode that inheritates
     *        from the concept node.
     * @param inheritanceValue Boolean used to indicate if the inheritance
     *        relation is valid or not.
     */
    void addInheritanceLink(std::string conceptNodeName,
                            Handle subNodeHandle,
                            bool inheritanceValue);

    void addInheritanceLink(Handle fatherNodeHandle, Handle subNodeHandle);

    /**
     * Adds a vector-type (e.g., "position" and "velocity") predicate for the
     * given object
     *
     * @param objectNode The handle of the node that represents the object
     * @param timestamp an unsigned long value that represents the timestamp of
     * this predicate.
     * @param vectorElement The DOMElement with the position arguments
     * @return a Vector object with the (x,y,z) position coordinates
     */
    Vector addVectorPredicate(Handle objectNode, const std::string&
            predicateName, unsigned long timestamp,
            DOMElement* vectorElement);

    Handle addVectorPredicate(Handle objectNode, const std::string& predicateName, 
            const Vector& vec);

    void addVectorPredicate(Handle objectNode, const std::string& predicateName, 
            const Vector& vec, unsigned long timestamp);

    /**
     * Adds a rotation predicate for the given object
     * @param objectNode    The handle of the node that represents the object
     * @param timestamp an unsigned long value that represents the timestamp of
     * this predicate.
     * @param rotationElement   The DOMElement with the rotation arguments
    * @return a Rotation object with the (pitch,roll,yaw) rotation values
     */
    Rotation addRotationPredicate(Handle objectNode, unsigned long timestamp,
            DOMElement* rotationElement);

    void addRotationPredicate(Handle objectNode, const Rotation& rot, unsigned long timestamp);

    /**
     * Add semantic structure of an entity into atomspace, which would be used
     * in language comprehension. A frame would be created will be created to
     * represent this entity.
     *
     * @param entityId unique id of entity.
     * @param entityClass class of entity.
     * @param entityType type of entity.
     */
    void addSemanticStructure(Handle objectNode, const std::string& entityId, 
            const std::string& entityClass, const std::string& entityType);




    /**
     * Adds a physiological feeling into the AtomSpace
     *
     * @param paramName   the name of the feeling parameter
     * @param paramValue  the value of the feeling parameter
     *
     * @return the Handle of the ListLink that contains the name and value nodes.
     * 
     * @note 
     *
     *     ListLink
     *         Node "paramName"
     *         Node "paramValue"
     */
    Handle addPhysiologicalFeelingParam(const char* paramName, const char* paramValue);

    /**
     * Adds a physiological feeling into the AtomSpace
     *
     * @param petID          the id of the pet
     * @param name           the name of the feeling
     * @param timestamp      a string in the xsd:datetime format representing the timestamp of this feeling
     * @param feelingParams  the sequence of Handles for each one of the feeling parameters.
     *
     * @return the Handle of the EvalLink that represents the feeling
     *
     * @note
     *
     *     AtTimeLink
     *         TimeNode "timestamp"
     *         EvaluationLink
     *             PredicateNode "petId.name"
     *             ListLink
     *                 ListLink 
     *                     Node "paramName1"
     *                     Node "paramValue1"
     *                 ListLink
     *                     Node "paramName2"
     *                     Node "paramValue2"
     *                 ...    
     */
    Handle addPhysiologicalFeeling(const string petID,
                                   const string name,
                                   unsigned long timestamp,
                                   float level);

    /**
     * Add a ownership predicate between the object.
     *
     * @param owner The handle of the owner of the object.
     * @param owned The handle of the owned objetc.
     * @param isPetObject If the owned handle is of the pet itself. In this
     * case, the added atom should not be forgotten.
     * @return The handle of the evaluation link of the ownership relation
     * relation.
     */
    Handle addOwnershipRelation(const Handle owner, const Handle owned, bool
            isPetObject);

    /**
     * Gets the right Node type for a given entity type
     * @param a String with the entity type
     * @return the proper Atom Type for the given string with the entity type
     */
    Type getSLObjectNodeType(const char* entityType);

    /**
     * Set the pending action plan sequence according to the status code passed.
     * If sequenceStr is NULL all actions are considered to have the same
     * status code.
     *
     * @param planIdStr The str representating the action plan id
     * @param sequenceStr The str representing the action sequence number
     * @param statusCode The status of the action (DONE or ERROR).
     */
    void setActionPlanStatus(ActionPlanID& planId, unsigned int sequence,
                             ActionStatus statusCode, unsigned long timestamp);

    #ifdef HAVE_PROTOBUF
    /** -----------------------------------------------------------------------
     * Functions that depends on google protocol buffer library.
     * -----------------------------------------------------------------------*/

    /**
     * Process the terrain information from minecraft-like world. The processed
     * result is then stored in local space map.
     */
    void processTerrainInfo(DOMElement * element, HandleSeq &toUpdateHandles);

    /**
     * The 2D local space map is now to be replaced by 3D block-octree map.
     * Currently, since we haven't finish all the functions of the 3D map, we still keep 2D map working as well.
     * Process the terrain information from minecraft-like world. The processed
     * result is then stored in Octree3DMapManager.
     * In this function, we only process the changes of blocks
     */
    void process3DSpaceTerrainInfo(Handle objectNode, const MapInfo& mapinfo, bool isFirstTimePerceptWorld, unsigned long tsValue);
 
    /**
     * Adds the entity information processed from map info into atomspace, the
     * information includes:
     *      entity id, entity type, entity name
     *      properties: edible, drinkable, is_home, is_food_bowl, is_water_bowl
     *                  color, material, texture
     *      geographic: position, rotation, velocity
     *
     * @param mapinfo the map info instance to be processed.
     * @param timestamp time stamp of the map info.
     *
     * @return handle of the entity node after insertion.
     */
    Handle addEntityToAtomSpace(const MapInfo& mapinfo, unsigned long timestamp, bool isFirstTimePercept);

    /**
     * Remove entity information from space server and mark it as non-existent
     * in atomspace.
     *
     * @param mapinfo the map info to be removed.
     * @param timestamp time stamp of the map info.
     *
     * @return handle of the entity to be removed.
     */
    Handle removeEntityFromAtomSpace(const MapInfo& mapinfo, unsigned long timestamp);

    bool addSpacePredicates( Handle objectNode, const MapInfo& mapinfo, bool isSelfObject, unsigned long timestamp,bool isFirstTimePercept);

    /**
     * Add property predicates of an entity such as edible, drinkable, material 
     * and so on into atomspace. If the entity is the avatar itself, then
     * specific stuffs should be done.
     *
     * @param objectNode the handle of object node.
     * @param isSelfObject the flag to see if the entity is the avatar itself.
     * @param mapinfo the map info instance containing information of this entity.
     */
    void addEntityProperties(Handle objectNode, bool isSelfObject, const MapInfo& mapinfo);

    /**
     * Add semantic structure of an entity into atomspace, which would be used
     * in language comprehension. A frame would be created will be created to
     * represent this entity.
     *
     * @param objectNode handle of the object that would be the topic.
     * @param mapinfo the map info instance containing all information of the entity.
     */
    void addSemanticStructure(Handle objectNode, const MapInfo& mapinfo);

    /**
     * Get property map from a map info.
     *
     * @param mapinfo the map-info instance.
     * @return the property map of passed in map-info.
     */
    PropertyMap getPropertyMap(const MapInfo& mapinfo);

    /**
     * Get the value of a property item via its key.
     *
     * @param properties the map of property.
     * @param the key of a property item.
     *
     * @return the value of a property, if not existing, return a default "NULL"
     * value.
     */
    std::string queryMapInfoProperty(const PropertyMap& properties, const std::string& key) const;

    /**
     * A handy function to test if the string value equals to "true".
     *
     * @param value a string to be compared, supposed to be either "true" or "false"
     * @return true if the param string equals to "true"
     */
    bool isPropertyTrue(const std::string& value) const;

    /**
     * Get boolean value of a given attribute key.
     *
     * @param properties the property map contained in a map info instance.
     * @param key the key of attribute to be retrieved.
     *
     * @return the boolean value of the property.
     */
    bool getBooleanProperty(const PropertyMap& properties, const std::string& key) const;

    /**
     * Get string value of a given attribute key.
     *
     * @param properties the property map contained in a map info instance.
     * @param key the key of attribute to be retrieved.
     *
     * @return the string value of the property.
     */
    std::string getStringProperty(const PropertyMap& properties, const std::string& key) const;

    /**
     * Check the visibility status of an object.
     *
     * @param properties the property map contained in a map info instance.
     * @return true if the object is visible.
     */
    bool isObjectVisible(const PropertyMap& properties) const;

    /**
     * Check if an object is a terrain object.
     *
     * @param entityClass the entity class.
     * @return true if the object is a terrain object.
     */
    bool isObjectBelongToTerrain(const std::string& entityClass) const;

    /**
     * Check if an object is an obstacle.
     *
     * @param objectNode the handle of an object node.
     * @param entityClass the entity class.
     * @param mapinfo the map info instance containing the information of this
     * object.
     * @return true if the object is an obstacle.
     */
    bool isObjectAnObstacle(Handle objectNode, const std::string& entityClass, const MapInfo& mapinfo) const;


    #endif


}; // class


} }  // namespace opencog::pai

#endif // PAI_H 
