/*
 * opencog/embodiment/AGISimSim/proxy/SimProxy.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by TO_COMPLETE
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
#ifndef __SIM_PROXY_H__
#define __SIM_PROXY_H__

#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <opencog/util/platform.h>
#include "AsynchronousMessageReceiver.h"
#include "AsynchronousPerceptionAndStatusHandler.h"

#include <map>
#include <deque>

/**
 * AGISIM Command constants
 */

#define AGISIM_ACTION "action"

// Agent movement:
#define AGISIM_TURN_LEFT "turn.left"
#define AGISIM_TURN_RIGHT "turn.right"
#define AGISIM_MOVE_FORWARD "move.forward"
#define AGISIM_MOVE_BACKWARD "move.backward"
#define AGISIM_STRAFE_LEFT "strafe.left"
#define AGISIM_STRAFE_RIGHT "strafe.right"
#define AGISIM_WALK_TOWARDS "walk.towards"
#define AGISIM_NUDGE_TO "nudge.to"
#define AGISIM_TURN_TO "turn.to"

// Eye movement:
#define AGISIM_EYE_UP "eye.up"
#define AGISIM_EYE_DOWN "eye.down"
#define AGISIM_EYE_LEFT "eye.left"
#define AGISIM_EYE_RIGHT "eye.right"

// Misc:
#define AGISIM_EAT "eat"
#define AGISIM_DRINK "drink"
#define AGISIM_LIFT "lift"
#define AGISIM_DROP "drop"
#define AGISIM_THROW "throw"
#define AGISIM_THROW_AT "throw.at"
#define AGISIM_KICK_LOW "kick.low"
#define AGISIM_KICK_HIGH "kick.high"
#define AGISIM_SMILE "smile"
#define AGISIM_FROWN "frown"

#define AGISIM_GOTO "goto"

// Communication:
#define AGISIM_NOISE_MAKE "noise.make"
//#define AGISIM_MESSAGE "message"
#define AGISIM_MESSAGE "msg"

// Internal State Commands: Use these ops after the stem object:
#define AGISIM_ON "on"
#define AGISIM_OFF "off"

// Turning specific senses on / off:
#define AGISIM_SIGHT "sight."
#define AGISIM_SMELL "smell."
#define AGISIM_TASTE "taste."
#define AGISIM_HEARING "hearing."
#define AGISIM_TOUCH "touch."

// Admin commands
#define AGISIM_NEW "new"
#define AGISIM_RESET "reset"
#define AGISIM_SENSE "sense"
#define AGISIM_CONFIG "config"

#define AGISIM_FRAME_UPDATE_DELAY "FrameUpdateDelay"

// Object Profiles

#define AGISIM_AGENT "Agent"
#define AGISIM_DEMON "demon"
#define AGISIM_FOOD "food"
#define AGISIM_LIGHTBALL "lightball"
#define AGISIM_HEAVYBALL "heavyball"
#define AGISIM_POISON "poison"
#define AGISIM_CANDY "candy"

// Constants for AGISIM XML responses

/*
<agisim>
  <msg> ok </msg>
</agisim>

<agisim>
  <msg> Agent </msg>
</agisim>
*/
#define AGISIM_ADMIN_ROOT_TAG "agisim"
#define AGISIM_ADMIN_MSG_TAG "msg"

/*
<agisim>
  <error> Empty command. </error>
</agisim>
*/
#define AGISIM_ADMIN_ERROR_TAG "error"

/*
<sensation>
...
</sensation>
*/
#define AGISIM_SENSATION_ROOT_TAG "sensation"

/* THIS IS THE BASIC FORMAT (SINGLE_COMPONENT MODE):
  <objectvisual>
    <object>
      <fovpos>
        <x> 600 </x>
        <y> 300 </y>
        <dist> 2589 </dist>
      </fovpos>
      <name> HeavyBall1/null </name>
      <brightness> 0 </brightness>
    </object>
  </objectvisual>
*/
/* THIS IS THE NEW FORMAT USING POLYGONS (MULTIPLE_COMPONENT MODE):
  <objectvisual>
    <object>
      <name> door_overlay </name>
      <polygonname> south </polygonname>
      <brightness> 0 </brightness>
      <corner> (-2500,3400,-100) </corner>
      <corner> (0,0,-100) </corner>
      <corner> (-2500,0,-100) </corner>
      <corner> (0,3400,-100) </corner>
    </object>
*/

#define AGISIM_OBJECT_VISUAL_TAG "objectvisual"
#define AGISIM_OBJECT_TAG "object"
#define AGISIM_NAME_TAG "name"
#define AGISIM_POLYGON_NAME_TAG "polygonname"
#define AGISIM_POLYGON_CORNER_TAG "corner"
#define AGISIM_BRIGHTNESS_TAG "brightness"
#define AGISIM_MAP_INFO_TAG "map-info"

/*
  <visual>
    <pixel>
      <color>
        <r> 80 </r>
        <g> 164 </g>
        <b> 188 </b>
      </color>
      <fovpos>
        <x> 20 </x>
        <y> 0 </y>
        <dist> 9066 </dist>
      </fovpos>
    </pixel>
  </visual>
*/
#define AGISIM_VISUAL_TAG "visual"
#define AGISIM_PIXEL_TAG "pixel"
#define AGISIM_COLOR_TAG "color"
#define AGISIM_RED_TAG "red"
#define AGISIM_GREEN_TAG "green"
#define AGISIM_BLUE_TAG "blue"
#define AGISIM_FOV_POS_TAG "fovpos"
#define AGISIM_X_TAG "x"
#define AGISIM_Y_TAG "y"
#define AGISIM_DIST_TAG "dist"

/*
  <selfdata>
    <energy> 100 </energy>
    <position> (0.00,0.16,0.00) </position>
    <orientation> (0.00,0.00,0.00,0.10) </orientation>
    <eye>
      <orientation> (0.00,0.00) </orientation>
    </eye>
    <custom>
      <msg> Yum! Gained energy: 100 </msg>
      <quality> 101 </quality>
      <intensity> 1 </intensity>
    </custom>
  </selfdata>
*/
#define AGISIM_SELF_DATA_TAG "selfdata"
#define AGISIM_ENERGY_TAG "energy"
#define AGISIM_POSITION_TAG "position"
#define AGISIM_ORIENTATION_TAG "orientation"
#define AGISIM_EYE_TAG "eye"
#define AGISIM_CUSTOM_TAG "custom"
#define AGISIM_MSG_TAG "msg"
#define AGISIM_QUALITY_TAG "quality"
#define AGISIM_INTENSITY_TAG "intensity"

/*
  <smelldata>
    <smell>
      <quality> 666 </quality>
      <intensity> 1 </intensity>
    </smell>
  </smelldata>
*/
#define AGISIM_SMELL_DATA_TAG "smelldata"
#define AGISIM_SMELL_TAG "smell"

/*
  <tastedata>
    <taste>
      <quality> 2 </quality>
      <intensity> 1 </intensity>
    </taste>
  </tastedata>
*/
#define AGISIM_TASTE_DATA_TAG "tastedata"
#define AGISIM_TASTE_TAG "taste"

/*
  <audiodata>
    <sound>
      <quality> 525 </quality>
      <intensity> 10 </intensity>
    </sound>
  </audiodata>
*/
#define AGISIM_AUDIO_DATA_TAG "audiodata"
#define AGISIM_SOUND_TAG "sound"

/*
  <map-info>
    <object>
      <name> fido </name>
      <type> pet </type>
      <pos> 6.5,7.5,0 </pos>
      <rot> 0,0,0 </rot>
      <dim> 1,14,14 </dim>
      <edible> false </edible>
      <drinkable> false </drinkable>
    </object>
  </map-info>

or

  <map-info>
    <object>
      <name> ball </name>
      <type> accessory </type>
      <remove> true </remove>
    </object>
  </map-info>
*/

#define AGISIM_OBJECT_NAME_TAG "name"
#define AGISIM_OBJECT_TYPE_TAG "type"
#define AGISIM_OBJECT_POSITION_TAG "pos"
#define AGISIM_OBJECT_ROTATION_TAG "rot"
#define AGISIM_OBJECT_SIZE_TAG "dim"
#define AGISIM_OBJECT_EDIBLE_TAG "edible"
#define AGISIM_OBJECT_DRINKABLE_TAG "drinkable"
#define AGISIM_OBJECT_PETHOME_TAG "petHome"
#define AGISIM_OBJECT_FOODBOWL_TAG "foodBowl"
#define AGISIM_OBJECT_WATERBOWL_TAG "waterBowl"
#define AGISIM_OBJECT_REMOVE_TAG "remove"

#define PROXY_USING_CUSTOM_SENSATION_QUALITY
#ifdef PROXY_USING_CUSTOM_SENSATION_QUALITY
/*
 * Custom Sensation quality values
 * NOTE: The constants bellow were extracted from AGISim project: agisim/shared/include/world_vocabulary.h
 *       Please check periodically if they are up-to-date.
 */
// Original events
#define CUSTOM_SENSATION_BROADCAST_MESSAGE 110
#define CUSTOM_SENSATION_AGENT_DEATH 0
#define CUSTOM_SENSATION_NO_OBJECT_TO_EAT 104
#define CUSTOM_SENSATION_OBJECT_TOO_FAR_TO_EAT 102
#define CUSTOM_SENSATION_OBJECT_WITHOUT_ENERGY_CANNOT_BE_EAT 103
#define CUSTOM_SENSATION_ENERGY_GAIN 100
#define CUSTOM_SENSATION_ENERGY_LOSS 101
#ifdef USE_ACTUATOR_BASED_EVENTS
// Actuator-related events
#define CUSTOM_SENSATION_LEG_ACTUATOR 500
#define CUSTOM_SENSATION_LEG_ACTION_STARTED 501
#define CUSTOM_SENSATION_LEG_ACTION_DONE 502
#define CUSTOM_SENSATION_LEG_ACTION_FAILED 503
#define CUSTOM_SENSATION_ARM_ACTUATOR 600
#define CUSTOM_SENSATION_ARM_ACTION_STARTED 601
#define CUSTOM_SENSATION_ARM_ACTION_DONE 602
#define CUSTOM_SENSATION_ARM_ACTION_FAILED 603
#define CUSTOM_SENSATION_HEAD_ACTUATOR 700
#define CUSTOM_SENSATION_HEAD_ACTION_STARTED 701
#define CUSTOM_SENSATION_HEAD_ACTION_DONE 702
#define CUSTOM_SENSATION_HEAD_ACTION_FAILED 703
#else
// Animation-type-related events
#define CUSTOM_SENSATION_MOVE_ANIMATION 500
#define CUSTOM_SENSATION_MOVE_ANIMATION_STARTED 501
#define CUSTOM_SENSATION_MOVE_ANIMATION_DONE 502
#define CUSTOM_SENSATION_MOVE_ANIMATION_FAILED 503
#define CUSTOM_SENSATION_TURN_ANIMATION 600
#define CUSTOM_SENSATION_TURN_ANIMATION_STARTED 601
#define CUSTOM_SENSATION_TURN_ANIMATION_DONE 602
#define CUSTOM_SENSATION_TURN_ANIMATION_FAILED 603
#define CUSTOM_SENSATION_GENERIC_ANIMATION 700
#define CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED 701
#define CUSTOM_SENSATION_GENERIC_ANIMATION_DONE 702
#define CUSTOM_SENSATION_GENERIC_ANIMATION_FAILED 703
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION 800
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION_STARTED 801
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION_DONE 802
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION_FAILED 803
#endif
#endif // PROXY_USING_CUSTOM_SENSATION_QUALITY
// Aditional events
#define CUSTOM_SENSATION_HOLDING_OBJECT 900

class DefaultPerceptionAndStatusHandler : public AsynchronousPerceptionAndStatusHandler
{
    virtual ~DefaultPerceptionAndStatusHandler();
    void mapInfo(std::vector<ObjMapInfo>& objects);
    void actionStatus(unsigned long actionTicket, bool success);
    void errorNotification(const std::string& errorMsg);
};

// External references
class SocketHandler;
class SimClientSocket;

class SimProxy : public AsynchronousMessageReceiver
{
private:
    pthread_t checkAsynchronousMessagesThread;
    bool checkingAsynchronousMessages;

public:
    SimProxy(const std::string &host, unsigned short port, AsynchronousPerceptionAndStatusHandler* handler, bool echoing = false);
    ~SimProxy();

    static void* ThreadCheckAsynchronousMessages(void* args);

    // Implementation of abstract methods from superclass
    void receiveAsynchronousMessage(const std::string&);

    // CONNECTION COMMANDS

    /**
     * Connects to an AGI-Sim server using the server address arguments received in the constructor.
     */
    bool connect();
    /**
     * Check if this SimProxy is connected to an AGI-Sim server
     */
    bool IsConnected();

    bool hasPendingActions();

    /**
     * This method checks for assynchronous messages comming from the connected AGI-Sim server.
     * If there is any incomming message, it calls the receiveAsynchronousMessage to handle it,
     * which may eventually call the receiveAsynchronousPerceptionAndStatus() method of the  AsynchronousPerceptionAndStatusHandler
     * object passed as argument in the constructor.
     * This method must be called periodically by the SimProxy user so that it receives all
     * sensations or admin responses from AGI-Sim server.
     */
    void checkAsynchronousMessages();

    // AGISIM ADMIN COMMANDS (if successfull, returns an empty string. Otherwise return an error message)

    /**
     * Creates a new agent in the connected AGI-Sim world/server.
    * @param x,y,z Agent's position coordinates
    * @param objectType one of the following element in the set ("avatar", "pet", "accessory", "object", "structure", "unknown")
    * @param meshType one of the following types ("ball", "box"). NOTE: for now, just use "ball"
    * @param radius the radius of the agent's body
    * @param agentBaseName the base name of the agent to be created (if it already exists, this name is suffixed with a seq number)
     * If successfully, returns the name of the just created Agent. Otherwise, returns an empty string.
     */
    std::string newAgent(float x, float y, float z,  const char* objectType , const char*  meshType, float radius, const char* agentBaseName = AGISIM_AGENT);
    /**
     * Gets the name of the current Agent created with newAgent() method.
     * If no Agent is created yet, return an empty string.
     */
    std::string getAgentName();
    /**
     * Resets the AGI-Sim world in a soft way (same as "reset soft" command), setting
     * all world objects to their initial positions and the Agent's attributes to their
     * initial values as well.
     */
    std::string resetCurrentWorld();
    //std::string reconfigWorld(std::string &simScriptFilename);
    /**
     * Sets the AGI-Sim UpdateFrameDelay config variable to a very high value, so that
     * AGI-Sim server do not increase its clock spontaneously.
     */
    std::string disableAgiSimClock();

    /**
     * This is like an action command with no action, just to force current agent sensation
     * update. This also forces a clock tick in the connected AGI-Sim world.
    * @return true if the command was successfully sent
     */
    bool getCurrentSense();

    /**
     * Gost to the object with the given name
     * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
            *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long moveGoto(std::string objName);

    // Special "admin" commands

    /**
     * Gets the position of the object with the given name
     * Returns the next raw response after sending this command:
     * Example of expected answer:
     *    <agisim> <msg> Position of Teacher1: ( -500,5,0 ) </msg> </agisim>
     * However, depending on timing, a different response may come from agisim server.
     * So, caller must check for the right result format and, if not as expected,
     * call the right method to process a possible sensation message.
     */
    std::string getPos(std::string objName);
    /**
     * Sets the position of the object with the given name to the given coordinates.
     * Returns the next raw response after sending this command:
     * Example of expected answer:
     *    <agisim> <msg> ok </msg> </agisim>
     * However, depending on timing, a different response may come from agisim server.
     * So, caller must check for the right result format and, if not as expected,
     * call the right method to process a possible sensation message.
     */
    std::string setPos(std::string objName, float x, float y, float z);
    /**
     * Gets the rotation of the object with the given name
     * Returns the next raw response after sending this command:
     * Example of expected answer:
     *    <agisim> <msg> Rotation of Teacher1: 0.5 </msg> </agisim>
     * However, depending on timing, a different response may come from agisim server.
     * So, caller must check for the right result format and, if not as expected,
     * call the right method to process a possible sensation message.
     */
    std::string getRot(std::string objName);
    /**
     * Sets the rotation of the object with the given name to the given coordinates.
     * Returns the next raw response after sending this command:
     * Example of expected answer:
     *    <agisim> <msg> ok </msg> </agisim>
     * However, depending on timing, a different response may come from agisim server.
     * So, caller must check for the right result format and, if not as expected,
     * call the right method to process a possible sensation message.
     */
    std::string setRot(std::string objName, float x, float y, float z);

    // AGISIM AGENT COMMANDS (all of them return a NM-XML string if successfull. Otherwise return an error message not in XML format)

    // Agent body movements

    /**
     * Turns the agent to the left by the given angle in radians.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long turnLeft(float value);
    /**
     * Turns the agent to the right by the given angle in radians.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long turnRight(float value);
    /**
     * Moves the agent forward by the given distance.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long moveForward(float value);
    /**
     * Moves the agent backward by the given distance.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long moveBackward(float value);
    /**
     * Moves the agent left by the given distance.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long strafeLeft(float value);
    /**
     * Moves the agent right by the given distance.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long strafeRight(float value);
    /**
     * Makes the agent walks toward to the object with the given name.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long walkTowards(std::string objName);
    /**
     * Makes the agent walks toward to given coordinates.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long walkTowards(float x, float z, float max_distance = -1.0f);
    /**
     * Makes the agent nudge the object with the given name to the given destination.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long nudgeTo(std::string objName, float x, float z);
    /**
     * Makes the agent turns to the object with the given name.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long turnTo(std::string objName);

    // Agent eye movements

    /**
     * Moves the agent's eyes up by the given angle in radians.
     * @return true if the command message was sent successfully. False, otherwise.
     */
    bool eyeUp(float value);
    /**
     * Moves the agent's eyes down by the given angle in radians.
     * @return true if the command message was sent successfully. False, otherwise.
     */
    bool eyeDown(float value);
    /**
     * Moves the agent's eyes left by the given angle in radians.
     * @return true if the command message was sent successfully. False, otherwise.
     */
    bool eyeLeft(float value);
    /**
     * Moves the agent's eyes right by the given angle in radians.
     * @return true if the command message was sent successfully. False, otherwise.
     */
    bool eyeRight(float value);

    // Agent misc

    /**
     * Makes the agent tries to eat the object with the given name that is near the agent.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long eat(std::string objName, float quantity = -1);

    /**
     * Makes the agent tries to drink the object with the given name that is near the agent.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long drink(std::string objName, float quantity = -1);

    /**
     * Makes the agent tries to lift the object with the given name.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long lift(std::string objName);
    /**
     * Makes the agent tries to drop the lifted object .
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long drop();
    /**
     * Makes the agent tries to throw the lifted object.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long throws(float distance);
    /**
     * Makes the agent tries to throw the lifted object to the given coordinate.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long throwsAt(float x, float z);
    /**
     * Makes the agent kick low.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long kickLow();
    /**
     * Makes the agent kick high.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long kickHigh();
    /**
     * Makes the agent kick high.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long smile();
    /**
     * Makes the agent kick high.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long frown();

    // Agent communications

    /**
     * Sends a noise.make action command.
     */
    bool noiseMake();
    /**
     * Sends a broadcast message to the connected Agisim's world.
     */
    unsigned long message(const char* msg);

    // specific goto commands -- consider using walkTorwards() method instead

    /**
     * Goes to the given target object using the navigation algorithm to avoid/contourn
     * obstacles.
    * @return an actionTicket so that SimProxy's user can receive notification about the status of this action
     *         If it fails to send the command to the server, returns ULONG_MAX as a flag for indicating this failure.
     */
    unsigned long gotoTarget(const char* target);

    // generic send method -- used for test purposes only. Do not use it in normal usage.
    std::string send(const std::string &str, bool isAdminCommand, bool waitResponse = true);

    // This method was made public for testing purposes (see SimProxyUTest.cxxtest)
    std::string processAgiSimMessage(const std::string& agiSimMessage, bool isAdminCommand);

private:
    static bool using_single_component_mode;
    static unsigned long numberOfInstances;

    struct eqstr {
        bool operator()(char *s1, char *s2) const;
    };
    typedef std::map<int, std::deque<unsigned long> > EventId2AtomRepMap;

    std::string host;
    unsigned short port;
    SocketHandler *sh;
    SimClientSocket *cc;
    bool echoing;
    AsynchronousPerceptionAndStatusHandler* perceptionAndStatusHandler;
    std::string agentName;
    unsigned int numberOfPendingActions;

    // Next action ticket to be used when the user calls one of the action methods.
    unsigned long nextActionTicket;

    static XERCES_CPP_NAMESPACE::DOMImplementation* domImplementation;
    void InitXMLPlatform();
    void TerminateXMLPlatform();

    EventId2AtomRepMap* eventMap;

    bool timeout(timeval beginTime, timeval currentTime, long waitTimeout);
    bool sendActionCommand(const char* cmdName, float value);
    bool sendActionCommand(const char* cmdName, float value1, float value2);
    bool sendActionCommand(const char* cmdName, float value1, float value2, float value3);
    bool sendActionCommand(const char* cmdName);
    bool sendActionCommand(const char* cmdName, const char* str);
    bool sendActionCommand(const char* cmdName, const char* str, float value);
    bool sendActionCommand(const char* cmdName, const char* str, float value1, float value2);
    XERCES_CPP_NAMESPACE::DOMDocument* parseXML(const std::string&);
    void processPerceptionAndStatus(XERCES_CPP_NAMESPACE::DOMDocument*);
    bool isAdminResponse(XERCES_CPP_NAMESPACE::DOMDocument* rawResponseDoc);
    bool processAdminResponse(XERCES_CPP_NAMESPACE::DOMDocument* rawResponseDoc, std::string& message);
    std::string getString(XERCES_CPP_NAMESPACE::DOMDocument*);
    static std::string convert(const XMLCh * xString);
    void splitValues(char*text, std::vector<char*>& values);
    void addAtomType(std::vector<char*>& atomTypes, const char* newAtomType);

    /**
     * Enqueue an action as InProgress and returns a ticket to be sent to the SimProxy's user so that it can identify the action
     * when a status notification about that action is given
     */
    unsigned long enqueueActionInProgress(int actionType);

    /**
     * Gets and removes the ticket for the first in progress action for the given actionType.
    * Return the ticket of the pending/in progress action, or 0, if there is no pending action associated to the given action type.
     */
    unsigned long dequeueActionInProgress(int actionType);

    /**
     * Get the ticket for the first in progress action for the given actionType
    * Return the ticket of the pending/in progress action, or 0, if there is no pending action associated to the given action type.
     */
    unsigned long getActionInProgress(int actionType);

    /**
     * Process a SelfData sensation message
     */
    void processSelfDataElements(XERCES_CPP_NAMESPACE::DOMElement* sensationElem);

    /**
     * Process a MapInfo sensation message
     */
    void processMapInfoElements(XERCES_CPP_NAMESPACE::DOMElement* sensationElem);
};

#endif //__SIM_PROXY_H__
