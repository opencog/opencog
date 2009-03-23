/**

Copyright Ari A. Heljakka / GenMind Ltd.
Not part of AGI-Sim.
Do not distribute.

*/

#include "simcommon.h"
#include "space.h"
//#include "simworld.h"
#include "CSagent.h"
#include "sensors.h"
#include "simserver.h"

#if CRYSTAL
 #include "CSops.h"
 #include "CSworld.h"
#endif


#ifdef WIN32
  #define M_PI_2 1.57079632679489661924
#endif
#define PI (M_PI_2*2)

/* NOTE: Dynamic movement is not supported, though there are some fragments
   of partial support, enabled depending on DYNAMIC_MOVEMENT preprocessor var. */

//--------------------------------------------------------------------------------------------------------------
struct EnergyListener : public Listener {
    virtual void OnUpdate(const void* new_energy)  {
        if (new_energy == NULL)  return;
        int  _energy = atoi((char*)new_energy);
        if (_energy < 0) {
             LOG("EnergyListener", 0, "AGENT DIED!");
        }
        else {
             LOG("EnergyListener", 1, ("Energy now " + toString(_energy)));
        }        
    }
};

//--------------------------------------------------------------------------------------------------------------
#if CRYSTAL
CSAgent::CSAgent (void* superobject, SimServer* _server, csRef<iMeshWrapper> _CS_body, csVector3 _startpos)
  : Demon(NULL), startpos(_startpos), agent_eye_phi(0), agent_eye_theta(0), animation_control(_CS_body)
#else
CSAgent::CSAgent (void* superobject, SimServer* _server, csRef<iMeshWrapper> _CS_body, csVector3 _startpos)
  : Demon(NULL), startpos(_startpos), agent_eye_phi(0), agent_eye_theta(0)
#endif
{
    Set (WENERGY, 0);
    SetPropertyListener (WENERGY, shared_ptr<EnergyListener> (new EnergyListener));

    CS_body         = _CS_body;
    body            = new Obj3D;
    hand            = new Obj3D;    
    liftedObjectPos = csVector3(0,0,0);
    
    resetProprioceptives();
    
    // Explicitly passing startpos is deprecated.
    csVector3  realStartPos = _CS_body->GetMovable()->GetPosition();
    startpos = realStartPos;

    walkTowardsActionInProgress = false;
    nudgeToActionInProgress = false;
    dropBeforeNudgeTo = false;
    
     LOG("CSAgent", 0, "Constructed.");
}

//--------------------------------------------------------------------------------------------------------------
void CSAgent::ResetSensors()
{
    Demon::ResetSensors();
    
    InsertSense("SENSETOUCH", shared_ptr<Sensor>(new TouchSensor(this)));
    InsertSense("SENSESELF", shared_ptr<Sensor>(new SelfSensor(this)));
}

//--------------------------------------------------------------------------------------------------------------
void CSAgent::resetProprioceptives()
{
    getRepresentation()->setPosition   (startpos.x,startpos.y, startpos.z);
#if CRYSTAL
    setView (CSWorld::Get().CreateView (startpos));
#endif
    body->setOrientation (0,0,0,0);
    
    Set (EYE_THETA, 0.0f);
    Set (EYE_PHI, 0.0f);    
    
    Set (WENERGY, StringConfig("INITIALENERGY"));
}

//--------------------------------------------------------------------------------------------------------------
CSAgent::~CSAgent()
{
     LOG("CSAgent", 1, "Destroying!!!");    
    // Demon superclass deletes the body.    
    delete hand;

     LOG("CSAgent", 3, "Destroyed.");    
    /*    LOG("CSAgent", 3, string("Body was shared by ")    + toString(CS_body.use_count()) +" objects. Now destroying locally.");    */
}

//--------------------------------------------------------------------------------------------------------------
int CSAgent::initialise(std::string nick)
{
    Demon::initialise(nick);
    InsertSense ("SENSETOUCH", shared_ptr<Sensor> (new TouchSensor(this)));
    InsertSense ("SENSESELF" , shared_ptr<Sensor> (new SelfSensor (this)));

    return 0;
}

//--------------------------------------------------------------------------------------------------------------

csRef<iMeshWrapper>  CSAgent::getCSBody   () const { return CS_body; }    
Obj3D*                 CSAgent::getRepresentation () { return body; }
Obj3D*                 CSAgent::getHand           () { return hand; }

//--------------------------------------------------------------------------------------------------------------      
void CSAgent::disconnect() { }    

//--------------------------------------------------------------------------------------------------------------
void CSAgent::onSensation(std::string m) {
}

//--------------------------------------------------------------------------------------------------------------
void CSAgent::onAction()
{
    UpdateView        ();
    ForceSensorUpdate ();
}


//--------------------------------------------------------------------------------------------------------------
ParseResult CSAgent::action(std::vector<std::string>& parameters)
{  
    std::string action = parameters[0];

    LOG("GenericAgent::action", 3, "Received action '" + action + "'");
    for (unsigned int i = 1; i < parameters.size(); i++) {
        LOG("GenericAgent::action", 3, "Received parameter: '" + parameters[i] + "'");
    }

    int  energy = _Int (Get("energy"));    

    if (energy <= 0 && IntConfig("DeathDisablesAgent"))    {
         LOG("Agent", 0, "Unable to move. Dead & disabled.");
        AddSensation( CustomSensation( "DEAD", 1, CUSTOM_SENSATION_AGENT_DEATH) );
        onAction();
        return PARSE_OK;
    }
    try {
    #ifdef DYNAMIC_MOVEMENT
        if ( action == "accel" || action == "turn" || action == "stop" ) {
            
            if (action == "accel") {
                boost::mutex::scoped_lock  lock(movement_mutex);
                acceleration = strtod(parameters[1].c_str(), NULL);
                 LOG("GenericAgent::action", 4, "Changing acceleration to " << acceleration);
            } else if (action == "turn") {
                boost::mutex::scoped_lock  lock(movement_mutex);
                turn_angle = strtod(parameters[1].c_str(), NULL);
                 LOG("GenericAgent::action", 4, "Changed to " << turn_angle << " radians to turn.");
            } else {
                boost::mutex::scoped_lock  lock(movement_mutex);
                stopping = true;
                 LOG("GenericAgent::action", 4, "Stopping (vel = 0)");
            }
            
            if (!movement_thread) {
                AgentMovementThread  mt (getSite(), this, &movement_thread_quit);
                movement_thread = new boost::thread(mt); // boost will make a copy of "mt"
                 LOG("GenericAgent::action", 2, "Movement thread created.");
            }
                    
        } else { LOG("GenericAgent::action", 2, "Unknown action type " << action); }
    #else    
        // Agent body movement commands:
        if (action == FORWARD || action == BACKWARD    || action == STRAFE_LEFT || action == STRAFE_RIGHT) {
          
          double x, y, z;
          getRepresentation()->getPosition( x, y, z );
          csVector3 start_pos(x, y, z);
          
          float  distance = atof(parameters[1].c_str());
            Demon::action(parameters);

          getRepresentation()->getPosition( x, y, z );
          csVector3 end_pos(x, y, z);
#if CRYSTAL
          std::string move_anim = StringConfig("AnimationMoveName");
          animation_control.ExecuteMoveAnimation(move_anim.c_str(), start_pos, end_pos, FloatConfig("AnimationMoveSpeed"));
#else          
          if (!nudgeToActionInProgress) {
          // Just simulates the action events
          long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
          if (!walkTowardsActionInProgress) {
              AddSensation( CustomSensation( "Leg related action started!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
          }
          if (walkTowardsActionInProgress) {
              AddSensation( CustomSensation( "walk.towards action done!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
              walkTowardsActionInProgress = false;
          } else {
              AddSensation( CustomSensation( "Leg related action done!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
          }
#else
          if (!walkTowardsActionInProgress) {
              AddSensation( CustomSensation( "Move animation started!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_STARTED) );
          }
          if (walkTowardsActionInProgress) {
              AddSensation( CustomSensation( "walk.towards action done!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_DONE) );
              walkTowardsActionInProgress = false;
          } else {
              AddSensation( CustomSensation( "Move animation done!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_DONE) );
          }
#endif
          } else if (walkTowardsActionInProgress) {
              walkTowardsActionInProgress = false;
          }
#endif
            energy -= (int) ( IntConfig("PricePerStep") * (distance / FloatConfig("MoveQuantum")) );
            Set (WENERGY, energy);            
          
            //getHand()->setPosition(ax + mx, ay + my, az + mz);

        }
        else if (action == TURN_LEFT || action == TURN_RIGHT) {
          double x, y, z, w, nw;
          getRepresentation()->getOrientation(x, y, z, w);
          Demon::action(parameters);

          if (IntConfig("EnableAnimation")) {
              if (action == TURN_RIGHT)
                nw = w + atof(parameters[1].c_str());
              else 
                nw = w - atof(parameters[1].c_str());
#if CRYSTAL
              animation_control.ExecuteRotateAnimation(StringConfig("AnimationTurnName").c_str(), 
                                   w, nw, 
                                   FloatConfig("AnimationTurnSpeed"));
#else
              if (!nudgeToActionInProgress) {
              // Just simulates the action events
              long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
              if (walkTowardsActionInProgress) {
                  AddSensation( CustomSensation( "walk.towards action started!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
              } else {
                 AddSensation( CustomSensation( "Leg related action started!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
              }
              if (!walkTowardsActionInProgress) {
                  AddSensation( CustomSensation( "Leg related action done!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
              }
#else
              if (walkTowardsActionInProgress) {
                  AddSensation( CustomSensation( "walk.towards action started!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_STARTED) );
              } else {
                  AddSensation( CustomSensation( "Turn animation started!", actionId, CUSTOM_SENSATION_TURN_ANIMATION_STARTED) );
              }
              if (!walkTowardsActionInProgress) {
                  AddSensation( CustomSensation( "Turn animation done!", actionId, CUSTOM_SENSATION_TURN_ANIMATION_DONE) );
              }
#endif
              }
#endif
          } else {
              getRepresentation()->getOrientation(x, y, z, nw);
#if CRYSTAL
              CS_body->GetMovable()->SetTransform(csMatrix3(x, y, z, nw));
                  CS_body->GetMovable()->SetTransform(csYRotMatrix3(-nw));
              CS_body->GetMovable()->UpdateMove();
#endif
          }

            float  dt = atof(parameters[1].c_str());
            energy -= (int) ( IntConfig("PricePerTurn") * (dt/FloatConfig("TurnQuantum")) );
            Set (WENERGY, energy);
        }
        else if (action == KICK_LOW) {
            if (IntConfig("EnableAnimation")) {
#if CRYSTAL                
                CSanimation* csa = CSWorld::Get().animated[CS_body->QueryObject()->GetName()];
                string animationName = StringConfig("AnimationKickLowName");
                float animationSpeed = FloatConfig("AnimationKickLowSpeed");
                csa->ExecuteGenericAnimation(animationName.c_str(), animationSpeed);
#else
                // Just simulates the action events
                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( "Leg related action started!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
                AddSensation( CustomSensation( "Leg related action done!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
#else
                AddSensation( CustomSensation( "Generic animation started!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED) );
                AddSensation( CustomSensation( "Generic animation done!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_DONE) );
#endif
#endif                
            } else {
                //TODO: Implement Non-cal3d robot kicking.
            }

            // TODO: Decrease energy when kicking as well.
            //float  dt = atof(parameters[1].c_str());
            //energy -= (int) ( IntConfig("PricePerTurn") * (dt/FloatConfig("TurnQuantum")) );
            //Set (WENERGY, energy);
        }
        else if (action == KICK_HIGH) {
            if (IntConfig("EnableAnimation")) {
#if CRYSTAL                
                CSanimation* csa = CSWorld::Get().animated[CS_body->QueryObject()->GetName()];
                string animationName = StringConfig("AnimationKickHighName");
                float animationSpeed = FloatConfig("AnimationKickHighSpeed");
                csa->ExecuteGenericAnimation(animationName.c_str(), animationSpeed);
#else
                // Just simulates the action events
                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( "Leg related action started!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
                AddSensation( CustomSensation( "Leg related action done!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
#else
                AddSensation( CustomSensation( "Generic animation started!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED) );
                AddSensation( CustomSensation( "Generic animation done!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_DONE) );
#endif
#endif                
            } else {
                //TODO: Implement Non-cal3d robot kicking.
            }

            // TODO: Decrease energy when kicking as well.
            //float  dt = atof(parameters[1].c_str());
            //energy -= (int) ( IntConfig("PricePerTurn") * (dt/FloatConfig("TurnQuantum")) );
            //Set (WENERGY, energy);
        }
        else if (action == WALK_TOWARDS || action == NUDGE_TO) {
            // TODO: validate number of parameters
            // Calculate the angle to turn to the target object
            csVector3  agentPos = CS_body->GetMovable()->GetFullPosition();
            double agentOrientation;
            {
                double x,y,z;
                this->getRepresentation()->getOrientation(x,y,z,agentOrientation);
            }
            // Gets the target/object position
            float targetX, targetZ;
            if (action == NUDGE_TO || parameters.size() == 2) {
               // next parameter is the object name
               iEngine*  engine = FROM_CS( iEngine );
               iMeshWrapper*   targetMesh   = engine->FindMeshObject( parameters[1].c_str() );
               if (!targetMesh)  {
                    LOG( "CSAgent", 0, std::string("ERROR: Could not find mesh object with name ") + parameters[1]);
                    long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                    AddSensation( CustomSensation( std::string("Could not find the object with the name ") + parameters[1], actionId, CUSTOM_SENSATION_LEG_ACTION_FAILED) ); 
#else
                    AddSensation( CustomSensation( std::string("Could not find the object with the name ") + parameters[1],  actionId, CUSTOM_SENSATION_MOVE_ANIMATION_FAILED) );
#endif
                    onAction();
                    return PARSE_OK;
                }
                //targetMesh->GetMovable()->UpdateMove();           
                csVector3  targetPos = targetMesh->GetMovable()->GetFullPosition();
                targetX = targetPos.x;
                targetZ = targetPos.z;
            } else {
                // next parameters are the target coordinates (x,z)
                targetX = atof(parameters[1].c_str());
                targetZ = atof(parameters[2].c_str());
            }
            printf("targetX = %f, targetZ = %f\n", targetX, targetZ);
            LOG("CSAgent", 2, "targetX = " + toString(targetX) + ", targetZ = " + toString(targetZ));

            // Compute angle and distance the Agent must use to move:
            float deltaX = targetX - agentPos.x;
            float deltaZ = targetZ - agentPos.z;
            float orientation = atan2f(deltaX,deltaZ);
            float angle = orientation - agentOrientation;
            float distance = sqrt(deltaX*deltaX + deltaZ*deltaZ);
            if (action == NUDGE_TO || parameters.size() == 2) { 
               // only when target object name provided
               float minimal_distance = FloatConfig("WalkTowardsMinimalDistance");
               printf("distance = %f, minimal distance from the target object = %f\n", distance, minimal_distance);
               LOG("CSAgent", 2, "distance = " + toString(distance) + ", minimal distance from the target object = " + toString(minimal_distance));
               distance -= minimal_distance;
            }
	    if (action == WALK_TOWARDS && parameters.size() > 3) {
                // Maximal distance specified
                float maxDistance = atof(parameters[3].c_str());
                if (distance > maxDistance) {
                   printf("Setting distance from %f to the specified max distance = %f\n", distance, maxDistance);
                   LOG("CSAgent", 2, "Setting distance from " + toString(distance) + " to the specified max distance = " + toString(maxDistance));
                   distance = maxDistance;
                }
            }

            SocketReportProvider*  reporter = new SocketReportProvider( socket );       
            char commandBuffer[500]; 

            printf("Calculated turn angle for %s action = %f\n", action.c_str(), angle);
            LOG("CSAgent", 2, "Calculated turn angle for " + action + " action = " + toString(angle));
	    if (fabs(angle) > 0.01) {
                // Turn command
                printf("\n**************** %s: Turning %f ****************\n", action.c_str(), angle);
                LOG("CSAgent", 2, "\n**************** " + action + ": Turning " + toString(angle) + " ****************\n" + toString(angle));
            
                shared_ptr<Command> turnCommand(new CommandAction(reporter));
                while (angle < 0) angle += (2*PI);
                if (angle <= PI) {
                    sprintf(commandBuffer, "turn.right  %f" , angle);
                } else {
                    angle = 2*PI - angle;
                    sprintf(commandBuffer, "turn.left  %f" , angle);
                }
                turnCommand->addArguments(commandBuffer);
                 LOG( "CSAgent",0,string( "Enqueueing action..." ) + commandBuffer );
                SimServer::Get()->EnqueueHighPriorityCommand( turnCommand ); //Parse later
                 LOG( "CSAgent",0,"Enque ok..." );
            }
            
            // Move command
            printf("\n**************** %s: Moving %f ****************\n", action.c_str(), distance);
            LOG("CSAgent", 2, "\n**************** " + action + ": Moving " + toString(distance) + " ****************\n");

            shared_ptr<Command> moveCommand(new CommandAction(reporter));
            sprintf(commandBuffer, "move.forward  %f" , distance);
            moveCommand->addArguments(commandBuffer);
             LOG( "CSAgent",0,string( "Enqueueing action..." ) + commandBuffer );
            SimServer::Get()->EnqueueHighPriorityCommand( moveCommand ); //Parse later
             LOG( "CSAgent",0,"Enque ok..." );


            if (action == WALK_TOWARDS) {
                if (nudgeToActionInProgress) {
                    // Second drop action inside nudge.to action
                    printf("\n**************** NUDGE_TO: Droping nudged object ****************\n");
                    LOG("CSAgent", 2, "\n**************** NUDGE_TO: Droping nudged object ****************\n");
                    shared_ptr<Command> dropCommand(new CommandAction(reporter));
                    sprintf(commandBuffer, "drop");
                    dropCommand->addArguments(commandBuffer);
                     LOG( "CSAgent",0,string( "Enqueueing action..." ) + commandBuffer );
                    SimServer::Get()->EnqueueHighPriorityCommand( dropCommand ); //Parse later
                     LOG( "CSAgent",0,"Enque ok..." );
                } else {
                    walkTowardsActionInProgress = true;
                }
            } else { // NUDGE_TO
                nudgeToActionInProgress = true;
                // This is a compounded action, implemented as follows: 
                // drop walkTowards(movableObj) grab(movableObj) walkTowards(targetPos)
		// The first walkTowards action is already done above. Do the remaining ones bellow... 

                // 1st parameter is the name of the object to be nudged
                const char* objectName = parameters[1].c_str();
		std::string remainingParams = parameters[2];
                if (parameters.size() > 3) {
                    remainingParams += " ";
		    remainingParams += parameters[3];
                }

                printf("object name = %s, remainingParams = %s\n", objectName, remainingParams.c_str());
                LOG("CSAgent", 2, std::string("objectName = ") + objectName + ", remainingParams = " + remainingParams);

                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( "Leg related action started!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
#else
                AddSensation( CustomSensation( "Move animation started!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_STARTED) );
#endif
                // First drop command, if needed 
                if (liftedObject.get()) {
                    dropBeforeNudgeTo = true;
                    printf("\n**************** NUDGE_TO: Droping ****************\n");
                    LOG("CSAgent", 2, "\n**************** NUDGE_TO: Droping ****************\n");
                    shared_ptr<Command> dropCommand(new CommandAction(reporter));
                    sprintf(commandBuffer, "drop");
                    dropCommand->addArguments(commandBuffer);
                     LOG( "CSAgent",0,string( "Enqueueing action..." ) + commandBuffer );
                    SimServer::Get()->EnqueueHighPriorityCommand( dropCommand ); //Parse later
                     LOG( "CSAgent",0,"Enque ok..." );
                }
        
                // grab action
                printf("\n**************** NUDGE_TO: Lifting %s ****************\n", objectName);
                LOG("CSAgent", 2, std::string("\n**************** NUDGE_TO: Lifting ")  + objectName + " ****************\n");
                shared_ptr<Command> liftCommand(new CommandAction(reporter));
                sprintf(commandBuffer, "lift %s" , objectName);
                liftCommand->addArguments(commandBuffer);
                 LOG( "CSAgent",0,string( "Enqueueing action..." ) + commandBuffer );
                SimServer::Get()->EnqueueHighPriorityCommand( liftCommand ); //Parse later
                 LOG( "CSAgent",0,"Enque ok..." );
            
                // Second walk.towards action
                printf("\n**************** NUDGE_TO: Walking towards (%s) ****************\n", remainingParams.c_str());
                LOG("CSAgent", 2, "\n**************** NUDGE_TO: Walking towards (" + remainingParams + ") ****************\n");
                shared_ptr<Command> walkTowardsCommand(new CommandAction(reporter));
                sprintf(commandBuffer, "walk.towards %s" , remainingParams.c_str());
                walkTowardsCommand->addArguments(commandBuffer);
                 LOG( "CSAgent",0,string( "Enqueueing action..." ) + commandBuffer );
                SimServer::Get()->EnqueueHighPriorityCommand( walkTowardsCommand ); //Parse later
                 LOG( "CSAgent",0,"Enque ok..." );
            }
        }
        else if (action == TURN_TO) {
            // Calculate the angle to turn to the target object
            csVector3  agentPos = CS_body->GetMovable()->GetFullPosition();
            double agentOrientation;
            {
                double x,y,z;
                this->getRepresentation()->getOrientation(x,y,z,agentOrientation);
            }
            // Gets the target position
            float targetX, targetZ;
            // parameter is the target object name
            iEngine*  engine = FROM_CS( iEngine );
            iMeshWrapper*   targetMesh   = engine->FindMeshObject( parameters[1].c_str() );
            if (!targetMesh)  {
                LOG( "CSAgent", 0, "ERROR: Could not find target mesh object of TURN_TO  action command ..." );
                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( "Could not find the object for turn.to action!", actionId, CUSTOM_SENSATION_LEG_ACTION_FAILED) );
#else
                AddSensation( CustomSensation( "Could not find the object for turn.to action!", actionId, CUSTOM_SENSATION_TURN_ANIMATION_FAILED) );
#endif
                onAction();
                return PARSE_OK;
            }
            //targetMesh->GetMovable()->UpdateMove();           
            csVector3  targetPos = targetMesh->GetMovable()->GetFullPosition();
            targetX = targetPos.x;
            targetZ = targetPos.z;
            printf("targetX = %f, targetZ = %f\n", targetX, targetZ);
            LOG("CSAgent", 2, "targetX = " + toString(targetX) + ", targetZ = " + toString(targetZ));

            // Compute angle the Agent must use to turn:
            float deltaX = targetX - agentPos.x;
            float deltaZ = targetZ - agentPos.z;
            float orientation = atan2f(deltaX,deltaZ);
            float angle = orientation - agentOrientation;

            SocketReportProvider*  reporter = new SocketReportProvider( socket );       
            char commandBuffer[500]; 

            printf("\n**************** TURN_TO: Turning %f ****************\n", angle);
            LOG("CSAgent", 2, "\n**************** TURN_TO: Turning " + toString(angle) + " ****************\n");
            
            if (fabs(angle) > 0.01f) {
                shared_ptr<Command> turnCommand(new CommandAction(reporter));
                while (angle < 0) angle += (2*PI);
                if (angle <= PI) {
                    sprintf(commandBuffer, "turn.right  %f" , angle);
                } else {
                    angle = 2*PI - angle;
                    sprintf(commandBuffer, "turn.left  %f" , angle);
                }
                turnCommand->addArguments(commandBuffer);
                 LOG( "CSAgent",0,string( "Enqueueing action..." ) + commandBuffer );
                SimServer::Get()->EnqueueHighPriorityCommand( turnCommand ); //Parse later
                 LOG( "CSAgent",0,"Enque ok..." );
            } else {
                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( "Agent is already turned to target object!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
                AddSensation( CustomSensation( "Agent is already turned to target object!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
#else
                AddSensation( CustomSensation( "Agent is already turned to target object!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_STARTED) );
                AddSensation( CustomSensation( "Agent is already turned to target object!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_DONE) );
#endif
            }
        }
        else if (action == HAND_FORWARD || action == HAND_BACKWARD || action == HAND_LEFT    || action == HAND_RIGHT   ||
                 action == HAND_UP        || action == HAND_DOWN       || action == HAND_FORWARD || action == HAND_FORWARD ||
                 action == HAND_GRASP   || action == HAND_UNGRASP)
        {    
            double  param = strtod (parameters[1].c_str(), NULL);
            
            double  x, y, z;
            double  nx, ny, nz, nphi;
            double  mx, my, mz;
            
            getRepresentation()->getOrientation (nx, ny, nz, nphi);
            getRepresentation()->getPosition    (mx, my, mz);
            
            getHand()->getPosition (x,y,z);
            x -= mx;  y -= my;  z -= mz;
            
            // TODO: limit extent of hand motion.
            if (action == HAND_FORWARD) {
                x += ( param * sin(nphi) );
                z += ( param * cos(nphi) );
            } 
            else if (action == HAND_BACKWARD) {
                x -= ( param *sin (nphi) );
                z -= ( param *cos (nphi) );    
            } 
            else if (action == HAND_UP) {
                y += param;    
            } 
            else if (action == HAND_DOWN) {
                y -= param;
            } 
            else if (action == HAND_LEFT) {
                x += ( param * sin (nphi - M_PI_2) );
                z += ( param * cos (nphi - M_PI_2) );
            } 
            else if (action == HAND_RIGHT) {
                x += ( param * sin(nphi + M_PI_2) );
                z += ( param * cos(nphi + M_PI_2) );
            } 
            else if (action == HAND_GRASP) {                
            } 
            else if (action == HAND_UNGRASP) {
            }
            
            getHand()->setPosition (mx + x,  my + y,  mz + z);        
        } 
        else if (action == EYE_UP || action == EYE_DOWN || action == EYE_LEFT || action == EYE_RIGHT) {
            float dt = atof(parameters[1].c_str());
            energy -= (int)(IntConfig("PricePerEyeMove") * (dt/FloatConfig("EyeMoveQuantum")));
            Set(WENERGY, energy);
            
            Demon::action(parameters);
        } 
        else if (action == NOISE) {
            
        } 
        else if (action == MESSAGE) {
            //std::string dest("all");
            Demon::action(parameters);
        }
        else if (action == SAY) {
             LOG("Agent", 0, "SAY: " + parameters[1]);
            Demon::action(parameters);
        }
        else if (action == GOTO) {
             LOG("Agent", 0, "GOTO: " + parameters[1]);
            Demon::action(parameters);
                // Just simulates the action events
                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
              AddSensation( CustomSensation( "Leg related action started!", actionId, CUSTOM_SENSATION_LEG_ACTION_STARTED) );
              AddSensation( CustomSensation( "Leg related action done!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
#else
              AddSensation( CustomSensation( "Move animation started!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_STARTED) );
              AddSensation( CustomSensation( "Move animation done!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_DONE) );
#endif
        }
        else if (action == EAT || action == DRINK)
        {
#if 1
             LOG("CommandAction", 0, ((action==EAT)?"Eat: ":"Drink: ") + parameters[1] + (parameters.size() > 2?(", "+parameters[2]):""));        
            string objName = parameters[1];
            iEngine* engine = FROM_CS(iEngine);
            iMeshWrapper* targetMesh = engine->FindMeshObject(objName.c_str());
            long actionId = getElapsedMillis();
            if (GetLifted().get()) {
                string errorMsg = "Can not eat or drink anything while holding an object: '";
                errorMsg += GetLifted()->wrapper->QueryObject()->GetName(); 
		errorMsg += "'!";
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_HEAD_ACTION_FAILED) );
#else
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_FAILED) );
#endif
		LOG("CSAgent", 0, errorMsg);
                onAction();
                return PARSE_OK;
            } 
            if (!targetMesh) {
                string errorMsg = "Could not find the object with name '";
                errorMsg += objName; 
		errorMsg += "'!";
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_HEAD_ACTION_FAILED) );
#else
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_FAILED) );
#endif
		LOG("CSAgent", 0, errorMsg);
                onAction();
                return PARSE_OK;
            } 
            WorldObjectProperty&  p = TheSimWorld.pmap[objName];
            int quality, intensity;
            p.taste.Get( WQUALITY, quality );
            p.taste.Get( WINTENSITY, intensity );
	    //printf("Got quality = %d, intensity = %d\n", quality, intensity);
	    if ((action == EAT && (quality != 2 || intensity != 1)) || 
	        (action == DRINK && (quality != 3 || intensity != 1))) {  
                string errorMsg = "Object '";
                errorMsg += objName; 
		errorMsg += "' is not ";
		errorMsg += (action == EAT)?"edible!":"drinkable!";
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_HEAD_ACTION_FAILED) );
#else
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_FAILED) );
#endif
		LOG("CSAgent", 0, errorMsg);
                onAction();
                return PARSE_OK;
	    }
            csVector3  agentPos = CS_body->GetMovable()->GetFullPosition();
            csVector3  targetPos = targetMesh->GetMovable()->GetFullPosition();
            float deltaX = targetPos.x - agentPos.x;
            float deltaZ = targetPos.z - agentPos.z;
            float distance = sqrt(deltaX*deltaX + deltaZ*deltaZ);

            csVector3 targetDim = targetMesh->dim;
            csVector3 agentDim = CS_body->dim;
            float targetRadius = sqrt(targetDim.x*targetDim.x + targetDim.z*targetDim.z)/2; 
            float agentRadius = sqrt(agentDim.x*agentDim.x + agentDim.z*agentDim.z)/2; 
            float eat_range = FloatConfig( "MaximalEatDistance" ) + targetRadius + agentRadius;
	    if (distance > eat_range) {
                string errorMsg = "Object '";
                errorMsg += objName; 
		errorMsg += "' is too far to be ";
		errorMsg += (action == EAT)?"eaten":"drunk";
		errorMsg += " by the agent!";
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_HEAD_ACTION_FAILED) );
#else
                AddSensation( CustomSensation( errorMsg, actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_FAILED) );
#endif
		LOG("CSAgent", 0, errorMsg + "( targetRadius = " + toString(targetRadius) + ", agentRadius = " + toString(agentRadius) + ", eat_range = " + toString(eat_range) + ", distance = " + toString(distance));
                onAction();
                return PARSE_OK;
            }

            float eatQuantum = FloatConfig( "EatQuantum" );
            if (parameters.size() > 2) {
               eatQuantum = atof(parameters[2].c_str());
	    }
	    // get and decrease the energy of the object and, if reached 0, remove it from the world.
            float objEnergy; 
            p.Get(WENERGY, objEnergy); 
	    if (eatQuantum > objEnergy) {
                eatQuantum = objEnergy;
            }
	    objEnergy -= eatQuantum;
            if (objEnergy <= 0) {
               LOG("CommandAction", 0, string((action==EAT)?"Eat: removing eaten":"Drink: removing drunk") + " object '" + objName + "' of type '" + targetMesh->material + "'"); 
                dynamic_cast<MapInfoSensor*> ( senses["SENSEMAPINFO"].get())->objectRemoved(objName, targetMesh->material);
                TheSimWorld.pmap.erase(objName);
		// If object is lifted by any agent, drop it before removing it.
                for (int i = 0; i < SimServer::Get()->AgentCount(); i++) {
                    shared_ptr<CSAgent> agent = SimServer::Get()->GetAgent(i);
                    shared_ptr<meshData>  liftedObject = agent->GetLifted();       
                    if (liftedObject && liftedObject->wrapper == targetMesh) {
                        agent->Drop();
                    }
		}
		engine->RemoveObject(targetMesh);
            } else {
                p.Set(WENERGY, objEnergy);
            }
	    // increase the energy of the agent
            float agentEnergy; 
            Get(WENERGY, agentEnergy);
            agentEnergy += eatQuantum;
            Set(WENERGY, agentEnergy); 

#ifdef USE_ACTUATOR_BASED_EVENTS
            AddSensation( CustomSensation( "Action started!", actionId, CUSTOM_SENSATION_HEAD_ACTION_STARTED) );
            AddSensation( CustomSensation( "Action done!", actionId, CUSTOM_SENSATION_HEAD_ACTION_DONE) );
#else
            AddSensation( CustomSensation( "Action started!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED) );
            AddSensation( CustomSensation( "Action done!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_DONE) );
#endif

#else // old code
            energy -= IntConfig ("PricePerEat");
            Set (WENERGY, energy);
             LOG("GenericAgent::action", 4, "Eat");        
            map< string, meshData >  current_edibles = static_cast<CST*>( CSproxy::Get() )->GetMovableVisibleObjects(this);
            
            // TODO: "Diagnostix sensation numbers should be documented somewhere!"
    
            if ( current_edibles.size() == 0 )  {
                string  msg = "Nothing to eat here! Do you have an object at the center of your FOV?";
                 LOG( "Server",3, msg);             
                AddSensation( CustomSensation( msg, 1 , 104) );
                onAction();
                return PARSE_OK;
            }
    
             LOG( "Server", 3, ("Number of potential food: " + toString(current_edibles.size())).c_str() );
            
            float     closest_dist = 9999999.0f;
            meshData  closest_e;
            string    closest_e_name;
    
            for (map< string, meshData >::iterator  ee = current_edibles.begin(); ee != current_edibles.end(); ee++)  {
                if ( ee->second.distance < closest_dist )   {
                    closest_e      = ee->second;
                    closest_e_name = ee->first;
                    closest_dist   = ee->second.distance;
                }
            }
    
            string  msg = ( "About to eat: " + closest_e_name);
             LOG( "Server", 2, msg);
                    
            float   eatrange = FloatConfig( "MaximalEatDistance" ) + FloatConfig( "AgentBodyRadius" ) + FloatConfig( "CameraRelativeY" );
    
            if ( closest_dist > eatrange )  {
                char  t[10000];
                 sprintf( t, "But too far away: %.2f / %.2f", closest_dist, FloatConfig( "MaximalEatDistance" ) );
                msg += t;
                
                 LOG( "Server", 2, t);
                AddSensation( CustomSensation( msg, (int)(closest_dist*1000),102 ) );
                //TheSocketManager.SendMsg(socket, msg);
    
                onAction();
                return PARSE_OK;
            }
                    
            WorldObjectProperty&  p = TheSimWorld.pmap[closest_e_name];
            int  e  = _Int( p.Get(WENERGY) );
            int  de =  min( IntConfig( "EatQuantum" ), abs(e) );
    
            int  old_smell_i = 0;
            p.smell.Get( WINTENSITY, old_smell_i );
            int  dsmell      = (int)(-old_smell_i * (float)de / e);
    
            char temps[100];
             sprintf( temps, "Eating %s:  +%d. Smell intensity %d => %d", closest_e_name.c_str(), de, old_smell_i, old_smell_i+dsmell);         
             LOG( "Server", 4, temps);
    
            p.smell.Set( WINTENSITY, old_smell_i + dsmell );
    
            if ( e < 0 )  de = -de;  //Poison
            
            p.Set(WENERGY, e - de);  //Nutrition drained.
            
            Set( WENERGY, _Int( Get(WENERGY) ) + de );
            
            if ( IntConfig( "SENSETASTE" ) )  {
                 LOG( "Server", 3, "Tasting..." );
                dynamic_cast<TasteSensor*> ( senses["SENSETASTE"].get())->OnTaste(p.taste );
            }
    
            if (!e) { 
                string msg = "Item contains no energy. Failed.";
                 LOG( "Server", 2, msg);
                AddSensation( CustomSensation(msg, 1, 103) );
            }
            else if ( de > 0 )  { 
                string msg = ( "Yum! Gained energy: "+ toString(de));
                 LOG( "Server", 2, msg);            
                AddSensation( CustomSensation(msg, 1, 101) );
                
                if ( _Int( p.Get(WENERGY) ) <= 0 )  p.taste = Taste(0,0);
            }
            else {
                string msg = ( "Auch! Lost energy: " + toString(de));
                 LOG( "Server", 2, msg);
    
                AddSensation( CustomSensation(msg, 1, 101) );
                
                if ( _Int( p.Get(WENERGY) ) <= 0 )  
                    p.taste = Taste(0,0);
            }
#endif
        }
        else if ( action == LIFT )  {
            if (parameters.size() > 1) {
                // Special handling of LIFT action command that specifies the target object name
                 LOG( "CommandAction", 0, "Lifting specific object: " + parameters[1]);
                iEngine* engine = FROM_CS(iEngine);
                iMeshWrapper* targetMesh = engine->FindMeshObject(parameters[1].c_str());
                if (targetMesh) {
                     long actionId = getElapsedMillis();
                     if (Lift( shared_ptr< meshData >( new meshData(targetMesh, 0) ) )) {
                         if (!nudgeToActionInProgress) {
                             // Just simulates the action events
#ifdef USE_ACTUATOR_BASED_EVENTS
                             AddSensation( CustomSensation( "Arm related action started!", actionId, CUSTOM_SENSATION_ARM_ACTION_STARTED) );
                             AddSensation( CustomSensation( "Arm related action done!", actionId, CUSTOM_SENSATION_ARM_ACTION_DONE) );
#else
                             AddSensation( CustomSensation( "SemiPose animation started!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_STARTED) );
                             AddSensation( CustomSensation( "SemiPose animation done!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_DONE) );
#endif
			 }
                         AddSensation( CustomSensation( parameters[1].c_str(), 1, CUSTOM_SENSATION_HOLDING_OBJECT) );
                     } else {
                          LOG("COmmandAction",0, "Lift failed. (Already holding object?)");
                         if (!nudgeToActionInProgress) {
#ifdef USE_ACTUATOR_BASED_EVENTS
                             AddSensation( CustomSensation( "Lift object failed!", actionId, CUSTOM_SENSATION_ARM_ACTION_FAILED) );
#else
                             AddSensation( CustomSensation( "Lift object failed!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_FAILED) );
#endif
                         }
                     }
                } else {
                    LOG( "CommandAction", 0, "Could not find any object named " + parameters[1] );
                    if (!nudgeToActionInProgress) {
                        long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                        AddSensation( CustomSensation( "Could not find the object for Lift action!", actionId, CUSTOM_SENSATION_ARM_ACTION_FAILED) );
#else
                        AddSensation( CustomSensation( "Could not find the object for Lift action!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_FAILED) );
#endif
                    }
                    onAction();
                    return PARSE_OK;
                }
            } else {
                LOG( "CommandAction", 0, "Trying to lift the object that is in front of agent ..." );
                map< string, meshData >  current_edibles = static_cast<CST*>( CSproxy::Get() )->GetMovableVisibleObjects(this);
                for (map< string, meshData >::iterator  e = current_edibles.begin(); e != current_edibles.end(); e++) {
                    LOG( "LocalServer", 0, "Lifting " + e->first );
                    iMeshWrapper* targetMesh = e->second.wrapper;
                    long actionId = getElapsedMillis();
                    if (Lift( shared_ptr< meshData >( new meshData(e->second) ))) {
                        AddSensation( CustomSensation( targetMesh->QueryObject()->GetName(), 1, CUSTOM_SENSATION_HOLDING_OBJECT) );
                        // Just simulates the action events
#ifdef USE_ACTUATOR_BASED_EVENTS
                        AddSensation( CustomSensation( "Arm related action started!", actionId, CUSTOM_SENSATION_ARM_ACTION_STARTED) );
                        AddSensation( CustomSensation( "Arm related action done!", actionId, CUSTOM_SENSATION_ARM_ACTION_DONE) );
#else
                        AddSensation( CustomSensation( "SemiPose animation started!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_STARTED) );
                        AddSensation( CustomSensation( "SemiPose animation done!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_DONE) );
#endif
                    } else {
#ifdef USE_ACTUATOR_BASED_EVENTS
                        AddSensation( CustomSensation( "Lift object failed!", actionId, CUSTOM_SENSATION_ARM_ACTION_FAILED) );
#else
                        AddSensation( CustomSensation( "Lift object failed!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_FAILED) );
#endif
                        LOG( "CommandAction", 0, "Lift object failed!" + parameters[1] );
                    }
                    break;
                }
            }
        }
        else if ( action == DROP ) {
             LOG( "LocalServer", 0, "Dropping... " );
            long actionId = getElapsedMillis();
            if (Drop()) {
                AddSensation( CustomSensation( "", 0, CUSTOM_SENSATION_HOLDING_OBJECT) );
                if (!nudgeToActionInProgress) {
                    // Just simulates the action events
#ifdef USE_ACTUATOR_BASED_EVENTS
                    AddSensation( CustomSensation( "Arm related action started!", actionId, CUSTOM_SENSATION_ARM_ACTION_STARTED) );
                    AddSensation( CustomSensation( "Arm related action done!", actionId, CUSTOM_SENSATION_ARM_ACTION_DONE) );
#else
                    AddSensation( CustomSensation( "SemiPose animation started!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_STARTED) );
                    AddSensation( CustomSensation( "SemiPose animation done!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_DONE) );
#endif
                } else {
                    if (dropBeforeNudgeTo) {
                        dropBeforeNudgeTo = false;
                    } else {
                        nudgeToActionInProgress = false;
#ifdef USE_ACTUATOR_BASED_EVENTS
                        AddSensation( CustomSensation( "Leg related action done!", actionId, CUSTOM_SENSATION_LEG_ACTION_DONE) );
#else
                        AddSensation( CustomSensation( "Move animation done!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_DONE) );
#endif
                    }	     
                }
            } else {
                if (!nudgeToActionInProgress) {
#ifdef USE_ACTUATOR_BASED_EVENTS
                    AddSensation( CustomSensation( "Drop action failed!", actionId, CUSTOM_SENSATION_ARM_ACTION_FAILED) );
#else
                    AddSensation( CustomSensation( "Drop action failed!", actionId, CUSTOM_SENSATION_SEMIPOSE_ANIMATION_FAILED) );
#endif
                } else {
                    nudgeToActionInProgress = false;
#ifdef USE_ACTUATOR_BASED_EVENTS
                    AddSensation( CustomSensation( "Leg related action failed!", actionId, CUSTOM_SENSATION_LEG_ACTION_FAILED) );
#else
                    AddSensation( CustomSensation( "Move animation failed!", actionId, CUSTOM_SENSATION_MOVE_ANIMATION_FAILED) );
#endif
                }
                LOG( "CommandAction", 0, "Drop action failed!");
            }
        }           
        else if (action == SMILE)
        {
            if (IntConfig("EnableAnimation"))
            {
#if CRYSTAL                
                CSanimation* csa = CSWorld::Get().animated[CS_body->QueryObject()->GetName()];
                csa->ExecuteGenericAnimation(StringConfig("AnimationSmileName").c_str(), FloatConfig("AnimationSmileSpeed"));
#else            
                // Just simulates the action events
                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( "Head related action started!", actionId, CUSTOM_SENSATION_HEAD_ACTION_STARTED) );
                AddSensation( CustomSensation( "Head related action done!", actionId, CUSTOM_SENSATION_HEAD_ACTION_DONE) );
#else
                AddSensation( CustomSensation( "Generic animation started!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED) );
                AddSensation( CustomSensation( "Generic animation done!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_DONE) );
#endif
#endif              
            }
        }
        else if (action == FROWN)
        {
            if (IntConfig("EnableAnimation"))
            {
#if CRYSTAL                
                CSanimation* csa = CSWorld::Get().animated[CS_body->QueryObject()->GetName()];
                csa->ExecuteGenericAnimation(StringConfig("AnimationFrownName").c_str(), FloatConfig("AnimationFrownSpeed"));
#else              
                // Just simulates the action events
                long actionId = getElapsedMillis();
#ifdef USE_ACTUATOR_BASED_EVENTS
                AddSensation( CustomSensation( "Head related action started!", actionId, CUSTOM_SENSATION_HEAD_ACTION_STARTED) );
                AddSensation( CustomSensation( "Head related action done!", actionId, CUSTOM_SENSATION_HEAD_ACTION_DONE) );
#else
                AddSensation( CustomSensation( "Generic animation started!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED) );
                AddSensation( CustomSensation( "Generic animation done!", actionId, CUSTOM_SENSATION_GENERIC_ANIMATION_DONE) );
#endif
#endif              
            }
        }
        else {
            LOG("GenericAgent::action", 0, "Unknown or unimplemented action type " << action);
        }
    #endif
    } catch(string s) { LOG("GenericAgent::action", 2, "Exception: "+s); }
    catch(...) { LOG("GenericAgent::action", 2, "Unknown exception"); }    
    
    onAction(); 
     LOG("GenericAgent::action", 3, "Action handling ok.");
    return PARSE_OK;
}

//--------------------------------------------------------------------------------------------------------------
csVector3 CSAgent::GetLiftedObjectPosition() const
{
    return liftedObjectPos;
}

//--------------------------------------------------------------------------------------------------------------
bool CSAgent::Lift(shared_ptr<meshData> object)
{
    if (liftedObject.get()) {
        if (object->wrapper != liftedObject->wrapper) {
            LOG("CSAgent", 1, string("Agent cannot lift object '") + object->wrapper->QueryObject()->GetName() + "' because the object '" + liftedObject->wrapper->QueryObject()->GetName() + "' is already lifted.");
            return false;    
        } else {
            LOG("CSAgent", 1, string("Object '") + object->wrapper->QueryObject()->GetName() + "' was already lifted.");
	    return true;
        }
    }

    float liftHeight = FloatConfig("LiftHeight");

    /// Find object's relative position.    
    double x, y, z;
    
    body->getPosition (x, y, z);
    csVector3  opos = object->wrapper->GetMovable()->GetPosition();    
    liftedObjectPos = csVector3 (opos.x - x,  (opos.y+liftHeight) - y,  opos.z - z);

    float dist = sqrt(pow(liftedObjectPos.x,2) + pow(liftedObjectPos.z,2)); // consider only distance in the plan xz

    csVector3 targetDim = object->wrapper->dim;
    csVector3 agentDim = CS_body->dim;
    float targetRadius = sqrt(targetDim.x*targetDim.x + targetDim.z*targetDim.z)/2; 
    float agentRadius = sqrt(agentDim.x*agentDim.x + agentDim.z*agentDim.z)/2; 
    float lift_range = FloatConfig( "MaximalLiftDistance" ) + targetRadius + agentRadius;
    if (dist > lift_range) {
        LOG("CSAgent", 1, string("Object ") + object->wrapper->QueryObject()->GetName() + " too far to be lifted by the agent (targetRadius = " + toString(targetRadius) + ", agentRadius = " + toString(agentRadius) + ", lift_range = " + toString(lift_range) + ", distance = " + toString(dist));
        return false;
    }

    liftedObject = object;    
    LocalServer::UpdatePosition( 0, liftHeight, 0, liftedObject->wrapper ); 
    
    return true;
}

//--------------------------------------------------------------------------------------------------------------
bool CSAgent::Drop()
{
    if (liftedObject.get())    {
         LOG("CSAgent", 2, string("Droping lifted object ") + liftedObject->wrapper->QueryObject()->GetName());
        LocalServer::UpdatePosition(0, -FloatConfig("LiftHeight"), 0, liftedObject->wrapper);
        liftedObject.reset();
        return true;
    } 

    LOG("CSAgent", 1, "No lifted object to drop...");
    //return false;
    return true; // Drop never fails...
}

//--------------------------------------------------------------------------------------------------------------
shared_ptr<meshData> CSAgent::GetLifted() const
{
    return liftedObject;
}

//--------------------------------------------------------------------------------------------------------------
void CSAgent::GoTo(double x, double y, double z)
{
}

#ifdef DYNAMIC_MOVEMENT

void AgentMovementThread::operator()()
{
#define ELAPSED_TIME(tvp, uvp)\
               ((tvp.tv_sec - uvp.tv_sec) * 1000000 + (tvp.tv_usec - uvp.tv_usec))
#define MAX_SPEED 2.0
               

// TODO: Dynamic movement is not set up to handle
// the orientation properly. orientation is a vector, with the angle to rotate around

    struct timeval tv;
    struct timeval accel_tv;
    struct timeval move_tv;
    struct timeval turn_tv;

#ifdef WIN32
   struct _timeb timebuffer;
   _ftime( &timebuffer );
    tv.tv_sec = timebuffer.time;
    tv.tv_usec = 1000*timebuffer.millitm;
#else
    gettimeofday(&tv,NULL);
#endif

    accel_tv = move_tv = turn_tv = tv;
    
     LOG("AgentMovementThread", 2, "Entered thread");
    while (!*movement_thread_quit) {
        {
            boost::mutex::scoped_lock lock(agent->movement_mutex);
            if (agent->stopping) {
                agent->velocity = 0.0;
                agent->acceleration = 0.0;
                agent->stopping = false;
                
                 LOG("AgentMovementThread", 2, "Stopped");
            }
            if ((agent->acceleration * agent->acceleration) > 0.0) {
#ifdef WIN32
                struct _timeb timebuffer;
                _ftime( &timebuffer );
                tv.tv_sec = timebuffer.time;
                tv.tv_usec = 1000*timebuffer.millitm;
#else
                gettimeofday(&tv,NULL);
#endif                
                agent->velocity += (ELAPSED_TIME(tv, accel_tv) / 100000.0) * agent->acceleration;
                
                // Limit speed of agent...
                if (agent->velocity > MAX_SPEED) agent->velocity = MAX_SPEED;
                if (agent->velocity < -MAX_SPEED) agent->velocity = -MAX_SPEED;
                
                 LOG("AgentMovementThread", 2, "New velocity " << agent->velocity);
#ifdef WIN32
                struct _timeb timebuffer;
                _ftime( &timebuffer );
                accel_tv.tv_sec = timebuffer.time;
                accel_tv.tv_usec = 1000*timebuffer.millitm;
#else
                gettimeofday(&accel_tv,NULL);
#endif                
            }            
            if ((agent->velocity*agent->velocity) > 0.0)
            {
#ifdef WIN32
   struct _timeb timebuffer;
   _ftime( &timebuffer );
    tv.tv_sec = timebuffer.time;
    tv.tv_usec = 1000*timebuffer.millitm;
#else
    gettimeofday(&tv,NULL);
#endif
                
                double distance = (ELAPSED_TIME(tv, move_tv) / 100000.0) * agent->velocity;
                
                double x, y, z;
                double nx, ny, nz, nphi;
                double length;
                
                agent->getRepresentation()->getOrientation(nx, ny, nz, nphi);
                agent->getRepresentation()->getPosition(x, y, z);
                length = sqrt((nx*nx) + (ny*ny) + (nz*nz) + (nphi*nphi));
    
                distance /= length;
                nx *= distance; ny *= distance; nz *= distance;
                nphi *= distance;
    
                
                agent->getRepresentation()->setPosition(x+nx, y+ny, z+nz);

                 LOG("AgentMovementThread", 2, "New position x " << nx+x << " y " << ny+y << " z " << nz+z);
#ifdef WIN32
   struct _timeb timebuffer;
   _ftime( &timebuffer );
    move_tv.tv_sec = timebuffer.time;
    move_tv.tv_usec = 1000*timebuffer.millitm;
#else
    gettimeofday(&move_tv,NULL);
#endif
            }
            if ((agent->turn_angle * agent->turn_angle) > 0.0)
            {
#ifdef WIN32
   struct _timeb timebuffer;
   _ftime( &timebuffer );
    tv.tv_sec = timebuffer.time;
    tv.tv_usec = 1000*timebuffer.millitm;
#else
    gettimeofday(&tv,NULL);
#endif
                 
                double speed = (ELAPSED_TIME(tv, turn_tv) / 100000.0) * (M_PI / 10.0);
                if (agent->turn_angle < 0.0) {
                    speed *= -1.0;
                    if (agent->turn_angle > speed) agent->turn_angle = 0.0;
                    else agent->turn_angle -= speed;
                } else {
                    if (agent->turn_angle < speed) agent->turn_angle = 0.0;
                    else agent->turn_angle -= speed;
                }
                
                double nx, ny, nz, nphi;
                double x, y, z;
                double angle;
                angle = speed;
                                
                agent->getRepresentation()->getOrientation(nx, ny, nz, nphi);
                x = (nx*cos(angle))-(nz*sin(angle)); //+ ny*cos(M_PI_2);
                y = ny; //(-nx*cos(M_PI_2)*cos(angle)) - (ny*sin(M_PI_2)*cos(angle)) + (nz*sin(angle));
                z = (nx*sin(angle)) + (nz*cos(angle)); //- (ny*sin(M_PI_2)*sin(angle));
                agent->getRepresentation()->setOrientation(x, y, z, nphi);
                 LOG("AgentMovementThread", 2, "New orientation x " << x << " y " << y << " z " << z << " phi " << nphi);
                
#ifdef WIN32
   struct _timeb timebuffer;
   _ftime( &timebuffer );
    turn_tv.tv_sec = timebuffer.time;
    turn_tv.tv_usec = 1000*timebuffer.millitm;
#else
    gettimeofday(&turn_tv,NULL);
#endif
            }
        }

        usleep(100000);
    }
}

#endif
