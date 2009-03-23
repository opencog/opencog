/***************************************************************************
 *  CS agent class.
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *																			
 *	19.01.06	FP	formatting  
 ****************************************************************************/

#ifndef CSAGENT_H
#define CSAGENT_H

#include <iostream>
#include "demon.h"
#include "action_vocabulary.h"
#if CRYSTAL
 #include "CSanimation.h"
#endif

class SimServer;
class Sensor;
class meshData;

//------------------------------------------------------------------------------------------------------------
/** @class CSAgent
	\brief The agent implementation on the server.*/
//------------------------------------------------------------------------------------------------------------
class CSAgent : public Demon {
  	float 	   rotx,roty;
	csVector3  startpos;
	double	   agent_eye_phi,agent_eye_theta;
	Obj3D 	   *hand;
	csVector3  liftedObjectPos;
	shared_ptr<meshData>  liftedObject;

	csRef<iMeshWrapper> CS_body;

public:  	
    bool walkTowardsActionInProgress; 
    bool nudgeToActionInProgress; 
    bool dropBeforeNudgeTo; 

    CSAgent (void* superobject, SimServer* _server, csRef<iMeshWrapper> _CS_body, csVector3 _startpos);    
	~CSAgent();

    virtual int 		  initialise (std::string nick);
	csRef<iMeshWrapper>   getCSBody  ()	const;

	void  GoTo(double x,double y,double z);
	bool  Lift(shared_ptr<meshData> object);
	bool  Drop();
	shared_ptr<meshData>  GetLifted() const;
	csVector3			  GetLiftedObjectPosition() const;
	
	Obj3D* getRepresentation();
	Obj3D* getHand();
	  
	/** Not really used. */
    virtual void disconnect();
	
	void onSensation(std::string m);

	/** Handle an action. Called typically from a socket command object. */
	virtual ParseResult action(std::vector<std::string>& parameters);
	virtual void onAction();
	
	/** Used by 'soft reset' */	
	void resetProprioceptives();
	void ResetSensors();
	
	friend class LocalServer;
	friend class SelfSensor;
};

#endif
