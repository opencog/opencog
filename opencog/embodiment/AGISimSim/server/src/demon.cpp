/*
 * opencog/embodiment/AGISimSim/server/src/demon.cpp
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
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

/**

Copyright Ari A. Heljakka / GenMind Ltd.
Not part of AGI-Sim.
Do not distribute.

*/

/***************************************************************************
 *  Demon class.
 *
 *  Project: Novamente
 *
 ****************************************************************************/

#ifdef WIN32
	#define M_PI_2 1.57079632679489661924
#endif

#include "simcommon.h"
#include <math.h>
#include "property.h"
#include "space.h"
#include "sensors.h"
#include "demon.h"
#include "action_vocabulary.h"

#if CRYSTAL
 #include "CSserver.h"
 #include "CSworld.h"
#endif

using boost::shared_ptr;
using std::string;

/* Guilty as charged. */
extern void hacklog(string s, ServerSocket *socket = NULL);


//-----------------------------------------------------------------------------------------------------------------------
Demon::Demon( Obj3D*  _body )
			: socket( NULL ), body( _body ) 
{
}

//-----------------------------------------------------------------------------------------------------------------------
Demon::Demon( csVector3  _startpos )
		    : port(0), startpos( _startpos ) 
{
	body = new Obj3D;
	body->setPosition   ( 0, 0 , 0 );
	body->radius        = 0;
	body->setOrientation( 0, 0, 0, 0 );

	// The following may be reset later, if this is an agent.
	
	getRepresentation()->setPosition ( startpos.x, startpos.y, startpos.z);
#if CRYSTAL
    if (IntConfig("ShowAgentView")) {
    	setView( CSWorld::Get().CreateView( startpos ) );	
        //printf("************** Created view for Demon *********\n\n\n\n\n\n");
    } else {
        csVector3 conf_pos(FloatConfig("TopViewCamX"),
               FloatConfig("TopViewCamY"),
               FloatConfig("TopViewCamZ"));
        csXRotMatrix3 conf_rot_x(FloatConfig("TopViewRotX"));
        csYRotMatrix3 conf_rot_y(FloatConfig("TopViewRotY"));
        csZRotMatrix3 conf_rot_z(FloatConfig("TopViewRotZ"));
        // HACKY: Creates a TopView for this demon...
        csRef<iView> top_view = csPtr<iView>(new csView (FROM_CS(iEngine), 
                        FROM_CS(iGraphics3D)));
        iGraphics2D* g2d = FROM_CS(iGraphics3D)->GetDriver2D ();
        
        float size = FloatConfig("TopViewWindowPercent");
        float width = (size/100.0)*g2d->GetWidth();
        float height = (size/100.0)*g2d->GetHeight();
        top_view->SetRectangle (0, (int)(g2d->GetHeight()-height), 
                    (int) width, (int)height);
        csRef<iCamera> cam = FROM_CS(iEngine)->CreateCamera();   
        top_view->SetCamera(cam);
        cam->SetSector(FROM_CS(iEngine)->FindSector("room"));
        // You can sneak-peek in collision detection world...
        //cam->SetSector(FROM_CS(iEngine)->FindSector("collider_sector"));
        cam->SetPerspectiveCenter(width/2.0, g2d->GetHeight()-height/2.0);
        cam->GetTransform().Identity();
        cam->GetTransform().SetOrigin(conf_pos);
        cam->GetTransform().SetT2O(conf_rot_x*conf_rot_y*conf_rot_z);
        
        csVector3 point(0, 0, 1);
//        std::cout << "XXXX Before: " << point << std::endl;
        point = cam->GetTransform().Other2This(point);
//        std::cout << "XXXX after : " << point << std::endl;
        
        setView(top_view);   
    }
#endif
	
	Set(EYE_THETA, 0.0f);
	Set(EYE_PHI  , 0.0f);
}

//-----------------------------------------------------------------------------------------------------------------------
void  Demon::SetPort(int _port) {
	 LOG( "Demon",0, "New port: " + toString( _port ) );
	port = _port;
}

//-----------------------------------------------------------------------------------------------------------------------
int  Demon::GetPort() const
{
	return port;
}

//-----------------------------------------------------------------------------------------------------------------------
string  Demon::GetName() const
{
    return nick;
}

//-----------------------------------------------------------------------------------------------------------------------
std::map< std::string, shared_ptr<Sensor> >  Demon::GetSenses() const
{
	return senses;
}

//-----------------------------------------------------------------------------------------------------------------------
Demon::~Demon()
{
	 LOG("Demon", 3, "Destroying.");
	delete body;

	for (std::map< std::string, shared_ptr<Sensor> > ::iterator  u = senses.begin(); u!= senses.end();	u++)  {
		 LOG("Demon", 3, u->first + " has " + toString( u->second.use_count() ) + " referrers.");
		 // TODO: MemoryLeak?
		//delete u->second;
	}

	//	(map<string, Sensor*>*)(&senses)->clear(); //Paranoia and hack. 	
	 LOG("Demon", 3, "Destroyed.");	
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::RemoveSense( string __sname )
{
	if ( STLhas(senses, toupper(__sname)) )  {
		 LOG( __sname, 3, "Switching OFF..." );
		senses.erase( toupper(__sname) );
	}
	else  {
		LOG( __sname, 3, "Tried to switch off, but was off already." );
	}
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::InsertSense(string __sname, shared_ptr< Sensor > __sobject)
{
	if ( IntConfig(__sname) )	{
		 LOG( __sname, 3, "Switching on..." );
		senses[toupper(__sname)] = __sobject;
	}
	else  {
		 LOG( __sname, 3, "Switching OFF due to configuration..." );
	}
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::SetSocket( ServerSocket*  _socket )
{
	socket = _socket;
}

//-----------------------------------------------------------------------------------------------------------------------
ServerSocket*  Demon::GetSocket() const
{
	return socket;
}

//void Demon::Create()
//-----------------------------------------------------------------------------------------------------------------------
int Demon::initialise(std::string _nick)
{
	nick = _nick;	
	ResetSensors ();
	
	return 0;
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::ResetVisionSensors()
{
#if CRYSTAL
	InsertSense( "SENSEOBJECTVISION", shared_ptr< Sensor >( new ObjectVisionSensor( this, FROM_CS(iGraphics3D)->GetDriver2D(), getView(), IntConfig("ObjectVisionResolution") )));
	InsertSense( "SENSEPIXELVISION" , shared_ptr< Sensor >( new VisionSensor      ( this, FROM_CS(iGraphics3D)->GetDriver2D(), getView(), IntConfig("PixelVisionResolution")  )));	
#endif
	InsertSense( "SENSEPSEUDOVISION", shared_ptr< Sensor >( new PseudoObjectVisionSensor( this) ));
	InsertSense( "SENSEMAPINFO", shared_ptr< Sensor >( new MapInfoSensor( this) ));
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::ResetSensors()
{		
	ResetVisionSensors();
	InsertSense( "SENSEAURAL", shared_ptr< Sensor >( new AudioSensor(this) ));
	InsertSense( "SENSETASTE", shared_ptr< Sensor >( new TasteSensor(this) ));
	InsertSense( "SENSESMELL", shared_ptr< Sensor >( new SmellSensor(this) ));
    InsertSense( "SENSESELF",  shared_ptr< Sensor >( new SelfSensor (this) ));    
}

//-----------------------------------------------------------------------------------------------------------------------
Obj3D* Demon::getRepresentation() const
{
	return body;
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::UnloadSensationQueue()
{
	try  {		
	char   fname[] = "test.log";
	FILE*  fout    = NULL;

	if ( IntConfig("LogSenseData") )	{
		fout = fopen( fname, "rt+" );
		if ( !fout )  fout = fopen( fname, "wt" );
	
		fseek( fout, 0L, SEEK_END );
	}
	
	long    totalsize = 0;	
	string  msg       = "<sensation>";

	while( !sensationQueue.empty() )  {
		string  s = sensationQueue.front();
#ifdef WIN32
		totalsize += (long)s.size();
#else
		totalsize += s.size();
#endif
			
		if ( IntConfig("LogSenseData") )  fputs( s.c_str(), fout );
		
		sensationQueue.pop();

		msg += s;
	}
	msg += "</sensation>";
	
	hacklog(msg, socket);
	
	if ( totalsize > 0 )  {
		char  t[500];
		 sprintf(t, "Sensation Queue unloaded to socket & test.log: %.1f kb.\n", totalsize / 1024.0f);
 		  LOG( "Demon",3, t );
	}

	if ( IntConfig("LogSenseData") )  fclose(fout);

	} catch(...) { csPrintf("Exception in UnloadSensationQueue"); }
}

#if CRYSTAL

//-----------------------------------------------------------------------------------------------------------------------
csRef< iView >  Demon::getView() const
{
	return view;
}

//-----------------------------------------------------------------------------------------------------------------------
void  Demon::setView( csRef< iView >  _view )
{
	view = _view;	
	// Must create new sensors that use the new view!	
	ResetVisionSensors();
}

#endif

//-----------------------------------------------------------------------------------------------------------------------
void Demon::ForceSensorUpdate()
{
	 LOG("Demon",3,"Forcing sensor update...");
	
	for (std::map< std::string, shared_ptr<Sensor> >:: iterator  i = senses.begin(); i != senses.end(); i++)
		i->second->ForceUpdate();
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::ResendSensorData( string sensorName )
{
	  LOG("Demon",3,"Forcing sensor update...");
	
	 if ( sensorName == "all" )  {
		for (std::map< std::string, shared_ptr<Sensor> > ::iterator  i = senses.begin(); i != senses.end(); i++)  {
			i->second->ClearCache ();
			i->second->ForceUpdate();
		}
	 }
	 else  {
		try
		{
			senses[toupper(sensorName)]->ClearCache();
			senses[toupper(sensorName)]->ForceUpdate();
		} catch(string s) { LOG("demon",1,s); }
		catch(...) { LOG("demon",1,"Sensor not available."); }
	 }
}

#if CRYSTAL

//-----------------------------------------------------------------------------------------------------------------------
void Demon::UpdateView() //csRef<iView> view)
{
	 LOG("CSbox",4, "Retrieving camera...");
	iCamera*     c = view->GetCamera();	
	simplecoord  sc;
	 
	 LOG("CSbox",4, "Retrieving body pos...");	  
	body->getPosition(sc.x, sc.y, sc.z);

    csVector3  cam_pos(	sc.x+FloatConfig( "CameraRelativeX" ), sc.y+FloatConfig( "CameraRelativeY" ), sc.z+FloatConfig( "CameraRelativeZ" ) );

	//      c->GetTransform().SetOrigin(cam_pos);	
	csOrthoTransform  ot1 = c->GetTransform();
	ot1.SetOrigin  ( cam_pos );
	c->SetTransform( ot1 );
	  	      
    // Get Orientation sorted out	
	 LOG( "CSbox",4, "Retrieving body orientation..." );

    double  ox, oy, oz, ophi;
    body->getOrientation(ox, oy, oz, ophi);
	// Also need to add glance angle
	float  eye_phi = 0.0,  eye_theta = 0.0;

	try {
		 LOG("CSbox",4, "Retrieving agent eye...");		  	
		Get( EYE_THETA, eye_theta );
		Get( EYE_PHI  , eye_phi   );
	} catch(...) {}
    
    // New way
	float rotY = ophi + eye_phi;
	float rotX = eye_theta;

	csMatrix3  rot = csXRotMatrix3( rotX ) * csYRotMatrix3( rotY );
	csOrthoTransform  ot( rot, c->GetTransform().GetOrigin() );
	c->SetTransform(ot);      
      
	 LOG("CSbox",4, "Transforms ok.");  
}
#else
//-----------------------------------------------------------------------------------------------------------------------
void Demon::UpdateView()
{
/*	 LOG("CSbox",4, "Retrieving camera...");
	simplecoord  sc;
	 
	 LOG("CSbox",4, "Retrieving body pos...");	  
	body->getPosition(sc.x, sc.y, sc.z);

    csVector3  cam_pos(	sc.x+FloatConfig( "CameraRelativeX" ), sc.y+FloatConfig( "CameraRelativeY" ), sc.z+FloatConfig( "CameraRelativeZ" ) );

    // Get Orientation sorted out	
	 LOG( "CSbox",4, "Retrieving body orientation..." );

    double  ox, oy, oz, ophi;
    body->getOrientation(ox, oy, oz, ophi);
	// Also need to add glance angle
	float  eye_phi = 0.0,  eye_theta = 0.0;

	try {
		 LOG("CSbox",4, "Retrieving agent eye...");		  	
		Get( EYE_THETA, eye_theta );
		Get( EYE_PHI  , eye_phi   );
	} catch(...) { LOG("CSbox",0, "EXCEPTION WHEN Retrieving agent eye."); }
    
    // New way
	float rotY = ophi + eye_phi;
	float rotX = eye_theta;

      
	 LOG("CSbox",4, "Transforms ok.");  
*/
}
#endif


//-----------------------------------------------------------------------------------------------------------------------
ParseResult Demon::action(std::vector<std::string>&  parameters)
{  
    std::string action = parameters[0];
     LOG("Demon::action", 1, "Received action " + action);

	try	 {
		// body movement commands:
		if (action == FORWARD     || action == BACKWARD	    || 
			action == TURN_LEFT   || action == TURN_RIGHT	|| 
			action == STRAFE_LEFT || action == STRAFE_RIGHT)
		{
			double  param = strtod( parameters[1].c_str(), NULL );
	                
			double  ax, ay, az;
			double  nx, ny, nz, nphi;
			double  handLength;
			
			double  mx=0, my=0, mz=0; //FP variables were not initialized.
			//getHand()->getPosition(mx, my, mz);
	        
			getRepresentation()->getOrientation( nx, ny, nz, nphi );
			getRepresentation()->getPosition   ( ax, ay, az );
	        
			// Hand from world coordinates to agent coordinates must be changed when VOS handles hierarchical transforms		
			mx -= ax;  my -= ay;  mz -= az;
			handLength = sqrt( (mx*mx) + (mz*mz) + (my*my) );

			// TODO on turn: hand recentres to middle. 
			if ( action == FORWARD )	{
				ax += ( param * sin(nphi) );
				az += ( param * cos(nphi) );
				 LOG( "Demon::action", 4, "Forward " << param );
			}
			else if ( action == BACKWARD ) {
				ax -= ( param * sin(nphi) );
				az -= ( param * cos(nphi) );
				 LOG( "Demon::action", 4, "Backward " << param );			
			}
			else if ( action == TURN_LEFT ) {
				nphi -= param;
				mx    = handLength * sin(nphi);
				mz    = handLength * cos(nphi);
				 LOG( "Demon::action", 4, "Turning left " << param << " radians." );
			}
			else if ( action == TURN_RIGHT ) {
				nphi += param;
				mx    = handLength * sin(nphi);
				mz    = handLength * cos(nphi);
				 LOG( "Demon::action", 4, "Turning right " << param << " radians." );
			}
			else if ( action == STRAFE_LEFT )
			{
				ax += ( param * sin( nphi - M_PI_2 ) );
				az += ( param * cos( nphi - M_PI_2 ) );
				 LOG( "Demon::action", 4, "Strafing left " << param );			
			}
			else if (action == STRAFE_RIGHT)
			{
				ax += ( param * sin (nphi + M_PI_2 ) );
				az += ( param * cos (nphi + M_PI_2 ) );
				 LOG("Demon::action", 4, "Strafing right " << param);
			}
			
			if ( action == FORWARD     || action == BACKWARD	|| 
				 action == STRAFE_LEFT || action == STRAFE_RIGHT )
			{
				 LOG("Demon::action", 3, "Resetting position to (mm): " + toString(ax) + ","+ toString(ay) + ","+ toString(az) );
				getRepresentation()->setPosition( ax, ay, az );
				 LOG("Demon::action", 3, "New position ok.");
			}
			
			const double  pi = 3.14159265;
			
			if ( nphi > 2*pi ) 	nphi -= 2*pi;
			if ( nphi < 0    ) 	nphi += 2*pi;
			
			getRepresentation()->setOrientation( nx, ny, nz, nphi );

			iMeshWrapper*  targetMesh = iEngine::Instance().FindMeshObject(nick.c_str());
			if ( targetMesh != NULL ) {
				csVector3 rot = targetMesh->GetMovable()->GetRotation();
				rot.y = nphi; // For now, phi == y rotation
				targetMesh->GetMovable()->SetRotation(rot);
			} 
					
			//	getHand()->setPosition(ax + mx, ay + my, az + mz);
	                
		} else if ( action == EYE_UP   || action == EYE_DOWN ||
					action == EYE_LEFT || action == EYE_RIGHT) 
		{
			float  eye_theta,  eye_phi;
			//getVisualPosition()->read(eye_theta, eye_phi);
			Get( EYE_THETA, eye_theta);
			Get( EYE_PHI  , eye_phi  );
			
			float  param = strtod( parameters[1].c_str(), NULL );
					   
			if ( action == EYE_UP )          { eye_theta += param; } 
			else if ( action == EYE_DOWN  )  { eye_theta -= param; } 
			else if ( action == EYE_LEFT  )  { eye_phi   -= param; } 
			else if ( action == EYE_RIGHT )  { eye_phi   += param; } 
			
			if ( eye_theta > FloatConfig( "EyeThetaMax" ) )  {
				eye_theta = FloatConfig( "EyeThetaMax" );
				 LOG( "Demon",2,"Eye range limit met." );
			}
			else if ( eye_theta < FloatConfig( "EyeThetaMin" ) ) {
				eye_theta = FloatConfig( "EyeThetaMin" );
				 LOG( "Demon",2,"Eye range limit met." );
			}
			if ( eye_phi > FloatConfig( "EyePhiMax" ) )  {
				eye_phi = FloatConfig("EyePhiMax");
				 LOG("Demon",2,"Eye range limit met.");
			}
			else if ( eye_phi < FloatConfig( "EyePhiMin" ) )  {
				eye_phi = FloatConfig("EyePhiMin");
				 LOG("Demon",2,"Eye range limit met.");
			}
					
			Set( EYE_THETA, eye_theta);
			Set( EYE_PHI  , eye_phi  );
			
		} 
		else if ( action == MESSAGE ) {
			send( parameters[1], "all" );
		}
		else if ( action == GOTO ) {
			LOG( "Demon::action",0, "TODO! Goto with arg " + parameters[1]);
			char* targetObjName = (char*)parameters[1].c_str();

			iMeshWrapper*  targetMesh = iEngine::Instance().FindMeshObject(targetObjName);
              if ( targetMesh != NULL ) {
		csVector3 pos = targetMesh->GetMovable()->GetPosition();
		printf("SetPos: %.3f, %.3f, %.3f\n", pos.x, pos.y, pos.z);
		LOG("CSAgent", 2, "SetPos: " + toString(pos.x) + ", " + toString(pos.y) + ", " + toString(pos.z));
		getRepresentation()->setPosition( pos.x-0.4, pos.y-0.4, pos.z-0.4 );
              } 
              else  {
               LOG( "Command: goto", 1, "ERROR: Could not find object named "+parameters[1]);
              }
		}
		else {
			LOG( "Demon::action", 2, "Unknown or unimplemented action type " << action );
		}
	} catch( string s ) { LOG( "Demon::action", 1, "Exception: "+s ); }
	catch(...) { LOG( "Demon::action", 1, "Unknown exception" ); }
	
	 LOG( "Demon::action", 3, "Action handling ok." );

	onAction();
	
    return PARSE_OK;
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::onAction()
{
    printf("Demon::onAction()\n");
	UpdateView       ();
	ForceSensorUpdate();
}

//-----------------------------------------------------------------------------------------------------------------------
int Demon::send( std::string  data, std::string  destination)
{
	 LOG( "Demon::send", 0, data + " => " + destination );		
	return 0;
}

//-----------------------------------------------------------------------------------------------------------------------
void Demon::AddSensation( CustomSensation  sensation ) const
{
    boost::mutex::scoped_lock  lock( lock_provider );  
    customSensationQueue.push( sensation );
}

