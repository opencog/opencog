/**

Copyright Ari A. Heljakka / GenMind Ltd.
Not part of AGI-Sim.
Do not distribute.

*/

/***************************************************************************
 *  CS operations server-side implementation.
 * 
 *  Project: Novamente
 *
 ****************************************************************************/

#include "simcommon.h"
#include "space.h"

#if CRYSTAL

#include "demon.h"
#include "CSproxy.h"
#include "CSserver.h"
#include "simserver.h"
#include "sensors.h"

csRef<iCollideSystem>  CSserver::cd_sys = NULL;


//-----------------------------------------------------------------------------------------------------------------
map< string, meshData >  CSBox::GetMovableVisibleObjects( CSAgent*  csagent1 )
{
	iGraphics2D*  g2d = g3d->GetDriver2D();
  	int  w = g2d->GetWidth  ();
	int  h = g2d->GetHeight ();
	
	// Define Field-of-eatables:	
	int  sx = w / 4;
	int  sy = h / 4;
	int  ex = w - sx;
	int  ey = h - sy;
	int  netsize = IntConfig( "ObjectVisionResolution" );

	iCamera  *cam = csagent1->getView()->GetCamera();

	map< int, map<int, meshData> >  objs     = GetVisibleObjects( cam, sx, sy, ex, ey, netsize);
	set< std::string >              movables = TheSimWorld.GetMovableNames();
	map< string, meshData >         current_edibles;
	
	for (map< int, map<int, meshData> > :: iterator  x = objs.begin(); x!=objs.end(); x++)  {
		for (map<int, meshData >::iterator y = x->second.begin(); y != x->second.end(); y++)  {
			if ( y->second.wrapper )  {
				string  oname = ( y->second.wrapper->QueryObject() ? y->second.wrapper->QueryObject()->GetName() : "<null>" );
#ifdef WIN32
				#pragma message ( "1 agent soln" )
#else
				#warning "1 agent soln"						
#endif
				if ( STLhas( movables, oname )  &&  oname != AGENT_NAME )
					current_edibles[oname] = ( y->second );
			}
		}
	}
	return current_edibles;
}


//--------------------------------------------------------------------------------------------------------------
CSproxy* CSproxy::Get()
{
	if ( !implementation ) implementation = new CSserver();
	return implementation;
}

//--------------------------------------------------------------------------------------------------------------
void CSbox::Set2DPanel(void* panel)
{
	//Only for wx use.
}

//--------------------------------------------------------------------------------------------------------------
/*bool CSserver::MyEventHandler (iEvent& ev)
{
	CSbox::MyEventHandler(ev);
}*/

//--------------------------------------------------------------------------------------------------------------
bool CSserver::InitcsPlugins(iGUIProvider* _GUIprovider ) 
{
	if ( !CSbox::InitcsPlugins (_GUIprovider) )  return false;	
	LoadCollisionDetector();  
  
	return true;
}

//--------------------------------------------------------------------------------------------------------------
bool CSserver::LoadCollisionDetector()
{
	csRef<iPluginManager>  plugmgr =	CS_QUERY_REGISTRY (object_reg, iPluginManager);
	csRef<iConfigManager>  config  =	CS_QUERY_REGISTRY (object_reg, iConfigManager);
	const char*  p = config->GetStr ("MyGame.Settings.CollDetPlugin", "crystalspace.collisiondetection.opcode");
	 LOG("CSbox",0, "Collision Detection plugin is " + string (p));
	cd_sys = CS_LOAD_PLUGIN (plugmgr, p, iCollideSystem);
	
	if ( !cd_sys )	{
  		 LOG("CSbox",0, "No Collision Detection plugin found!");
	  	return false;
	}
	//	cd_sys->SetOneHitOnly (true); //Setting this true will cause segm. fault?
	//	cd_sys->ResetCollisionPairs();
	
	return true;
}

void CSserver::DrawViews()
{
  view = SimServer::Get()->GetDemon(0)->getView();
  
  if ( IntConfig("DrawGUI") )	{
    LOG("CSbox",4, "G3D Drawing...");		    
    if (!g3d->BeginDraw (engine->GetBeginDrawFlags () | CSDRAW_3DGRAPHICS | CSDRAW_CLEARZBUFFER | CSDRAW_CLEARSCREEN))
      return;
    LOG("CSbox",4, "View drawing...");  	  
    // Tell the camera to render into the frame buffer.
    
    // if animation is going...
    if (IntConfig("ForceAnimatedAgentView")) {
        shared_ptr< CSAgent >  agent = SimServer::Get()->GetAgent(0);
        if (agent && agent->GetAnimationControl().AnimationActive()) 
        {
	    csRef<iCamera> cam = view->GetCamera();
    	    csRef<iMeshWrapper> body =  SimServer::Get()->GetAgent(0)->getCSBody();
	    csVector3 campos = body->GetMovable()->GetPosition();
	    campos += csVector3(FloatConfig("CameraRelativeX"),
			        FloatConfig("CameraRelativeY"),
			        FloatConfig("CameraRelativeZ"));
	    cam->GetTransform().SetT2O(body->GetMovable()->GetTransform().GetO2T());
	    cam->GetTransform().SetOrigin(campos);
        }
    }
    
    view->Draw();
    LOG("CSbox",4, "View Draw ok.");  
  }
  

  if (IntConfig("ShowTopView"))
    {
      static csRef<iView> top_view(NULL);
      csVector3 conf_pos(FloatConfig("TopViewCamX"),
			 FloatConfig("TopViewCamY"),
			 FloatConfig("TopViewCamZ"));
      csXRotMatrix3 conf_rot_x(FloatConfig("TopViewRotX"));
      csYRotMatrix3 conf_rot_y(FloatConfig("TopViewRotY"));
      if (!top_view)
	{
	  LOG("CSserver", 1, "Top view creation");
	  top_view = csPtr<iView>(new csView (FROM_CS(iEngine), 
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
	  cam->SetPerspectiveCenter(width/2.0, g2d->GetHeight()-height/2.0);
	  cam->GetTransform().Identity();
	  cam->GetTransform().SetO2T(conf_rot_x*conf_rot_y);
	  cam->GetTransform().SetOrigin(conf_pos);
	  LOG("CSserver", 1, "Top view creation done");
	}
      else
	{
	  g3d->BeginDraw (engine->GetBeginDrawFlags () | CSDRAW_3DGRAPHICS | CSDRAW_CLEARZBUFFER);
	  LOG("CSserver", 4, "Top view draw");
	  
	  if (IntConfig("TopViewFollowAgent"))
	    {
	      csRef<iCamera> cam = top_view->GetCamera();
	      csBox3 box;
	      box = SimServer::Get()->GetAgent(0)->getCSBody()->GetWorldBoundingBox();
	      csVector3 cam_pos = box.GetCenter();
	      //cam_pos = view->GetCamera()->GetTransform().GetOrigin();
	      cam_pos += conf_pos;
	      cam->GetTransform().SetOrigin(cam_pos);
	    }
	  top_view->Draw();
	}
    }
}


//--------------------------------------------------------------------------------------------------------------
bool CSserver::SetupFrame()
{
  if (IntConfig("EnableAnimation"))
    {
      if (SimServer::Get()->GetAgent(0))
	{
	  if (SimServer::Get()->GetAgent(0)->GetAnimationControl().AnimationActive())
	    {
	      CSanimation & ac = SimServer::Get()->GetAgent(0)->GetAnimationControl();
	      float time = FROM_CS(iVirtualClock)->GetElapsedTicks()/1000.0f;
	      ac.UpdateAnimation(time);
	      DrawViews();
	      std::cout<<"Animaiton updated!\n";
	      return true;
	    }
	}
    }

	 LOG("CSbox",4, "SetupFrame");
	
	// Enable command processing on other threads by protecting the CS pump by this mutex.
	boost::mutex::scoped_lock serverlock( SimServer::Get()->AcquireLock() );
	LOG("CSbox",4, "SetupFrame passed server LOCK");
	
#ifdef WIN32
	#pragma message ("1 agent soln")
#else
	#warning "1 agent soln"
#endif

	if ( !SimServer::Get()->GetDemon(0) )  {
	  serverlock.unlock();
		SimServer::Get()->OnIdleEvent();		
		 LOG("CSbox",4, "SetupFrame ending. Was idle.");
		return true;
	}

	//GUIprovider->OnSetupFrameBegin();
#ifdef WIN32
	#pragma message ("Hacky")
#else
	#warning "Hacky"	
#endif

	DrawViews();
  
	GUIprovider->OnSetupFrameEnd();
	  
	 LOG("CSbox",4, "Idle event...");
	serverlock.unlock();

	  SimServer::Get()->OnIdleEvent();
	 LOG("CSbox",4, "SetupFrame ok.");

	return true;
}

#ifdef WIN32
	#pragma message ("Explicitly passed worldURL is deprecated")
#else
	#warning "Explicitly passed worldURL is deprecated"
#endif

//--------------------------------------------------------------------------------------------------------------
bool CSserver::ConnectToWorld(const char* worldURL)
{
	return SimServer::Get()->ConnectToWorld();
}

//--------------------------------------------------------------------------------------------------------------
bool CSserver::DisconnectFromWorld()
{
	return SimServer::Get()->DisconnectFromWorld();
}

//--------------------------------------------------------------------------------------------------------------
csRef<iCollideSystem> CSserver::GetColliderSystem() 
{ 
	return  cd_sys; 
}

//--------------------------------------------------------------------------------------------------------------
/*bool CSserver::InitEventHandler()
{
  return csInitializer::SetupEventHandler (object_reg, MyEventHandler);
}
*/

#endif
